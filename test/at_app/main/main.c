#include <sx127x_at.h>
#include <sx127x.h>
#include <esp_log.h>
#include <driver/rtc_io.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "uart_at.h"
#include "at_util.h"
#include <string.h>

#define MAX_INPUT_LEN 256
#define MAX_PARAM_COUNT 10

// TTGO lora32 v2.1 1.6.1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define DIO0 26
#define RST 23
// older versions of TTGO require manual wiring of pins below
#define DIO1 33
#define DIO2 32

#define ERROR_CHECK(x)           \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return __err_rc;           \
    }                            \
  } while (0)

sx127x *device = NULL;
uart_at_handler_t *handler = NULL;
at_util_vector_t *frames;
const char *TAG = "sx127x_at";
const UBaseType_t xArrayIndex = 0;
TaskHandle_t handle_interrupt;

static void uart_rx_task(void *arg) {
  uart_at_handler_process(handler);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    if (ulTaskNotifyTakeIndexed(xArrayIndex, pdTRUE, portMAX_DELAY) > 0) {
      sx127x_handle_interrupt(device);
    }
  }
}

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(handle_interrupt, xArrayIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup_gpio_interrupts(gpio_num_t gpio, gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, NULL);
}

void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
  sx127x *device = (sx127x *) ctx;
  sx127x_frame_t *result = malloc(sizeof(sx127x_frame_t));
  if (result == NULL) {
    return;
  }
  result->data_length = data_length;
  result->data = malloc(sizeof(uint8_t) * result->data_length);
  if (result->data == NULL) {
    sx127x_util_frame_destroy(result);
    return;
  }
  memcpy(result->data, data, sizeof(uint8_t) * result->data_length);
  int32_t frequency_error;
  esp_err_t code = sx127x_rx_get_frequency_error(device, &frequency_error);
  if (code == ESP_OK) {
    result->frequency_error = frequency_error;
  } else {
    ESP_LOGE(TAG, "unable to get frequency error: %s", esp_err_to_name(code));
    result->frequency_error = 0;
  }
  int16_t rssi;
  code = sx127x_rx_get_packet_rssi(device, &rssi);
  if (code == ESP_OK) {
    result->rssi = rssi;
  } else {
    ESP_LOGE(TAG, "unable to get rssi: %s", esp_err_to_name(code));
    // No room for proper "NULL". -255 should look suspicious
    result->rssi = -255;
  }
  // do not supported yet
  result->timestamp = 0;
  at_util_vector_add(result, frames);
  ESP_LOGI(TAG, "received: %d rssi: %d freq error: %ld", data_length, rssi, frequency_error);
}

void tx_callback(void *ctx) {
  const char *output = "OK\r\n";
  uart_at_handler_send((char *) output, strlen(output), handler);
}

static void reset() {
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t) RST, GPIO_MODE_INPUT_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 0));
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 1));
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "sx127x was reset");
  ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t) RST));
}

static int split_params(char *input, char *params[]) {
  int count = 0;
  char *token = strtok(input, ",");
  while (token && count < MAX_PARAM_COUNT) {
    params[count++] = token;
    token = strtok(NULL, ",");
  }
  return count;
}

static sx127x_modulation_t parse_modulation(const char *str) {
  if (strcmp(str, "LORA") == 0) return SX127x_MODULATION_LORA;
  if (strcmp(str, "FSK") == 0) return SX127x_MODULATION_FSK;
  if (strcmp(str, "OOK") == 0) return SX127x_MODULATION_OOK;
  return -1; // Invalid
}

static int extra_at_handler_impl(sx127x *device, const char *input, char *output, size_t output_len) {
  // Check for AT+ prefix
  if (strncmp(input, "AT+", 3) != 0) {
    return SX127X_ERR_INVALID_ARG;
  }

  // Copy input to avoid modifying the original
  char cmd[MAX_INPUT_LEN];
  strncpy(cmd, input + 3, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Split command and parameters
  char *cmd_name = strtok(cmd, "=");
  char *param_str = strtok(NULL, "=");
  bool is_query = (cmd_name && cmd_name[strlen(cmd_name) - 1] == '?');
  if (is_query) {
    cmd_name[strlen(cmd_name) - 1] = '\0'; // Remove '?'
  }

  char *params[MAX_PARAM_COUNT];
  int param_count = 0;
  if (param_str) {
    param_count = split_params(param_str, params);
  }

  if (strcmp(cmd_name, "RESET") == 0) {
    if (is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    reset();
    snprintf(output, output_len, "OK\r\n");
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "TX") == 0) {
    if (is_query || param_count != 1) {
      return SX127X_ERR_INVALID_ARG;
    }
    sx127x_modulation_t modulation = parse_modulation(params[0]);
    //TODO change dio mapping direction for fsk
    ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, modulation, device));
    // do not send OK here. Send it from tx_callback
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "PULL") == 0) {
    if (!is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    char message[2048];
    char formatted[2512];
    size_t formatted_length = 2512;
    //FIXME lock access
    for (size_t i = 0; i < at_util_vector_size(frames); i++) {
      sx127x_frame_t *cur_frame = NULL;
      at_util_vector_get(i, (void *) &cur_frame, frames);
      int code = at_util_hex2string(cur_frame->data, cur_frame->data_length, message);
      if (code == 0) {
        snprintf(formatted, formatted_length, "%s,%d,%g,%" PRId32 ",%" PRIu64 "\r\n", message, cur_frame->rssi, cur_frame->snr, cur_frame->frequency_error, cur_frame->timestamp);
        uart_at_handler_send(formatted, strlen(formatted), handler);
      }
      sx127x_util_frame_destroy(cur_frame);
    }
    at_util_vector_clear(frames);
    snprintf(formatted, formatted_length, "OK\r\n");
    uart_at_handler_send(formatted, strlen(formatted), handler);
    return SX127X_OK;
  }

  return SX127X_CONTINUE;
}

static int extra_at_handler(sx127x *device, const char *input, char *output, size_t output_len) {
  int result = extra_at_handler_impl(device, input, output, output_len);
  if (result == SX127X_ERR_INVALID_ARG) {
    snprintf(output, output_len, "invalid argument\r\nERROR\r\n");
  } else if (result == SX127X_CONTINUE || result == SX127X_OK) {
    // do nothing
  } else {
    snprintf(output, output_len, "operation failed %d\r\nERROR\r\n", result);
  }
  return result;
}

void app_main(void) {
  ESP_ERROR_CHECK(at_util_vector_create(&frames));
  spi_bus_config_t bus_config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, 1));
  spi_device_interface_config_t dev_config = {
      .clock_speed_hz = 3000000,
      .spics_io_num = SS,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  device = malloc(sizeof(struct sx127x_t));
  if (device == NULL) {
    return;
  }
  spi_device_handle_t spi_device;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &spi_device));
  ESP_ERROR_CHECK(sx127x_create(spi_device, device));
  sx127x_rx_set_callback(rx_callback, device, device);
  sx127x_tx_set_callback(tx_callback, device, device);
  reset();
  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t) DIO0, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO1, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO2, GPIO_INTR_POSEDGE);
  ESP_ERROR_CHECK(uart_at_handler_create(extra_at_handler, device, &handler));
  xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 8, handler, configMAX_PRIORITIES - 1, NULL);
  xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, NULL, 2, &handle_interrupt, xPortGetCoreID());
}