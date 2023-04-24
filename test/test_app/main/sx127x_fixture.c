#include "sx127x_fixture.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_err.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define ERROR_CHECK(x)           \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return __err_rc;           \
    }                            \
  } while (0)

static const char *TAG = "sx127x_test";

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xTaskResumeFromISR(((sx127x_fixture_t *)arg)->handle_interrupt);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    vTaskSuspend(NULL);
    sx127x_handle_interrupt((sx127x *)arg);
  }
}

void sx127x_fixture_rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int16_t rssi;
  ESP_ERROR_CHECK(sx127x_rx_get_packet_rssi(device, &rssi));
  float snr;
  ESP_ERROR_CHECK(sx127x_lora_rx_get_packet_snr(device, &snr));
  int32_t frequency_error;
  ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  ESP_LOGI(TAG, "received: %d %s rssi: %d snr: %f freq_error: %" PRId32, data_length, payload, rssi, snr, frequency_error);
}

void cad_callback(sx127x *device, int cad_detected) {
  if (cad_detected == 0) {
    ESP_LOGI(TAG, "cad not detected");
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
    return;
  }
  // put into RX mode first to handle interrupt as soon as possible
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  ESP_LOGI(TAG, "cad detected\n");
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x_fixture_t *fixture) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *)fixture);
}

int sx127x_fixture_create(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture) {
  ESP_LOGI(TAG, "starting up");
  spi_bus_config_t bus_config = {
      .mosi_io_num = config->mosi,
      .miso_io_num = config->miso,
      .sclk_io_num = config->sck,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));
  spi_device_interface_config_t dev_config = {
      .clock_speed_hz = 3000000,
      .spics_io_num = config->ss,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  spi_device_handle_t spi_device;
  sx127x *device = NULL;
  ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &spi_device));
  ERROR_CHECK(sx127x_create(spi_device, &device));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  ERROR_CHECK(sx127x_set_frequency(437200012, device));
  ERROR_CHECK(sx127x_lora_reset_fifo(device));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device));
  ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, device));
  ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, device));
  ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, device));
  ERROR_CHECK(sx127x_lora_set_syncword(18, device));
  ERROR_CHECK(sx127x_set_preamble_length(8, device));
  sx127x_lora_cad_set_callback(cad_callback, device);

  sx127x_fixture_t *result = malloc(sizeof(sx127x_fixture_t));
  if (result == NULL) {
    return SX127X_ERR_NO_MEM;
  }
  *result = (sx127x_fixture_t){0};
  result->device = device;
  result->tx_done = xSemaphoreCreateBinary();
  result->rx_done = xSemaphoreCreateBinary();
  result->cad_done = xSemaphoreCreateBinary();
  result->spi_device = spi_device;

  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, device, 2, &(result->handle_interrupt), xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    sx127x_fixture_destroy(result);
    return SX127X_ERR_INVALID_ARG;
  }
  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)config->dio0, result);

  *fixture = result;
  return SX127X_OK;
}

void sx127x_fixture_destroy(sx127x_fixture_t *fixture) {
  if (fixture == NULL) {
    return;
  }
  if (fixture->device != NULL) {
    sx127x_destroy(fixture->device);
    fixture->device = NULL;
  }
  gpio_uninstall_isr_service();
  if( fixture->spi_device != NULL ) {
    spi_bus_remove_device(fixture->spi_device);
  }
  spi_bus_free(HSPI_HOST);
  vTaskDelete(fixture->handle_interrupt);
  free(fixture);
}