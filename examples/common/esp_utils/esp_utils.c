#include "esp_utils.h"
#include <esp_log.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

int total_packets_received = 0;
const UBaseType_t xArrayIndex = 0;
TaskHandle_t handle_interrupt;
static const char *TAG = "esp_utils";

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(handle_interrupt, xArrayIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    if (ulTaskNotifyTakeIndexed(xArrayIndex, pdTRUE, portMAX_DELAY) > 0) {
      sx127x_handle_interrupt((sx127x *) arg);
    }
  }
}

void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int32_t frequency_error;
  ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  int16_t rssi;
  int code = sx127x_rx_get_packet_rssi(device, &rssi);
  if (code == SX127X_ERR_NOT_FOUND) {
    ESP_LOGI(TAG, "received: %d %s freq_error: %" PRId32, data_length, payload, frequency_error);
  } else {
    ESP_LOGI(TAG, "received: %d %s rssi: %d freq_error: %" PRId32, data_length, payload, rssi, frequency_error);
  }
  total_packets_received++;
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device, gpio_int_type_t type) {
  ESP_ERROR_CHECK(gpio_set_direction(gpio, GPIO_MODE_INPUT));
  ESP_ERROR_CHECK(gpio_pulldown_en(gpio));
  ESP_ERROR_CHECK(gpio_pullup_dis(gpio));
  ESP_ERROR_CHECK(gpio_set_intr_type(gpio, type));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *) device));
}

esp_err_t setup_task(sx127x *device) {
  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, device, 2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    return ESP_FAIL;
  }
  return ESP_OK;
}

void sx127x_reset() {
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t) RST, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 0));
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 1));
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "sx127x was reset");
}

void sx127x_init_spi(spi_device_handle_t *handle) {
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &config, 1));
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 4E6,
      .spics_io_num = SS,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, handle));
}