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
#include <string.h>

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

sx127x *device = NULL;
uart_at_handler_t *handler = NULL;
const char *TAG = "sx127x_at";

static void uart_rx_task(void *arg) {
  uart_at_handler_process(handler);
}

void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
  sx127x *device = (sx127x *) ctx;
  int32_t frequency_error = 0;
  ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  int16_t rssi = 0;
  sx127x_rx_get_packet_rssi(device, &rssi);
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

void app_main(void) {
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
  ESP_ERROR_CHECK(uart_at_handler_create(device, &handler));
  xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 4, handler, configMAX_PRIORITIES - 1, NULL);
}