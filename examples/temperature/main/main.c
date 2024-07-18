#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <sx127x.h>
#include <rom/ets_sys.h>

// TTGO lora32 v2.1 1.6.1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26
// older versions of TTGO require manual wiring of pins below
#define DIO1 33
#define DIO2 32

// Heltec lora32 v2
// #define DIO1 35
// #define DIO2 34

static const char *TAG = "sx127x";

sx127x *device = NULL;

void app_read_temperature() {
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_FSRX, SX127x_MODULATION_FSK, device));
  //enable temp monitoring
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_temp_monitor(true, device));
  // a little bit longer for FSRX mode to kick off
  ets_delay_us(150);
  //disable temp monitoring
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_temp_monitor(false, device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));

  int8_t raw_temperature;
  ESP_ERROR_CHECK(sx127x_fsk_ook_get_raw_temperature(device, &raw_temperature));
  ESP_LOGI(TAG, "raw temperature: %d", raw_temperature);
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
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
  spi_device_handle_t spi_device;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device));
  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));

  while (1) {
    app_read_temperature();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
