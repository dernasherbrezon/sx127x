#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <sx127x.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";
sx127x device;

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(4, &device));
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, &device));
  ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, &device));

  uint8_t payload[] = {0xCA, 0xFE, 0x01, 0x02, 0xBE, 0xEF};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(payload), &device));
  ESP_LOGI(TAG, "sending beacons...");
  ESP_ERROR_CHECK(sx127x_fsk_ook_tx_start_beacon(payload, sizeof(payload), 1000, &device));
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK(sx127x_fsk_ook_tx_stop_beacon(&device));
  ESP_LOGI(TAG, "stopped");
}
