#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
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
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_OOK, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_OOK, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_auto(true, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_bandwidth(5000.0, &device));
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, &device));
  ESP_ERROR_CHECK(sx127x_ook_set_data_shaping(SX127X_1_BIT_RATE, SX127X_PA_RAMP_10, &device));
  ESP_ERROR_CHECK(sx127x_ook_rx_set_peak_mode(SX127X_0_5_DB, 0x0C, SX127X_1_1_CHIP, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_calibrate(&device));

  sx127x_rx_set_callback(rx_callback, &device, &device);

  ESP_ERROR_CHECK(setup_task(&device));

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO1, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO2, &device, GPIO_INTR_POSEDGE);

  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_OOK, &device));
}
