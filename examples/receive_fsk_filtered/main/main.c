#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <inttypes.h>
#include <sx127x.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";
sx127x device;

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_util_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_auto(true, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_bandwidth(5000.0, &device));
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x00, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, &device));

  sx127x_rx_set_callback(rx_callback, &device, &device);

  ESP_ERROR_CHECK(setup_task(&device));

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t) DIO0, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO1, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO2, &device, GPIO_INTR_POSEDGE);

  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_FSK, &device));
}
