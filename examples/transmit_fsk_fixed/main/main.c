#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <sx127x.h>
#include <esp_utils.h>
#define PAYLOAD_SIZE 2047

static const char *TAG = "sx127x";
sx127x device;
int messages_sent = 0;

void tx_callback(void *ctx) {
  sx127x *device = (sx127x *)ctx;
  if (messages_sent > 0) {
    ESP_LOGI(TAG, "transmitted");
  }
  if (messages_sent == 0) {
    uint16_t data_length = 2047;
    uint8_t *data = malloc(sizeof(uint8_t) * data_length);
    if (data == NULL) {
      ESP_LOGE(TAG, "unable to initialize data to send: no memory");
      return;
    }
    for (int i = 0; i < 2047; i++) {
      data[i] = (i % 10);
    }
    ESP_ERROR_CHECK(sx127x_fsk_ook_tx_set_for_transmission(data, data_length, device));
    free(data);
  } else {
    // FSK mode require manual switch from TX to Standby
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_FSK, device));
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_FSK, device));
  ESP_LOGI(TAG, "transmitting");
  messages_sent++;
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(4, &device));
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_SCRAMBLED, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_FIXED, 2047, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, &device));
  ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127X_PA_PIN_BOOST, 4, &device));

  sx127x_tx_set_callback(tx_callback, &device, &device);

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t) DIO0, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO1, &device, GPIO_INTR_NEGEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO2, &device, GPIO_INTR_POSEDGE);

  ESP_ERROR_CHECK(setup_tx_task(&device, tx_callback));
}
