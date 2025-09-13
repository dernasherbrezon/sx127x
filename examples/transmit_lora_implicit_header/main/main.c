#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/semphr.h>
#include <sx127x.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";
int messages_sent = 0;
sx127x device;

void tx_callback(sx127x *device) {
  if (messages_sent > 0) {
    ESP_LOGI(TAG, "transmitted");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  if (messages_sent == 0) {
    uint8_t data[] = {0xCA, 0xFE};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), device));
  } else {
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device));
  ESP_LOGI(TAG, "transmitting");
  messages_sent++;
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, &device));
  sx127x_implicit_header_t header = {
      .coding_rate = SX127x_CR_4_5,
      .enable_crc = true,
      .length = 2};
  ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(&header, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));
  sx127x_tx_set_callback(tx_callback, &device);

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);
  // 4 is OK
  ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, &device));

  ESP_ERROR_CHECK(setup_tx_task(&device, tx_callback));
}
