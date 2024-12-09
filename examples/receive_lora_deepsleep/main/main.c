#include <driver/rtc_io.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <sx127x.h>
#include <inttypes.h>
#include <esp_utils.h>

sx127x device;
static const char *TAG = "sx127x";

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));

  esp_sleep_wakeup_cause_t cpu0WakeupReason = esp_sleep_get_wakeup_cause();
  if (cpu0WakeupReason == ESP_SLEEP_WAKEUP_EXT0) {
    sx127x_rx_set_callback(lora_rx_callback, &device);
    sx127x_handle_interrupt(&device);
  } else {
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, &device));
    ESP_ERROR_CHECK(sx127x_set_frequency(437200012, &device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, &device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, &device));
  }

  ESP_ERROR_CHECK(rtc_gpio_set_direction((gpio_num_t)DIO0, RTC_GPIO_MODE_INPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_pulldown_en((gpio_num_t)DIO0));
  ESP_LOGI(TAG, "entering deep sleep");
  ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup((gpio_num_t)DIO0, 1));
  esp_deep_sleep_start();
}
