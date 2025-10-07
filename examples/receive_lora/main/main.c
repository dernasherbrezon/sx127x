#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <sx127x.h>
#include <inttypes.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";
sx127x device;

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_util_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
  ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
  ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127X_LNA_GAIN_G4, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127X_BW_125000, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(SX127X_SF_9, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));
  sx127x_rx_set_callback(lora_rx_callback, &device, &device);
  sx127x_lora_cad_set_callback(cad_callback, &device, &device);

  ESP_ERROR_CHECK(setup_task(&device));

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);

  ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &device));
}
