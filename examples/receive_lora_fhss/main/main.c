#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <sx127x.h>
#include <inttypes.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";

sx127x device;
uint64_t frequencies[] = {437700000, 438200000, 437200012};

void lora_fhss_rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, device));
  lora_rx_callback(device, data, data_length);
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_frequency_hopping(5, frequencies, sizeof(frequencies) / sizeof(uint64_t), &device));
  ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
  ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, &device));
  ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, &device));
  ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));
  sx127x_rx_set_callback(lora_fhss_rx_callback, &device);

  ESP_ERROR_CHECK(setup_task(&device));

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO1, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO2, &device, GPIO_INTR_POSEDGE);

  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, &device));
}
