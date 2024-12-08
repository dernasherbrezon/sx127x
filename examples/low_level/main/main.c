#include <driver/spi_common.h>
#include <esp_log.h>
#include <sx127x.h>
#include <sx127x_registers.h>
#include <esp_utils.h>

static const char *TAG = "sx127x";

sx127x device;

void app_main() {
  ESP_LOGI(TAG, "starting up");
  sx127x_reset();

  spi_device_handle_t spi_device;
  sx127x_init_spi(&spi_device);

  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));

  uint8_t value = 0b01100111;
  ESP_ERROR_CHECK(sx127x_write_register(REGINVERTIQ, value, &device.spi_device));
  printf("written: %d\n", value);
  uint8_t written = 0;
  ESP_ERROR_CHECK(sx127x_read_register(REGINVERTIQ, &device.spi_device, &written));
  printf("read: %d\n", written);
}
