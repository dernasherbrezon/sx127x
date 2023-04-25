#include "sx127x_fixture.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_err.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define ERROR_CHECK(x)           \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return __err_rc;           \
    }                            \
  } while (0)

static const char *TAG = "sx127x_test";

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xTaskResumeFromISR(((sx127x_fixture_t *)arg)->handle_interrupt);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    vTaskSuspend(NULL);
    sx127x_handle_interrupt((sx127x *)arg);
  }
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x_fixture_t *fixture) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *)fixture);
}

int sx127x_fixture_create_base(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture) {
    ESP_LOGI(TAG, "starting up");
    spi_bus_config_t bus_config = {
            .mosi_io_num = config->mosi,
            .miso_io_num = config->miso,
            .sclk_io_num = config->sck,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
    };
    ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));
    spi_device_interface_config_t dev_config = {
            .clock_speed_hz = 3000000,
            .spics_io_num = config->ss,
            .queue_size = 16,
            .command_bits = 0,
            .address_bits = 8,
            .dummy_bits = 0,
            .mode = 0};
    spi_device_handle_t spi_device;
    sx127x *device = NULL;
    ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &spi_device));
    ERROR_CHECK(sx127x_create(spi_device, &device));

    sx127x_fixture_t *result = malloc(sizeof(sx127x_fixture_t));
    if (result == NULL) {
        return SX127X_ERR_NO_MEM;
    }
    *result = (sx127x_fixture_t){0};
    result->device = device;
    result->tx_done = xSemaphoreCreateBinary();
    result->rx_done = xSemaphoreCreateBinary();
    result->cad_done = xSemaphoreCreateBinary();
    result->spi_device = spi_device;
    result->handle_interrupt = NULL;

    *fixture = result;
    return SX127X_OK;
}

int sx127x_fixture_create(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture) {
    sx127x_fixture_t *result = NULL;
    ERROR_CHECK(sx127x_fixture_create_base(config, &result));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, result->device));
  ERROR_CHECK(sx127x_set_frequency(437200012, result->device));
  ERROR_CHECK(sx127x_lora_reset_fifo(result->device));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, result->device));
  ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, result->device));
  ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, result->device));
  ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, result->device));
  ERROR_CHECK(sx127x_lora_set_syncword(18, result->device));
  ERROR_CHECK(sx127x_set_preamble_length(8, result->device));

  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, result->device, 2, &(result->handle_interrupt), xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    sx127x_fixture_destroy(result);
    return SX127X_ERR_INVALID_ARG;
  }
  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)config->dio0, result);

  *fixture = result;
  return SX127X_OK;
}

void sx127x_fixture_destroy(sx127x_fixture_t *fixture) {
  if (fixture == NULL) {
    return;
  }
  if (fixture->device != NULL) {
    sx127x_destroy(fixture->device);
    fixture->device = NULL;
  }
  gpio_uninstall_isr_service();
  if( fixture->spi_device != NULL ) {
    spi_bus_remove_device(fixture->spi_device);
  }
  spi_bus_free(HSPI_HOST);
  if( fixture->handle_interrupt != NULL ) {
      vTaskDelete(fixture->handle_interrupt);
  }
  free(fixture);
}