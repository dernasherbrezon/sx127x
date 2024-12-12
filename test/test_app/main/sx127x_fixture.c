#include "sx127x_fixture.h"

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
const UBaseType_t xArrayIndex = 0;

void IRAM_ATTR

handle_interrupt_fromisr(void *arg) {
  sx127x_fixture_t *fixture = (sx127x_fixture_t *) arg;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(fixture->handle_interrupt, xArrayIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void handle_interrupt_task(void *arg) {
  sx127x_fixture_t *fixture = (sx127x_fixture_t *) arg;
  while (1) {
    if (ulTaskNotifyTakeIndexed(xArrayIndex, pdTRUE, portMAX_DELAY) > 0) {
      xSemaphoreTake(fixture->xMutex, portMAX_DELAY);
      sx127x_handle_interrupt(fixture->device);
      xSemaphoreGive(fixture->xMutex);
    }
  }
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x_fixture_t *fixture, gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *) fixture);
}

int sx127x_fixture_create_base(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture) {
  ESP_LOGI(TAG, "starting up");
  sx127x_fixture_t *result = malloc(sizeof(sx127x_fixture_t));
  if (result == NULL) {
    return -1;
  }
  *result = (sx127x_fixture_t) {0};
  *fixture = result;

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
  result->device = malloc(sizeof(struct sx127x_t));
  if (result->device == NULL) {
    return -1;
  }
  ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &result->spi_device));
  ERROR_CHECK(sx127x_create(result->spi_device, result->device));

  result->tx_done = xSemaphoreCreateBinary();
  result->rx_done = xSemaphoreCreateBinary();
  return SX127X_OK;
}

int sx127x_fixture_create(sx127x_fixture_config_t *config, sx127x_modulation_t modulation, sx127x_fixture_t **fixture) {
  ERROR_CHECK(gpio_set_direction((gpio_num_t) config->rst, GPIO_MODE_INPUT_OUTPUT));
  ERROR_CHECK(gpio_set_level((gpio_num_t) config->rst, 0));
  vTaskDelay(pdMS_TO_TICKS(5));
  ERROR_CHECK(gpio_set_level((gpio_num_t) config->rst, 1));
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "sx127x was reset");
// it looks like if leave this pin "HIGH" then interrupt in deep sleep mode won't be generated
  ERROR_CHECK(gpio_reset_pin((gpio_num_t) config->rst));

  ERROR_CHECK(sx127x_fixture_create_base(config, fixture));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, modulation, (*fixture)->device));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, modulation, (*fixture)->device));
  ERROR_CHECK(sx127x_set_frequency(437200000, (*fixture)->device));
  ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G6, (*fixture)->device));
  ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, (*fixture)->device));
  if (modulation == SX127x_MODULATION_LORA) {
    ERROR_CHECK(sx127x_lora_reset_fifo((*fixture)->device));
    ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, (*fixture)->device));
    ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, (*fixture)->device));
    ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, (*fixture)->device));
    ERROR_CHECK(sx127x_lora_set_syncword(18, (*fixture)->device));
    ERROR_CHECK(sx127x_set_preamble_length(8, (*fixture)->device));
  } else if (modulation == SX127x_MODULATION_FSK) {
    ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_auto(true, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_bandwidth(10000.0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_bandwidth(5000.0, (*fixture)->device));
    uint8_t syncWord[] = {0x12, 0xAD};
    ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_PREAMBLE, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, (*fixture)->device));
  } else if (modulation == SX127x_MODULATION_OOK) {
    ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_AUTO, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_bitrate(1200.0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_auto(false, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_bandwidth(5000.0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_bandwidth(2400.0, (*fixture)->device));
    uint8_t syncWord[] = {0x12, 0xAD};
    ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, (*fixture)->device));
    ERROR_CHECK(sx127x_ook_set_data_shaping(SX127X_1_BIT_RATE, SX127X_PA_RAMP_10, (*fixture)->device));
    ERROR_CHECK(sx127x_ook_rx_set_peak_mode(SX127X_0_5_DB, 0x0C, SX127X_1_1_CHIP, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, (*fixture)->device));
    ERROR_CHECK(sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, (*fixture)->device));
  } else {
    return SX127X_ERR_INVALID_ARG;
  }

  (*fixture)->xMutex = xSemaphoreCreateMutex();
  if ((*fixture)->xMutex == NULL) {
    return SX127X_ERR_INVALID_ARG;
  }

  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, (*fixture), 2, &((*fixture)->handle_interrupt), xPortGetCoreID());
  if (task_code != pdPASS) {
    return SX127X_ERR_INVALID_ARG;
  }
  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t) config->dio0, (*fixture), GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) config->dio1, (*fixture), GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) config->dio2, (*fixture), GPIO_INTR_POSEDGE);


  return SX127X_OK;
}

int sx127x_fixture_fsk_ook_tx_set_for_transmission(const uint8_t *data, uint16_t data_length, sx127x_fixture_t *fixture) {
  xSemaphoreTake(fixture->xMutex, portMAX_DELAY);
  int code = sx127x_fsk_ook_tx_set_for_transmission(data, data_length, fixture->device);
  if (code != SX127X_OK) {
    xSemaphoreGive(fixture->xMutex);
    return code;
  }
  code = sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device);
  xSemaphoreGive(fixture->xMutex);
  return code;
}

int sx127x_fixture_fsk_ook_tx_set_for_transmission_with_address(const uint8_t *data, uint16_t data_length, uint8_t address_to, sx127x_fixture_t *fixture) {
  xSemaphoreTake(fixture->xMutex, portMAX_DELAY);
  int code = sx127x_fsk_ook_tx_set_for_transmission_with_address(data, data_length, address_to, fixture->device);
  if (code != SX127X_OK) {
    xSemaphoreGive(fixture->xMutex);
    return code;
  }
  code = sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device);
  xSemaphoreGive(fixture->xMutex);
  return code;
}

void sx127x_fixture_destroy(sx127x_fixture_t *fixture) {
  if (fixture == NULL) {
    return;
  }
  if (fixture->device != NULL) {
    free(fixture->device);
    fixture->device = NULL;
  }
  gpio_uninstall_isr_service();
  if (fixture->spi_device != NULL) {
    spi_bus_remove_device(fixture->spi_device);
  }
  spi_bus_free(HSPI_HOST);
  if (fixture->handle_interrupt != NULL) {
    vTaskDelete(fixture->handle_interrupt);
  }
  if (fixture->tx_done != NULL) {
    vSemaphoreDelete(fixture->tx_done);
  }
  if (fixture->rx_done != NULL) {
    vSemaphoreDelete(fixture->rx_done);
  }
  if (fixture->xMutex != NULL) {
    vSemaphoreDelete(fixture->xMutex);
  }
  free(fixture);
}