#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <sx127x.h>

// TTGO lora32 v2.1 1.6.1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26
// older versions of TTGO require manual wiring of pins below
#define DIO1 33
#define DIO2 32

// Heltec lora32 v2
// #define DIO1 35
// #define DIO2 34

static const char *TAG = "sx127x";

sx127x device;
int messages_sent = 0;
static SemaphoreHandle_t xBinarySemaphore;
static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
      sx127x_handle_interrupt((sx127x *) arg);
    }
  }
}

void tx_callback(sx127x *device) {
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
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  ESP_LOGI(TAG, "transmitting");
  messages_sent++;
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device, gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *) device);
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t) RST, GPIO_MODE_INPUT_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 0));
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 1));
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "sx127x was reset");
  ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t) RST));

  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &config, 1));
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 4E6,
      .spics_io_num = SS,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  spi_device_handle_t spi_device;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device));
  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, &device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(4, &device));
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_SCRAMBLED, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_FIXED, 2047, &device));
  ESP_ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, &device));
  ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, &device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, &device));

  sx127x_tx_set_callback(tx_callback, &device);

  xBinarySemaphore = xSemaphoreCreateBinary();
  if (xBinarySemaphore == NULL) {
    ESP_LOGE(TAG, "unable to create semaphore");
    return;
  }

  TaskHandle_t handle_interrupt;
  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196 * 2, &device, 2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    return;
  }

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t) DIO0, &device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO1, &device, GPIO_INTR_NEGEDGE);
  setup_gpio_interrupts((gpio_num_t) DIO2, &device, GPIO_INTR_POSEDGE);

  tx_callback(&device);
}
