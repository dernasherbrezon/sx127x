#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <sx127x.h>

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

sx127x *device = NULL;

static const char *TAG = "sx127x";

int current_power = 0;
int messages_current_power = 0;
TaskHandle_t handle_interrupt;

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xTaskResumeFromISR(handle_interrupt);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    vTaskSuspend(NULL);
    sx127x_handle_interrupt((sx127x *)arg);
  }
}

void tx_callback(sx127x *device) {
  ESP_LOGI(TAG, "transmitted");
  if (current_power > 20) {
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_pa_config(SX127x_PA_PIN_BOOST, current_power, device));

  uint8_t data[] = {0xCA, 0xFE};
  ESP_ERROR_CHECK(sx127x_set_for_transmission(data, sizeof(data), device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device));
  ESP_LOGI(TAG, "transmitting %d %d", current_power, messages_current_power);
  messages_current_power++;
  if (messages_current_power > 5) {
    current_power++;
    messages_current_power = 0;
  }
  if (current_power == 18) {
    current_power = 20;
  }
}

void app_main() {
  ESP_LOGI(TAG, "starting up");
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &config, 1));
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 8E6,
      .spics_io_num = SS,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  spi_device_handle_t spi_device;
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_cfg, &spi_device));   
  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, device));
  ESP_ERROR_CHECK(sx127x_reset_fifo(device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device));
  ESP_ERROR_CHECK(sx127x_set_bandwidth(SX127x_BW_125000, device));
  ESP_ERROR_CHECK(sx127x_set_implicit_header(NULL, device));
  ESP_ERROR_CHECK(sx127x_set_modem_config_2(SX127x_SF_9, device));
  ESP_ERROR_CHECK(sx127x_set_syncword(18, device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, device));
  sx127x_set_tx_callback(tx_callback, device);

  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, device, 2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    sx127x_destroy(device);
    return;
  }

  gpio_set_direction((gpio_num_t)DIO0, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)DIO0);
  gpio_pullup_dis((gpio_num_t)DIO0);
  gpio_set_intr_type((gpio_num_t)DIO0, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)DIO0, handle_interrupt_fromisr, (void *)device);

  sx127x_tx_header_t header;
  header.crc = SX127x_RX_PAYLOAD_CRC_ON;
  header.coding_rate = SX127x_CR_4_5;
  ESP_ERROR_CHECK(sx127x_set_tx_explicit_header(&header, device));
  current_power = 2;
  tx_callback(device);
  while (1) {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
