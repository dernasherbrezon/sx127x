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
TaskHandle_t handle_interrupt;
int total_packets_received = 0;
static const char *TAG = "sx127x";

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xTaskResumeFromISR(handle_interrupt);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    vTaskSuspend(NULL);
    sx127x_handle_interrupt((sx127x *)arg);
  }
}

void rx_callback(sx127x *device) {
  uint8_t *data = NULL;
  uint8_t data_length = 0;
  esp_err_t code = sx127x_read_payload(device, &data, &data_length);
  if (code != ESP_OK) {
    ESP_LOGE(TAG, "can't read %d", code);
    return;
  }
  if (data_length == 0) {
    // no message received
    return;
  }
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int16_t rssi;
  ESP_ERROR_CHECK(sx127x_get_packet_rssi(device, &rssi));
  float snr;
  ESP_ERROR_CHECK(sx127x_get_packet_snr(device, &snr));
  int32_t frequency_error;
  ESP_ERROR_CHECK(sx127x_get_frequency_error(device, &frequency_error));

  ESP_LOGI(TAG, "received: %d %s rssi: %d snr: %f freq_error: %ld", data_length, payload, rssi, snr, frequency_error);

  total_packets_received++;
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
  ESP_ERROR_CHECK(sx127x_set_lna_boost_hf(SX127x_LNA_BOOST_HF_ON, device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device));
  ESP_ERROR_CHECK(sx127x_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  ESP_ERROR_CHECK(sx127x_set_bandwidth(SX127x_BW_125000, device));
  sx127x_implicit_header_t header = {
      .coding_rate = SX127x_CR_4_5,
      .crc = SX127x_RX_PAYLOAD_CRC_ON,
      .length = 2};
  ESP_ERROR_CHECK(sx127x_set_implicit_header(&header, device));
  ESP_ERROR_CHECK(sx127x_set_modem_config_2(SX127x_SF_9, device));
  ESP_ERROR_CHECK(sx127x_set_syncword(18, device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, device));
  sx127x_set_rx_callback(rx_callback, device);

  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, device, 2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    sx127x_destroy(device);
    return;
  }

  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)DIO0, GPIO_MODE_INPUT));
  ESP_ERROR_CHECK(gpio_pulldown_en((gpio_num_t)DIO0));
  ESP_ERROR_CHECK(gpio_pullup_dis((gpio_num_t)DIO0));
  ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)DIO0, GPIO_INTR_POSEDGE));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)DIO0, handle_interrupt_fromisr, (void *)device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  while (1) {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
