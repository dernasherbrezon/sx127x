#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <sx1278.h>

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

sx1278 *device = NULL;

void tx_callback(sx1278 *device) {
  Serial.printf("transmitted\n");
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting up");
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  esp_err_t code = spi_bus_initialize(HSPI_HOST, &config, 0);
  if (code != ESP_OK) {
    Serial.println("can't init bus");
    return;
  }
  code = sx1278_create(HSPI_HOST, SS, &device);
  if (code != ESP_OK) {
    Serial.println("can't create device");
    return;
  }

  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_SLEEP, device));
  ESP_ERROR_CHECK(sx1278_set_frequency(437200012, device));
  ESP_ERROR_CHECK(sx1278_reset_fifo(device));
  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_STANDBY, device));
  ESP_ERROR_CHECK(sx1278_set_bandwidth(SX1278_BW_125000, device));
  ESP_ERROR_CHECK(sx1278_set_implicit_header(NULL, device));
  ESP_ERROR_CHECK(sx1278_set_modem_config_2(SX1278_SF_9, device));
  ESP_ERROR_CHECK(sx1278_set_syncword(18, device));
  ESP_ERROR_CHECK(sx1278_set_preamble_length(8, device));
  sx1278_set_tx_callback(tx_callback, device);

  gpio_pad_select_gpio(DIO0);
  gpio_set_direction((gpio_num_t)DIO0, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)DIO0);
  gpio_pullup_dis((gpio_num_t)DIO0);
  gpio_set_intr_type((gpio_num_t)DIO0, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)DIO0, sx1278_handle_interrupt_fromisr, (void *)device);

  // 4 is OK
  ESP_ERROR_CHECK(sx1278_set_pa_config(SX1278_PA_PIN_BOOST, 4, device));
  ESP_ERROR_CHECK(sx1278_set_tx_crc(SX1278_RX_PAYLOAD_CRC_ON, device));

  uint8_t data[] = {0xCA, 0xFE};
  ESP_ERROR_CHECK(sx1278_set_for_transmission(data, sizeof(data), device));
  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_TX, device));
  Serial.printf("transmitting\n");
}

void loop() {
  delay(100000);
}
