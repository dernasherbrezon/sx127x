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
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &config, 0));
  ESP_ERROR_CHECK(sx1278_create(HSPI_HOST, SS, &device));
  // TODO check if sleep is ok
  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_STANDBY, device));
  ESP_ERROR_CHECK(sx1278_set_lna_gain(SX1278_LNA_GAIN_G1, device));
  ESP_ERROR_CHECK(sx1278_set_lna_boost_hf(SX1278_LNA_BOOST_HF_ON, device));
  ESP_ERROR_CHECK(sx1278_set_bandwidth(SX1278_BW_125000, device));
  ESP_ERROR_CHECK(sx1278_set_implicit_header(NULL, device));
  ESP_ERROR_CHECK(sx1278_set_modem_config_2(SX1278_SF_9, device));
  ESP_ERROR_CHECK(sx1278_set_syncword(18, device));
  ESP_ERROR_CHECK(sx1278_set_preamble_length(8, device));
  ESP_ERROR_CHECK(sx1278_set_frequency(437200012, device));

  gpio_pad_select_gpio(DIO0);
  gpio_set_direction((gpio_num_t)DIO0, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)DIO0);
  gpio_pullup_dis((gpio_num_t)DIO0);
  gpio_set_intr_type((gpio_num_t)DIO0, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)DIO0, sx1278_handle_interrupt, (void *)device);

  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_RX_CONT, device));
}

void loop() {
  if (device == NULL) {
    return;
  }
  uint8_t *data = NULL;
  uint8_t data_length = 0;
  esp_err_t code = sx1278_receive(device, &data, &data_length);
  if (code != ESP_OK) {
    Serial.printf("can't read %d", code);
    return;
  }
  if (data_length == 0) {
    // no message received
    return;
  }
  printf("received: %d\n", data_length);
  for (int i = 0; i < data_length; i++) {
    printf("%x", data[i]);
  }
  printf("\n");
  
  int16_t rssi;
  ESP_ERROR_CHECK(sx1278_get_rssi(device, &rssi));
  printf("rssi: %d\n", rssi);

  ESP_ERROR_CHECK(sx1278_set_opmod(SX1278_MODE_SLEEP, device));
  sx1278_destroy(device);
  device = NULL;
}
