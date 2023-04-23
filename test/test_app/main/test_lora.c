#include <esp_log.h>
#include <unity.h>
#include <unity_test_runner.h>

#include "sx127x_fixture.h"

sx127x_fixture_config_t fixture_config = {
    .sck = 5,
    .miso = 19,
    .mosi = 27,
    .ss = 18,
    .rst = 23,
    .dio0 = 26};

sx127x_fixture_t *fixture = NULL;

void tx_callback(sx127x *device) {
  xSemaphoreGive(fixture->tx_done);
}

void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  sx127x_fixture_rx_callback(device, data, data_length);
  xSemaphoreGive(fixture->rx_done);
}

void sx127x_test_lora_rx_explicit_header() {
  sx127x_rx_set_callback(rx_callback, fixture->device);
  sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device);
  xSemaphoreTake(fixture->rx_done, portMAX_DELAY);
}

void sx127x_test_lora_tx_explicit_header() {
  sx127x_tx_set_callback(tx_callback, fixture->device);
  sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, fixture->device);
  sx127x_tx_header_t header = {
      .enable_crc = true,
      .coding_rate = SX127x_CR_4_5};
  sx127x_lora_tx_set_explicit_header(&header, fixture->device);
  uint8_t data[] = {0xCA, 0xFE};
  sx127x_lora_tx_set_for_transmission(data, sizeof(data), fixture->device);
  sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, fixture->device);
  ESP_LOGI("sx127x_test", "done. waiting for message");
  xSemaphoreTake(fixture->tx_done, portMAX_DELAY);
}

TEST_CASE_MULTIPLE_DEVICES("lora_rx", "[lora]", sx127x_test_lora_rx_explicit_header, sx127x_test_lora_tx_explicit_header);

void setUp() {
  // FIXME verify return code
  sx127x_fixture_create(&fixture_config, &fixture);
}

void tearDown() {
  if (fixture != NULL) {
    sx127x_fixture_destroy(fixture);
    fixture = NULL;
  }
}