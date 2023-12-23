#include <unity_test_runner.h>
#include <esp_log.h>
#include <string.h>
#include <sx127x.h>
#include <unity.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <stdio.h>

#include "sx127x_fixture.h"

#define MAX_BEACONS_EXPECTED 5
#define BEACON_INTERVAL 200
#define FIXED_MAX_LENGTH 2047

sx127x_fixture_config_t rx_fixture_config = {
    .sck = 5,
    .miso = 19,
    .mosi = 27,
    .ss = 18,
    .rst = 23,
    .dio0 = 26,
    .dio1 = 33, // TTGO lora32
    .dio2 = 32};

sx127x_fixture_config_t tx_fixture_config = {
    .sck = 5,
    .miso = 19,
    .mosi = 27,
    .ss = 18,
    .rst = 23,
    .dio0 = 26,
    .dio1 = 33, // TTGO lora32
    .dio2 = 32};

sx127x_fixture_t *fixture = NULL;
TickType_t TIMEOUT = pdMS_TO_TICKS(10000);
uint8_t lora_small_message[] = {0xCA, 0xFE};
uint8_t fsk_small_message[] = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00};
// 63 bytes max for variable to fit into FIFO
uint8_t fsk_max_single_batch[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,
                                  0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e};
uint8_t fsk_max_single_batch_fixed[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a,
                                        0x2b,
                                        0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};
uint8_t fsk_max_variable[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c,
                              0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                              0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86,
                              0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
                              0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0,
                              0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe};


int message_counter = 0;

sx127x_implicit_header_t explicit_header = {
    .coding_rate = SX127x_CR_4_5,
    .enable_crc = true,
    .length = 2};

void tx_callback(sx127x *device) {
  xSemaphoreGive(fixture->tx_done);
  ESP_LOGI("sx127x_test", "sent");
}

void rx_log_message(sx127x *device, uint8_t *data, uint16_t data_length) {
  int32_t frequency_error = 0;
  ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  int16_t rssi = 0;
  sx127x_rx_get_packet_rssi(device, &rssi);
  ESP_LOGI("sx127x_test", "received: %d rssi: %d freq error: %ld", data_length, rssi, frequency_error);
}

void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  rx_log_message(device, data, data_length);
  memcpy(fixture->rx_data, data, data_length);
  fixture->rx_data_length = data_length;
  xSemaphoreGive(fixture->rx_done);
}

void cad_callback(sx127x *device, int cad_detected) {
  if (cad_detected == 0) {
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
    return;
  }
  // put into RX mode first to handle interrupt as soon as possible
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
}

void rx_callback_with_counter(sx127x *device, uint8_t *data, uint16_t data_length) {
  rx_log_message(device, data, data_length);
  message_counter++;
  if (message_counter >= MAX_BEACONS_EXPECTED) {
    xSemaphoreGive(fixture->rx_done);
  }
}

void wait_for_rx_done() {
  // delay depends on bit rate, modulation and current state
  vTaskDelay(pdMS_TO_TICKS(5));
  // tests will wait for this message before sending data
  ESP_LOGI("sx127x_test", "RX started");
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
}

void print_registers() {
  uint8_t registers[0x80];
  sx127x_dump_registers(registers, fixture->device);
  for (int i = 0; i < sizeof(registers); i++) {
    if (i != 0) {
      printf(",");
    }
    printf("%d", registers[i]);
  }
  printf("\n");
}
TEST_CASE("sx127x_test_fsk_rx_print_registers", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create_base(&rx_fixture_config, &fixture));
  uint8_t registers[0x80];
  sx127x_dump_registers(registers, fixture->device);
  for (int i = 0; i < sizeof(registers); i++) {
    if (i != 0) {
      printf(",");
    }
    printf("%d", registers[i]);
  }
  printf("\n");
}

TEST_CASE("sx127x_test_fsk_rx_variable_length", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_single_batch), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_single_batch, fixture->rx_data, sizeof(fsk_max_single_batch));
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_variable), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_variable, fixture->rx_data, sizeof(fsk_max_variable));
}

TEST_CASE("sx127x_test_fsk_tx_variable_length", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
  vTaskDelay(pdMS_TO_TICKS(50));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_single_batch, sizeof(fsk_max_single_batch), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
  vTaskDelay(pdMS_TO_TICKS(50));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_variable, sizeof(fsk_max_variable), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_fsk_rx_beacons", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
  sx127x_rx_set_callback(rx_callback_with_counter, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_INT(MAX_BEACONS_EXPECTED, message_counter);
}

TEST_CASE("sx127x_test_fsk_tx_beacons", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(fsk_small_message, sizeof(fsk_small_message), BEACON_INTERVAL, fixture->device));
  vTaskDelay(pdMS_TO_TICKS((MAX_BEACONS_EXPECTED + 0.5f) * BEACON_INTERVAL));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_stop_beacon(fixture->device));
}

TEST_CASE("sx127x_test_fsk_rx_fixed_small", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
}

TEST_CASE("sx127x_test_fsk_tx_fixed_small", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_fsk_rx_fixed_batch", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_max_single_batch_fixed), fixture->device));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_single_batch_fixed), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_single_batch_fixed, fixture->rx_data, sizeof(fsk_max_single_batch_fixed));
}

TEST_CASE("sx127x_test_fsk_tx_fixed_batch", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_max_single_batch_fixed), fixture->device));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_single_batch_fixed, sizeof(fsk_max_single_batch_fixed), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_fsk_rx_fixed_max", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, FIXED_MAX_LENGTH, fixture->device));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(FIXED_MAX_LENGTH, fixture->rx_data_length);
  uint8_t *expected = malloc(FIXED_MAX_LENGTH);
  for (int i = 0; i < FIXED_MAX_LENGTH; i++) {
    expected[i] = (i % 10);
  }
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, FIXED_MAX_LENGTH);
  free(expected);
}

TEST_CASE("sx127x_test_fsk_tx_fixed_max", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, FIXED_MAX_LENGTH, fixture->device));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  uint8_t *expected = malloc(FIXED_MAX_LENGTH);
  for (int i = 0; i < FIXED_MAX_LENGTH; i++) {
    expected[i] = (i % 10);
  }
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(expected, FIXED_MAX_LENGTH, fixture->device));
  free(expected);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_fsk_rx_max_baud", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_bitrate(300000.0, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_set_fdev(100000.0, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_afc_auto(false, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_afc_bandwidth(170000.0, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_bandwidth(170000.0, fixture->device));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
}

TEST_CASE("sx127x_test_fsk_tx_max_baud", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_bitrate(300000.0, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_set_fdev(100000.0, fixture->device));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_fsk_rx_filtered", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0xca, 0xfe, fixture->device));
  sx127x_rx_set_callback(rx_callback_with_counter, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_INT(MAX_BEACONS_EXPECTED, message_counter);
}

TEST_CASE("sx127x_test_fsk_tx_filtered", "[fsk]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_FSK, &fixture));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  uint8_t addresses[] = {0xca, 0x12, 0xfe, 0xca, 0xca, 0xfe};
  for (int i = 0; i < sizeof(addresses); i++) {
    if (i != 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission_with_address(fsk_small_message, sizeof(fsk_small_message), addresses[i], fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
  }
}

TEST_CASE("sx127x_test_lora_rx_deepsleep_verify", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create_base(&rx_fixture_config, &fixture));
  esp_sleep_wakeup_cause_t cpu0WakeupReason = esp_sleep_get_wakeup_cause();
  TEST_ASSERT_EQUAL_INT(ESP_SLEEP_WAKEUP_EXT0, cpu0WakeupReason);
  sx127x_rx_set_callback(rx_callback, fixture->device);
  sx127x_handle_interrupt(fixture->device);
  TEST_ASSERT_EQUAL_UINT16(sizeof(lora_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(lora_small_message, fixture->rx_data, sizeof(lora_small_message));
}

TEST_CASE("sx127x_test_lora_rx_deepsleep", "[lora]") {

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, rtc_gpio_set_direction((gpio_num_t) rx_fixture_config.dio0, RTC_GPIO_MODE_INPUT_ONLY));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, rtc_gpio_pulldown_en((gpio_num_t) rx_fixture_config.dio0));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device));
  ESP_LOGI("sx127x_test", "entering deep sleep");
  TEST_ASSERT_EQUAL_INT(SX127X_OK, esp_sleep_enable_ext0_wakeup((gpio_num_t) rx_fixture_config.dio0, 1));
  esp_deep_sleep_start();
}

TEST_CASE("sx127x_test_lora_rx_after_cad", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  sx127x_lora_cad_set_callback(cad_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, fixture->device));
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT16(sizeof(lora_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(lora_small_message, fixture->rx_data, sizeof(lora_small_message));
}

TEST_CASE("sx127x_test_lora_rx_explicit_header", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(lora_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(lora_small_message, fixture->rx_data, sizeof(lora_small_message));
}

TEST_CASE("sx127x_test_lora_tx_explicit_header", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  sx127x_tx_set_callback(tx_callback, fixture->device);
  sx127x_tx_header_t header = {
      .enable_crc = true,
      .coding_rate = SX127x_CR_4_5};
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_explicit_header(&header, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(lora_small_message, sizeof(lora_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_lora_rx_implicit_header", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(&explicit_header, fixture->device));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(lora_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(lora_small_message, fixture->rx_data, sizeof(lora_small_message));
}

TEST_CASE("sx127x_test_lora_tx_implicit_header", "[lora]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_LORA, &fixture));
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(&explicit_header, fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(lora_small_message, sizeof(lora_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_ook_rx_variable_length", "[ook]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&rx_fixture_config, SX127x_MODULATION_OOK, &fixture));
  sx127x_rx_set_callback(rx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_OOK, fixture->device));
  wait_for_rx_done();
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_single_batch), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_single_batch, fixture->rx_data, sizeof(fsk_max_single_batch));
  xSemaphoreTake(fixture->rx_done, TIMEOUT);
  TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_variable), fixture->rx_data_length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_variable, fixture->rx_data, sizeof(fsk_max_variable));
}

TEST_CASE("sx127x_test_ook_tx_variable_length", "[ook]") {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&tx_fixture_config, SX127x_MODULATION_OOK, &fixture));
  setup_gpio_interrupts((gpio_num_t) tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
  sx127x_tx_set_callback(tx_callback, fixture->device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_OOK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
  vTaskDelay(pdMS_TO_TICKS(50));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_single_batch, sizeof(fsk_max_single_batch), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_OOK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
  vTaskDelay(pdMS_TO_TICKS(50));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_variable, sizeof(fsk_max_variable), fixture->device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_OOK, fixture->device));
  xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

void tearDown() {
  if (fixture != NULL) {
    sx127x_fixture_destroy(fixture);
    fixture = NULL;
  }
  message_counter = 0;
}

void app_main(void) {
  unity_run_menu();
}
