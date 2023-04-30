#include <esp_log.h>
#include <unity.h>
#include <unity_test_runner.h>

#include "sx127x_fixture.h"

#define MAX_BEACONS_EXPECTED 5
#define BEACON_INTERVAL 200
#define FIXED_MAX_LENGTH 2047

sx127x_fixture_config_t fsk_rx_fixture_config = {
        .sck = 5,
        .miso = 19,
        .mosi = 27,
        .ss = 18,
        .rst = 23,
        .dio0 = 26,
        .dio1 = 35, // Heltec lora32 v2
        .dio2 = 34,
        .modulation = SX127x_MODULATION_FSK};

sx127x_fixture_config_t fsk_tx_fixture_config = {
        .sck = 5,
        .miso = 19,
        .mosi = 27,
        .ss = 18,
        .rst = 23,
        .dio0 = 26,
        .dio1 = 33, // TTGO lora32
        .dio2 = 32,
        .modulation = SX127x_MODULATION_FSK};

int rx_counter = 0;


void rx_callback_with_counter(sx127x *device, uint8_t *data, uint16_t data_length) {
    rx_counter++;
    if (rx_counter >= MAX_BEACONS_EXPECTED) {
        xSemaphoreGive(fixture->rx_done);
    }
}

uint8_t fsk_small_message[] = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00};
// 63 bytes max for variable to fit into FIFO
uint8_t fsk_max_single_batch[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,
                                  0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e};
uint8_t fsk_max_single_batch_fixed[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,
                                  0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};
uint8_t fsk_max_variable[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c,
                              0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
                              0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86,
                              0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3,
                              0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0,
                              0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe};

TEST_CASE("sx127x_test_fsk_rx_print_registers", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create_base(&fsk_rx_fixture_config, &fixture));
    sx127x_dump_registers(fixture->device);
}

TEST_CASE("sx127x_test_fsk_rx_variable_length", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "received: %d", fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "received: %d", fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_single_batch), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_single_batch, fixture->rx_data, sizeof(fsk_max_single_batch));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "received: %d", fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_variable), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_variable, fixture->rx_data, sizeof(fsk_max_variable));
}

TEST_CASE("sx127x_test_fsk_tx_variable_length", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_tx_fixture_config, &fixture));
    setup_gpio_interrupts((gpio_num_t) fsk_tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
    sx127x_tx_set_callback(tx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "sent: %zu", sizeof(fsk_small_message));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_single_batch, sizeof(fsk_max_single_batch), fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "sent: %zu", sizeof(fsk_max_single_batch));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_variable, sizeof(fsk_max_variable), fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    ESP_LOGI("sx127x_test", "sent: %zu", sizeof(fsk_max_variable));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, fixture->device));
}

TEST_CASE("sx127x_test_fsk_rx_beacons", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
    sx127x_rx_set_callback(rx_callback_with_counter, fixture->device);
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_INT(MAX_BEACONS_EXPECTED, rx_counter);
}

TEST_CASE("sx127x_test_fsk_tx_beacons", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
    setup_gpio_interrupts((gpio_num_t) fsk_tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(fsk_small_message, sizeof(fsk_small_message), BEACON_INTERVAL, fixture->device));
    vTaskDelay(pdMS_TO_TICKS((MAX_BEACONS_EXPECTED + 1) * BEACON_INTERVAL));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_tx_stop_beacon(fixture->device));
}

TEST_CASE("sx127x_test_fsk_rx_fixed_small", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_small_message), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_small_message, fixture->rx_data, sizeof(fsk_small_message));
}
TEST_CASE("sx127x_test_fsk_tx_fixed_small", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_small_message), fixture->device));
    setup_gpio_interrupts((gpio_num_t) fsk_tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
    sx127x_tx_set_callback(tx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_small_message, sizeof(fsk_small_message), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, fixture->device));
}
TEST_CASE("sx127x_test_fsk_rx_fixed_batch", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_max_single_batch_fixed), fixture->device));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(sizeof(fsk_max_single_batch_fixed), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fsk_max_single_batch_fixed, fixture->rx_data, sizeof(fsk_max_single_batch_fixed));
}
TEST_CASE("sx127x_test_fsk_tx_fixed_batch", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(fsk_max_single_batch_fixed), fixture->device));
    setup_gpio_interrupts((gpio_num_t) fsk_tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
    sx127x_tx_set_callback(tx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(fsk_max_single_batch_fixed, sizeof(fsk_max_single_batch_fixed), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, fixture->device));
}
TEST_CASE("sx127x_test_fsk_rx_fixed_max", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK,sx127x_fsk_ook_set_packet_format(SX127X_FIXED, FIXED_MAX_LENGTH, fixture->device));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_UINT16(FIXED_MAX_LENGTH, fixture->rx_data_length);
    uint8_t expected[FIXED_MAX_LENGTH];
    for (int i = 0; i < FIXED_MAX_LENGTH; i++) {
        expected[i] = (i % 10);
    }
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, FIXED_MAX_LENGTH);
}
TEST_CASE("sx127x_test_fsk_tx_fixed_max", "[fsk]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&fsk_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, FIXED_MAX_LENGTH, fixture->device));
    setup_gpio_interrupts((gpio_num_t) fsk_tx_fixture_config.dio1, fixture, GPIO_INTR_NEGEDGE);
    sx127x_tx_set_callback(tx_callback, fixture->device);
    uint8_t expected[FIXED_MAX_LENGTH];
    for (int i = 0; i < FIXED_MAX_LENGTH; i++) {
        expected[i] = (i % 10);
    }
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(expected, sizeof(expected), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, fixture->device));
}
