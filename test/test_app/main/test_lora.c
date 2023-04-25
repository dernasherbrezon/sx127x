#include <esp_log.h>
#include <unity.h>
#include <unity_test_runner.h>
#include <string.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include "sx127x_fixture.h"

sx127x_fixture_config_t lora_rx_fixture_config = {
        .sck = 5,
        .miso = 19,
        .mosi = 27,
        .ss = 18,
        .rst = 23,
        .dio0 = 26,
        .dio1 = 35, // Heltec lora32 v2
        .dio2 = 34,
        .modulation = SX127x_MODULATION_LORA};

sx127x_fixture_config_t lora_tx_fixture_config = {
        .sck = 5,
        .miso = 19,
        .mosi = 27,
        .ss = 18,
        .rst = 23,
        .dio0 = 26,
        .dio1 = 33, // TTGO lora32
        .dio2 = 32,
        .modulation = SX127x_MODULATION_LORA};

sx127x_fixture_t *fixture = NULL;
TickType_t TIMEOUT = pdMS_TO_TICKS(10000);

sx127x_implicit_header_t explicit_header = {
        .coding_rate = SX127x_CR_4_5,
        .enable_crc = true,
        .length = 2};

void tx_callback(sx127x *device) {
    xSemaphoreGive(fixture->tx_done);
}

void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
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

TEST_CASE("sx127x_test_lora_rx_deepsleep_verify", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create_base(&lora_rx_fixture_config, &fixture));
    esp_sleep_wakeup_cause_t cpu0WakeupReason = esp_sleep_get_wakeup_cause();
    TEST_ASSERT_EQUAL_INT(ESP_SLEEP_WAKEUP_EXT0, cpu0WakeupReason);
    sx127x_rx_set_callback(rx_callback, fixture->device);
    sx127x_handle_interrupt(fixture->device);
    uint8_t expected[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, sizeof(expected));
}

TEST_CASE("sx127x_test_lora_rx_deepsleep", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, rtc_gpio_set_direction((gpio_num_t) lora_rx_fixture_config.dio0, RTC_GPIO_MODE_INPUT_ONLY));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, rtc_gpio_pulldown_en((gpio_num_t) lora_rx_fixture_config.dio0));
    ESP_LOGI("sx127x_test", "entering deep sleep");
    TEST_ASSERT_EQUAL_INT(SX127X_OK, esp_sleep_enable_ext0_wakeup((gpio_num_t) lora_rx_fixture_config.dio0, 1));
    esp_deep_sleep_start();
}

TEST_CASE("sx127x_test_lora_rx_after_cad", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_rx_fixture_config, &fixture));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    sx127x_lora_cad_set_callback(cad_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    uint8_t expected[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, sizeof(expected));
}

TEST_CASE("sx127x_test_lora_rx_explicit_header", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_rx_fixture_config, &fixture));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    uint8_t expected[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, sizeof(expected));
}

TEST_CASE("sx127x_test_lora_tx_explicit_header", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_tx_fixture_config, &fixture));
    sx127x_tx_set_callback(tx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, fixture->device));
    sx127x_tx_header_t header = {
            .enable_crc = true,
            .coding_rate = SX127x_CR_4_5};
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_explicit_header(&header, fixture->device));
    uint8_t data[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(data, sizeof(data), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

TEST_CASE("sx127x_test_lora_rx_implicit_header", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_rx_fixture_config, &fixture));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(&explicit_header, fixture->device));
    sx127x_rx_set_callback(rx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, fixture->device));
    xSemaphoreTake(fixture->rx_done, TIMEOUT);
    uint8_t expected[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_UINT16(sizeof(expected), fixture->rx_data_length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, fixture->rx_data, sizeof(expected));
}

TEST_CASE("sx127x_test_lora_tx_implicit_header", "[lora]") {
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fixture_create(&lora_tx_fixture_config, &fixture));
    sx127x_tx_set_callback(tx_callback, fixture->device);
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(&explicit_header, fixture->device));
    uint8_t data[] = {0xCA, 0xFE};
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(data, sizeof(data), fixture->device));
    TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, fixture->device));
    xSemaphoreTake(fixture->tx_done, TIMEOUT);
}

void tearDown() {
    if (fixture != NULL) {
        sx127x_fixture_destroy(fixture);
        fixture = NULL;
    }
}