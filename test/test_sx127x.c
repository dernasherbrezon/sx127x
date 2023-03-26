#include <check.h>
#include <stdlib.h>
#include <sx127x.h>

#include "sx127x_mock_spi.h"

sx127x *device = NULL;

START_TEST(test_lora) {
  uint8_t registers[255];
  memset(registers, 0, sizeof(registers));
  registers[0x42] = 0x12;
  spi_mock_registers(registers, SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_create(NULL, &device));
  spi_mock_write(SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  ck_assert_int_eq(SX127X_OK, sx127x_set_frequency(437200012, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_reset_fifo(device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_bandwidth(SX127x_BW_125000, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_implicit_header(NULL, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_modem_config_2(SX127x_SF_9, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_syncword(18, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_preamble_length(8, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_low_datarate_optimization(true, device));
  ck_assert_int_eq(SX127X_OK, sx127x_rx_set_lna_boost_hf(true, device));
  ck_assert_int_eq(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));

  ck_assert_int_eq(registers[0x01], 0b10000000);
  ck_assert_int_eq(registers[0x06], 0x6d);
  ck_assert_int_eq(registers[0x07], 0x4c);
  ck_assert_int_eq(registers[0x08], 0xcd);
  ck_assert_int_eq(registers[0x0e], 0x00);
  ck_assert_int_eq(registers[0x0f], 0x00);
  ck_assert_int_eq(registers[0x1d], 0b01110000);
  ck_assert_int_eq(registers[0x31], 0xc3);
  ck_assert_int_eq(registers[0x37], 0x0a);
  ck_assert_int_eq(registers[0x1e], 0b10010000);
  ck_assert_int_eq(registers[0x39], 18);
  ck_assert_int_eq(registers[0x20], 0);
  ck_assert_int_eq(registers[0x21], 8);
  ck_assert_int_eq(registers[0x26], 0b00001000);
  ck_assert_int_eq(registers[0x0c], 0b10000011);

  uint32_t bandwidth;
  ck_assert_int_eq(SX127X_OK, sx127x_lora_get_bandwidth(device, &bandwidth));
  ck_assert_int_eq(125000, bandwidth);

  registers[0x19] = (uint8_t)(-21);
  float snr;
  ck_assert_int_eq(SX127X_OK, sx127x_lora_rx_get_packet_snr(device, &snr));
  ck_assert_float_eq(-5.25, snr);

  int16_t rssi;
  ck_assert_int_eq(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  // FIXME
}
END_TEST

START_TEST(test_init_failure) {
  uint8_t registers[255];
  memset(registers, 0, sizeof(registers));
  spi_mock_registers(registers, SX127X_ERR_INVALID_ARG);
  ck_assert_int_eq(SX127X_ERR_INVALID_ARG, sx127x_create(NULL, &device));
  registers[0x42] = 0x13;
  spi_mock_registers(registers, SX127X_OK);
  ck_assert_int_eq(SX127X_ERR_INVALID_VERSION, sx127x_create(NULL, &device));
}
END_TEST

void teardown() {
  if (device != NULL) {
    sx127x_destroy(device);
  }
}

void setup() {
  // do nothing
}

Suite *common_suite(void) {
  Suite *s;
  TCase *tc_core;

  s = suite_create("sx127x");

  /* Core test case */
  tc_core = tcase_create("Core");

  tcase_add_test(tc_core, test_init_failure);
  tcase_add_test(tc_core, test_lora);
  // tcase_add_test(tc_core, test_small_buffer);
  // tcase_add_test(tc_core, test_big_buffer);

  tcase_add_checked_fixture(tc_core, setup, teardown);
  suite_add_tcase(s, tc_core);

  return s;
}

int main(void) {
  int number_failed;
  Suite *s;
  SRunner *sr;

  s = common_suite();
  sr = srunner_create(s);

  srunner_set_fork_status(sr, CK_NOFORK);
  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
