#include <check.h>
#include <stdlib.h>
#include <sx127x.h>

#include "sx127x_mock_spi.h"

sx127x *device = NULL;

START_TEST(test_fsk_ook_rssi) {
  uint8_t registers[255];
  memset(registers, 0, sizeof(registers));
  registers[0x42] = 0x12;
  spi_mock_registers(registers, SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_create(NULL, &device));
  spi_mock_write(SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));

  int16_t rssi;
  ck_assert_int_eq(SX127X_ERR_NOT_FOUND, sx127x_rx_get_packet_rssi(device, &rssi));

  ck_assert_int_eq(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, device));
  ck_assert_int_eq(registers[0x41], 0b11000000);

  // simulate interrupt
  registers[0x3e] = 0b00000010;
  registers[0x11] = 30;
  sx127x_handle_interrupt(device);

  ck_assert_int_eq(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  ck_assert_int_eq(-15, rssi);
}
END_TEST

START_TEST(test_fsk_ook) {
  uint8_t registers[255];
  memset(registers, 0, sizeof(registers));
  registers[0x42] = 0x12;
  spi_mock_registers(registers, SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_create(NULL, &device));
  spi_mock_write(SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));
  ck_assert_int_eq(registers[0x01], 0b00000000);
  ck_assert_int_eq(SX127X_OK, sx127x_set_frequency(437200012, device));
  ck_assert_int_eq(registers[0x06], 0x6d);
  ck_assert_int_eq(registers[0x07], 0x4c);
  ck_assert_int_eq(registers[0x08], 0xcd);
  ck_assert_int_eq(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_bitrate(4800.0, device));
  ck_assert_int_eq(registers[0x02], 0x1A);
  ck_assert_int_eq(registers[0x03], 0x0A);
  ck_assert_int_eq(registers[0x5d], 0x0A);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_set_fdev(5000.0, device));
  ck_assert_int_eq(registers[0x04], 0x00);
  ck_assert_int_eq(registers[0x05], 0x51);
  uint8_t syncWord[] = {0x12, 0xAD};
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_syncword(syncWord, 2, device));
  ck_assert_int_eq(registers[0x28], 0x12);
  ck_assert_int_eq(registers[0x29], 0xAD);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_packet_encoding(SX127X_SCRAMBLED, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  ck_assert_int_eq(registers[0x33], 0x11);
  ck_assert_int_eq(registers[0x34], 0x12);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  ck_assert_int_eq(registers[0x31], 0b00000000);
  ck_assert_int_eq(registers[0x32], 0xFF);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_set_preamble_type(SX127X_PREAMBLE_55, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_afc_auto(true, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, device));
  ck_assert_int_eq(registers[0x13], 0x14);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_bandwidth(5000.0, device));
  ck_assert_int_eq(registers[0x12], 0x16);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_collision_restart(true, 10, device));
  ck_assert_int_eq(registers[0x0f], 10);
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, device));
  ck_assert_int_eq(SX127X_OK, sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, device));
  ck_assert_int_eq(SX127X_OK, sx127x_ook_rx_set_peak_mode(SX127X_0_5_DB, 0x0C, SX127X_1_1_CHIP, device));

  ck_assert_int_eq(registers[0x0d], 0b10010111);
  ck_assert_int_eq(registers[0x0c], 0b10000000);
  ck_assert_int_eq(registers[0x30], 0b11010100);
  ck_assert_int_eq(registers[0x27], 0b00110001);
  ck_assert_int_eq(registers[0x0a], 0b01001001);
  ck_assert_int_eq(registers[0x0e], 0b00000010);
  ck_assert_int_eq(registers[0x1f], 0b10101010);

  ck_assert_int_eq(SX127X_OK, sx127x_ook_set_data_shaping(SX127X_1_BIT_RATE, SX127X_PA_RAMP_10, device));
  ck_assert_int_eq(registers[0x0a], 0b00101001);

  ck_assert_int_eq(SX127X_OK, sx127x_ook_rx_set_fixed_mode(0x11, device));
  ck_assert_int_eq(registers[0x14], 0b00000000);
  ck_assert_int_eq(registers[0x15], 0x11);

  ck_assert_int_eq(SX127X_OK, sx127x_ook_rx_set_avg_mode(SX127X_2_DB, SX127X_4_PI, device));
  ck_assert_int_eq(registers[0x14], 0b00010000);
  ck_assert_int_eq(registers[0x16], 0b00000110);

  registers[0x1d] = 0xFF;
  registers[0x1e] = 0xF0;
  int32_t frequency_error;
  ck_assert_int_eq(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  ck_assert_int_eq(-976, frequency_error);
}
END_TEST

START_TEST(test_lora) {
  uint8_t registers[255];
  memset(registers, 0, sizeof(registers));
  registers[0x42] = 0x12;
  spi_mock_registers(registers, SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_create(NULL, &device));
  spi_mock_write(SX127X_OK);
  ck_assert_int_eq(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  ck_assert_int_eq(registers[0x01], 0b10000000);
  ck_assert_int_eq(SX127X_OK, sx127x_set_frequency(437200012, device));
  ck_assert_int_eq(registers[0x06], 0x6d);
  ck_assert_int_eq(registers[0x07], 0x4c);
  ck_assert_int_eq(registers[0x08], 0xcd);
  ck_assert_int_eq(SX127X_OK, sx127x_lora_reset_fifo(device));
  ck_assert_int_eq(registers[0x0e], 0x00);
  ck_assert_int_eq(registers[0x0f], 0x00);
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_bandwidth(SX127x_BW_125000, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_implicit_header(NULL, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_modem_config_2(SX127x_SF_9, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_syncword(18, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_preamble_length(8, device));
  ck_assert_int_eq(SX127X_OK, sx127x_lora_set_low_datarate_optimization(true, device));
  ck_assert_int_eq(SX127X_OK, sx127x_rx_set_lna_boost_hf(true, device));
  ck_assert_int_eq(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  ck_assert_int_eq(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, device));

  ck_assert_int_eq(registers[0x1d], 0b01110000);
  ck_assert_int_eq(registers[0x31], 0xc3);
  ck_assert_int_eq(registers[0x37], 0x0a);
  ck_assert_int_eq(registers[0x1e], 0b10010000);
  ck_assert_int_eq(registers[0x39], 18);
  ck_assert_int_eq(registers[0x20], 0);
  ck_assert_int_eq(registers[0x21], 8);
  ck_assert_int_eq(registers[0x26], 0b00001000);
  ck_assert_int_eq(registers[0x0c], 0b10000011);
  ck_assert_int_eq(registers[0x4d], 0b10000100);
  ck_assert_int_eq(registers[0x09], 0b10000010);
  ck_assert_int_eq(registers[0x0b], 0x28);

  uint32_t bandwidth;
  ck_assert_int_eq(SX127X_OK, sx127x_lora_get_bandwidth(device, &bandwidth));
  ck_assert_int_eq(125000, bandwidth);

  registers[0x19] = (uint8_t)(-21);
  float snr;
  ck_assert_int_eq(SX127X_OK, sx127x_lora_rx_get_packet_snr(device, &snr));
  ck_assert_float_eq(-5.25, snr);

  registers[0x1a] = 134;
  int16_t rssi;
  ck_assert_int_eq(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  ck_assert_int_eq(-35, rssi);

  registers[0x28] = 0x0F;
  registers[0x29] = 0xFF;
  registers[0x2a] = 0xF0;
  int32_t frequency_error;
  ck_assert_int_eq(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  ck_assert_int_eq(-2, frequency_error);

  sx127x_tx_header_t header = {
      .enable_crc = true,
      .coding_rate = SX127x_CR_4_5};
  ck_assert_int_eq(SX127X_OK, sx127x_lora_tx_set_explicit_header(&header, device));
  ck_assert_int_eq(registers[0x1d], 0b01110010);
  ck_assert_int_eq(registers[0x1e], 0b10010100);
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
    device = NULL;
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
  tcase_add_test(tc_core, test_fsk_ook);
  tcase_add_test(tc_core, test_fsk_ook_rssi);

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
