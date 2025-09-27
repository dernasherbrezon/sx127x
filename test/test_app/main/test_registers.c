#include "test_registers.h"
#include <unity.h>

void test_registers_fsk_ook(sx127x *device) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device), "unable to sx127x_set_opmod");
  sx127x_mode_t mode;
  sx127x_modulation_t modulation;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_opmod(device, &mode, &modulation), "unable to sx127x_get_opmod");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_MODE_SLEEP, mode, "invalid mode");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_MODULATION_FSK, modulation, "invalid modulation");
  // LnaGain is only readable in the standby mode
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device), "unable to sx127x_set_opmod");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_frequency(868200012, device), "unable to sx127x_set_frequency");
  uint64_t frequency;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_frequency(device, &frequency), "unable to sx127x_get_frequency");
  TEST_ASSERT_EQUAL_MESSAGE(868200000, frequency, "invalid frequency");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device), "unable to sx127x_rx_set_lna_gain");
  sx127x_gain_t lna_gain;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_get_lna_gain(device, &lna_gain), "unable to sx127x_rx_get_lna_gain");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_LNA_GAIN_G4, lna_gain, "invalid lna_gain");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_bitrate(4800.0, device), "unable to sx127x_fsk_ook_set_bitrate");
  float bitrate;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_bitrate(device, &bitrate), "unable to sx127x_fsk_ook_get_bitrate");
  TEST_ASSERT_EQUAL_MESSAGE(4800.0, bitrate, "invalid bitrate");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_set_fdev(5000.0, device), "unable to sx127x_fsk_set_fdev");
  float frequency_deviation;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_get_fdev(device, &frequency_deviation), "unable to sx127x_fsk_get_fdev");
  TEST_ASSERT_EQUAL_MESSAGE(4943.0, frequency_deviation, "invalid frequency_deviation");

  uint8_t syncWord[] = {0x12, 0xAD};
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_syncword(syncWord, 2, device), "unable to sx127x_fsk_ook_set_syncword");
  uint8_t syncword[8];
  uint8_t syncword_length;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_syncword(device, syncword, &syncword_length), "unable to sx127x_fsk_ook_get_syncword");
  TEST_ASSERT_EQUAL_INT_MESSAGE(2, syncword_length, "unexpected syncword length");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0x12, syncword[0], "unexpected syncword");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0xAD, syncword[1], "unexpected syncword");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_packet_encoding(SX127X_SCRAMBLED, device), "unable to sx127x_fsk_ook_set_packet_encoding");
  sx127x_packet_encoding_t encoding;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_packet_encoding(device, &encoding), "unable to sx127x_fsk_ook_get_packet_encoding");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_SCRAMBLED, encoding, "invalid encoding");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device), "unable to sx127x_fsk_ook_set_crc");
  sx127x_crc_type_t crc_type;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_crc(device, &crc_type), "unable to sx127x_fsk_ook_get_crc");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_CRC_CCITT, crc_type, "invalid crc_type");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device), "unable to sx127x_fsk_ook_set_address_filtering");
  sx127x_address_filtering_t filter_type;
  uint8_t node_address;
  uint8_t broadcast_address;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_address_filtering(device, &filter_type, &node_address, &broadcast_address), "unable to sx127x_fsk_ook_get_address_filtering");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_FILTER_NODE_AND_BROADCAST, filter_type, "invalid filter_type");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0x11, node_address, "invalid node_address");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0x12, broadcast_address, "invalid broadcast_address");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device), "unable to sx127x_fsk_ook_set_packet_format");
  sx127x_packet_format_t format;
  uint16_t max_payload_length;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_packet_format(device, &format, &max_payload_length), "unable to sx127x_fsk_ook_get_packet_format");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_VARIABLE, format, "invalid format");
  TEST_ASSERT_EQUAL_INT_MESSAGE(255, max_payload_length, "invalid max_payload_length");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, device), "unable to sx127x_fsk_set_data_shaping");
  sx127x_fsk_data_shaping_t data_shaping;
  sx127x_pa_ramp_t pa_ramp;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_get_data_shaping(device, &data_shaping, &pa_ramp), "unable to sx127x_fsk_get_data_shaping");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_BT_0_5, data_shaping, "invalid data_shaping");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_PA_RAMP_10, pa_ramp, "invalid pa_ramp");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_preamble_type(SX127X_PREAMBLE_55, device), "unable to sx127x_fsk_ook_set_preamble_type");
  sx127x_preamble_type_t preamble_type;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_preamble_type(device, &preamble_type), "unable to sx127x_fsk_ook_get_preamble_type");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_PREAMBLE_55, preamble_type, "invalid preamble_type");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_afc_auto(true, device), "unable to sx127x_fsk_ook_rx_set_afc_auto");
  bool afc_auto;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_afc_auto(device, &afc_auto), "unable to sx127x_fsk_ook_rx_get_afc_auto");
  TEST_ASSERT_EQUAL_MESSAGE(true, afc_auto, "invalid afc_auto");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, device), "unable to sx127x_fsk_ook_rx_set_afc_bandwidth");
  float afc_bandwidth;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_afc_bandwidth(device, &afc_bandwidth), "unable to sx127x_fsk_ook_rx_get_afc_bandwidth");
  TEST_ASSERT_EQUAL_MESSAGE(20833.0, afc_bandwidth, "invalid afc_bandwidth");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_bandwidth(5000.0, device), "unable to sx127x_fsk_ook_rx_set_bandwidth");
  float bandwidth;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_bandwidth(device, &bandwidth), "unable to sx127x_fsk_ook_rx_get_bandwidth");
  TEST_ASSERT_EQUAL_MESSAGE(5208.0, bandwidth, "invalid bandwidth");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, device), "unable to sx127x_fsk_ook_rx_set_rssi_config");
  sx127x_rssi_smoothing_t smoothing;
  int8_t offset;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_rssi_config(device, &smoothing, &offset), "unable to sx127x_fsk_ook_rx_get_rssi_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_8, smoothing, "invalid smoothing");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, offset, "invalid offset");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_collision_restart(true, 10, device), "unable to sx127x_fsk_ook_rx_set_collision_restart");
  bool enable;
  uint8_t threshold;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_collision_restart(device, &enable, &threshold), "unable to sx127x_fsk_ook_rx_get_collision_restart");
  TEST_ASSERT_EQUAL_MESSAGE(true, enable, "invalid enable");
  TEST_ASSERT_EQUAL_MESSAGE(10, threshold, "invalid threshold");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, device), "unable to sx127x_fsk_ook_rx_set_trigger");
  sx127x_rx_trigger_t trigger;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_trigger(device, &trigger), "unable to sx127x_fsk_ook_rx_get_trigger");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_RX_TRIGGER_RSSI_PREAMBLE, trigger, "invalid trigger");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, device), "unable to sx127x_fsk_ook_rx_set_preamble_detector");
  uint8_t detector_size;
  uint8_t detector_tolerance;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_rx_get_preamble_detector(device, &enable, &detector_size, &detector_tolerance), "unable to sx127x_fsk_ook_rx_get_preamble_detector");
  TEST_ASSERT_EQUAL_MESSAGE(true, enable, "invalid enable");
  TEST_ASSERT_EQUAL_MESSAGE(2, detector_size, "invalid detector_size");
  TEST_ASSERT_EQUAL_MESSAGE(0x0A, detector_tolerance, "invalid detector_tolerance");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, 2047, device), "unable to sx127x_fsk_ook_set_packet_format");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_get_packet_format(device, &format, &max_payload_length), "unable to sx127x_fsk_ook_get_packet_format");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_FIXED, format, "invalid format");
  TEST_ASSERT_EQUAL_INT_MESSAGE(2047, max_payload_length, "invalid max_payload_length");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_OOK, device), "unable to sx127x_set_opmod OOK");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_ERR_INVALID_ARG, sx127x_fsk_ook_set_bitrate(800, device), "unable to sx127x_fsk_ook_set_bitrate");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_fsk_ook_set_bitrate(4800.0, device), "unable to sx127x_fsk_ook_set_bitrate");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_set_peak_mode(SX127X_0_5_DB, 0x0C, SX127X_1_1_CHIP, device), "unable to sx127x_ook_rx_set_peak_mode");
  sx127x_ook_thresh_type_t thresh_type;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_get_ook_thresh_type(device, &thresh_type), "unable to sx127x_ook_get_ook_thresh_type");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_OOK_PEAK, thresh_type, "invalid thresh_type");
  sx127x_ook_peak_thresh_step_t step;
  uint8_t floor_threshold;
  sx127x_ook_peak_thresh_dec_t decrement;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_get_peak_mode(device, &step, &floor_threshold, &decrement), "unable to sx127x_ook_rx_get_peak_mode");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_0_5_DB, step, "invalid step");
  TEST_ASSERT_EQUAL_MESSAGE(0x0C, floor_threshold, "invalid floor_threshold");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_1_1_CHIP, decrement, "invalid decrement");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_set_data_shaping(SX127X_1_BIT_RATE, SX127X_PA_RAMP_10, device), "unable to sx127x_ook_set_data_shaping");
  sx127x_ook_data_shaping_t ook_data_shaping;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_get_data_shaping(device, &ook_data_shaping, &pa_ramp), "unable to sx127x_ook_get_data_shaping");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_1_BIT_RATE, ook_data_shaping, "invalid ook_data_shaping");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_PA_RAMP_10, pa_ramp, "invalid pa_ramp");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_set_fixed_mode(0x11, device), "unable to sx127x_ook_rx_set_fixed_mode");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_get_ook_thresh_type(device, &thresh_type), "unable to sx127x_ook_get_ook_thresh_type");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_OOK_FIXED, thresh_type, "invalid thresh_type");
  uint8_t fixed_threshold;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_get_fixed_mode(device, &fixed_threshold), "unable to sx127x_ook_rx_get_fixed_mode");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0x11, fixed_threshold, "invalid fixed_threshold");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_set_avg_mode(SX127X_2_DB, SX127X_4_PI, device), "unable to sx127x_ook_rx_set_avg_mode");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_get_ook_thresh_type(device, &thresh_type), "unable to sx127x_ook_get_ook_thresh_type");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_OOK_AVG, thresh_type, "invalid thresh_type");
  sx127x_ook_avg_offset_t avg_offset;
  sx127x_ook_avg_thresh_t avg_thresh;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_ook_rx_get_avg_mode(device, &avg_offset, &avg_thresh), "unable to sx127x_ook_rx_get_avg_mode");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_2_DB, avg_offset, "invalid avg_offset");
  TEST_ASSERT_EQUAL_MESSAGE(SX127X_4_PI, avg_thresh, "invalid avg_thresh");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_preamble_length(8, device), "unable to sx127x_set_preamble_length");
  uint16_t preamble_length;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_preamble_length(device, &preamble_length), "unable to sx127x_get_preamble_length");
  TEST_ASSERT_EQUAL_INT_MESSAGE(8, preamble_length, "invalid preamble_length");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_AUTO, device), "unable to sx127x_rx_set_lna_gain");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_get_lna_gain(device, &lna_gain), "unable to sx127x_rx_get_lna_gain");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_LNA_GAIN_AUTO, lna_gain, "invalid lna_gain");
}

void test_registers_lora(sx127x *device) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device), "unable to sx127x_set_opmod LORA");
  sx127x_mode_t mode;
  sx127x_modulation_t modulation;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_opmod(device, &mode, &modulation), "unable to sx127x_get_opmod LORA");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_MODE_SLEEP, mode, "invalid mode");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_MODULATION_LORA, modulation, "invalid modulation");
  // LnaGain is only readable in the standby mode
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device), "unable to sx127x_set_opmod LORA");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_frequency(868200012, device), "unable to sx127x_set_frequency LORA");
  uint64_t frequency;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_frequency(device, &frequency), "unable to sx127x_get_frequency LORA");
  TEST_ASSERT_EQUAL_MESSAGE(868200000, frequency, "invalid frequency");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_bandwidth(SX127x_BW_125000, device), "unable to sx127x_lora_set_bandwidth");
  sx127x_bw_t bandwidth;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_bandwidth(device, &bandwidth), "unable to sx127x_lora_get_bandwidth");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_BW_125000, bandwidth, "invalid bandwidth");
  TEST_ASSERT_EQUAL_INT_MESSAGE(125000, sx127x_bandwidth_to_hz(bandwidth), "unable to sx127x_bandwidth_to_hz");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127x_BW_15600, sx127x_hz_to_bandwidth(15600), "unable to sx127x_hz_to_bandwidth");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_implicit_header(NULL, device), "unable to sx127x_lora_set_implicit_header");
  bool enabled;
  sx127x_implicit_header_t explicit_header;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_implicit_header(device, &explicit_header, &enabled), "unable to sx127x_lora_get_implicit_header");
  TEST_ASSERT_EQUAL_MESSAGE(false, enabled, "invalid enabled");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_spreading_factor(SX127x_SF_9, device), "unable to sx127x_lora_set_spreading_factor");
  sx127x_sf_t spreading_factor;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_spreading_factor(device, &spreading_factor), "unable to sx127x_lora_get_spreading_factor");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_SF_9, spreading_factor, "invalid spreading_factor");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_syncword(18, device), "unable to sx127x_lora_set_syncword");
  uint8_t syncword;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_syncword(device, &syncword), "unable to sx127x_lora_get_syncword");
  TEST_ASSERT_EQUAL_INT_MESSAGE(18, syncword, "invalid syncword");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_set_preamble_length(8, device), "unable to sx127x_set_preamble_length");
  uint16_t preamble_length;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_get_preamble_length(device, &preamble_length), "unable to sx127x_get_preamble_length");
  TEST_ASSERT_EQUAL_INT_MESSAGE(8, preamble_length, "invalid preamble_length");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_low_datarate_optimization(true, device), "unable to sx127x_lora_set_low_datarate_optimization");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_low_datarate_optimization(device, &enabled), "unable to sx127x_lora_get_low_datarate_optimization");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_set_lna_boost_hf(true, device), "unable to sx127x_rx_set_lna_boost_hf");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_get_lna_boost_hf(device, &enabled), "unable to sx127x_rx_get_lna_boost_hf");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device), "unable to sx127x_rx_set_lna_gain");
  sx127x_gain_t lna_gain;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_get_lna_gain(device, &lna_gain), "unable to sx127x_rx_get_lna_gain");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_LNA_GAIN_G4, lna_gain, "invalid lna_gain");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, device), "unable to sx127x_tx_set_pa_config");
  sx127x_pa_pin_t pin;
  int power;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_BOOST, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(4, power, "invalid power");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_RFO, -4, device), "unable to sx127x_tx_set_pa_config");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_RFO, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(-4, power, "invalid power");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_RFO, 15, device), "unable to sx127x_tx_set_pa_config");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_RFO, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(15, power, "invalid power");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 2, device), "unable to sx127x_tx_set_pa_config");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_BOOST, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(2, power, "invalid power");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 20, device), "unable to sx127x_tx_set_pa_config");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_BOOST, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(20, power, "invalid power");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 17, device), "unable to sx127x_tx_set_pa_config");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_pa_config(device, &pin, &power), "unable to sx127x_tx_get_pa_config");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_PA_PIN_BOOST, pin, "invalid pin");
  TEST_ASSERT_EQUAL_MESSAGE(17, power, "invalid power");

  uint8_t milliamps;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_ocp(true, 100, device), "unable to sx127x_tx_set_ocp");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_ocp(device, &enabled, &milliamps), "unable to sx127x_tx_get_ocp");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");
  TEST_ASSERT_EQUAL_MESSAGE(100, milliamps, "invalid enabled");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_ocp(true, 150, device), "unable to sx127x_tx_set_ocp");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_ocp(device, &enabled, &milliamps), "unable to sx127x_tx_get_ocp");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");
  TEST_ASSERT_EQUAL_MESSAGE(150, milliamps, "invalid milliamps");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_set_ocp(true, 250, device), "unable to sx127x_tx_set_ocp");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_tx_get_ocp(device, &enabled, &milliamps), "unable to sx127x_tx_get_ocp");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");
  TEST_ASSERT_EQUAL_MESSAGE(240, milliamps, "invalid milliamps");

  sx127x_tx_header_t header = {
      .enable_crc = true,
      .coding_rate = SX127x_CR_4_5};
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_tx_set_explicit_header(&header, device), "unable to sx127x_lora_tx_set_explicit_header");
  sx127x_tx_header_t expected_header;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_tx_get_explicit_header(device, &enabled, &expected_header), "unable to sx127x_lora_tx_get_explicit_header");
  TEST_ASSERT_EQUAL_MESSAGE(true, enabled, "invalid enabled");
  TEST_ASSERT_EQUAL_MESSAGE(true, expected_header.enable_crc, "invalid enable_crc");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_CR_4_5, expected_header.coding_rate, "invalid coding_rate");

  int32_t frequency_error;
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_set_ppm_offset(4000, device), "unable to sx127x_lora_set_ppm_offset");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_lora_get_ppm_offset(device, &frequency_error), "unable to sx127x_lora_get_ppm_offset");
  TEST_ASSERT_EQUAL_INT_MESSAGE(3655, frequency_error, "invalid frequency_error");

  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_AUTO, device), "unable to sx127x_rx_set_lna_gain");
  TEST_ASSERT_EQUAL_INT_MESSAGE(SX127X_OK, sx127x_rx_get_lna_gain(device, &lna_gain), "unable to sx127x_rx_get_lna_gain");
  TEST_ASSERT_EQUAL_MESSAGE(SX127x_LNA_GAIN_AUTO, lna_gain, "invalid lna_gain");
}