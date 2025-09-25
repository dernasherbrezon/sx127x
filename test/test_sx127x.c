#include <stdlib.h>
#include <sx127x.h>
#include <string.h>
#include "unity.h"
#include <test_registers.h>
#include "sx127x_mock_spi.h"

sx127x *device = NULL;
int transmitted = 0;
int cad_status = 0;
uint8_t *registers = NULL;
uint8_t registers_length = 255;

uint8_t *rx_callback_data = NULL;
uint16_t rx_callback_data_length = 0;

void tx_callback(void *local_device) {
  transmitted = 1;
}

void rx_callback(void *local_device, uint8_t *data, uint16_t data_length) {
  rx_callback_data = data;
  rx_callback_data_length = data_length;
}

void cad_callback(void *local_device, int cad_detected) {
  cad_status = cad_detected;
}

void test_fsk_ook_rx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  sx127x_rx_set_callback(rx_callback, device, device);

  uint8_t payload[2048];
  for (int i = 1; i < (sizeof(payload) - 1); i++) {
    payload[i] = i - 1;
  }

  // 1. Small payload which should fit into FIFO
  payload[0] = 63;
  spi_mock_fifo(payload, 64, SX127X_OK);
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(payload[0], rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 1, rx_callback_data, rx_callback_data_length);

  // 2. Max payload
  payload[0] = 255;
  spi_mock_fifo(payload, 256, SX127X_OK);
  registers[0x3f] = 0b00100000;  // fifolevel
  for (int i = 0; i < 8; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(payload[0], rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 1, rx_callback_data, rx_callback_data_length);

  // 3. Small payload with address
  payload[0] = 63;
  spi_mock_fifo(payload, 64, SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(payload[0] - 1, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 2, rx_callback_data, rx_callback_data_length);

  // 4. Max payload with address
  payload[0] = 255;
  spi_mock_fifo(payload, 256, SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  registers[0x3f] = 0b00100000;  // fifolevel
  for (int i = 0; i < 8; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(payload[0] - 1, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 2, rx_callback_data, rx_callback_data_length);

  // 5. Fixed packet with small payload
  uint16_t packet_length = 64;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0x00, 0x00, device));
  spi_mock_fifo(payload, packet_length, SX127X_OK);
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(packet_length, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload, rx_callback_data, rx_callback_data_length);

  // 6. Fixed packet with small payload and specific address
  packet_length = 63;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  spi_mock_fifo(payload, packet_length + 1, SX127X_OK);
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(packet_length - 1, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 1, rx_callback_data, rx_callback_data_length);

  // 7. Fixed packet with max payload
  packet_length = 2047;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0x00, 0x00, device));
  spi_mock_fifo(payload, packet_length, SX127X_OK);
  registers[0x3f] = 0b00100000;  // fifolevel
  for (int i = 0; i < 80; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(packet_length, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload, rx_callback_data, rx_callback_data_length);

  // 8. Fixed packet with max payload and specific address
  packet_length = 2046;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  spi_mock_fifo(payload, packet_length, SX127X_OK);
  registers[0x3f] = 0b00100000;  // fifolevel
  for (int i = 0; i < 80; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00000110;  // payload_ready & crc_ok
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(packet_length - 1, rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 1, rx_callback_data, rx_callback_data_length);

  // 9. Small payload with failed CRC
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0x00, 0x00, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device));
  rx_callback_data_length = 0;
  payload[0] = 63;
  spi_mock_fifo(payload, 64, SX127X_OK);
  registers[0x3f] = 0b00000100;  // payload_ready
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(0b00010000, registers[0x3f]);  // fifo_overrun
  TEST_ASSERT_EQUAL_INT(0, rx_callback_data_length);

  // 10. Small payload with ignored CRC
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_crc(SX127X_CRC_NONE, device));
  payload[0] = 63;
  spi_mock_fifo(payload, 64, SX127X_OK);
  registers[0x3f] = 0b00000100;  // payload_ready
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(payload[0], rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload + 1, rx_callback_data, rx_callback_data_length);
}

void test_fsk_ook_beacon() {
  uint8_t data[] = {0xCA, 0xFE};
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, sizeof(data), device));
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_STATE, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 1000, device));

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, sizeof(data), device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 1000, device));
  spi_assert_write(data, sizeof(data));
  TEST_ASSERT_EQUAL_INT(243, registers[0x39]);
  TEST_ASSERT_EQUAL_INT(57, registers[0x3a]);
  TEST_ASSERT_EQUAL_INT(0b00001001, registers[0x38]);

  TEST_ASSERT_EQUAL_INT(0b10011111, registers[0x35]);
  TEST_ASSERT_EQUAL_INT(0b00010000, registers[0x3f]);
  TEST_ASSERT_EQUAL_INT(0b00001000, registers[0x31]);
  TEST_ASSERT_EQUAL_INT(0b10100100, registers[0x36]); // sequencer

  // test different timer settings
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 15, device));
  TEST_ASSERT_EQUAL_INT(117, registers[0x39]);
  TEST_ASSERT_EQUAL_INT(117, registers[0x3a]);
  TEST_ASSERT_EQUAL_INT(0b00000101, registers[0x38]);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 20, device));
  TEST_ASSERT_EQUAL_INT(156, registers[0x39]);
  TEST_ASSERT_EQUAL_INT(156, registers[0x3a]);
  TEST_ASSERT_EQUAL_INT(0b00000101, registers[0x38]);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_stop_beacon(device));
  TEST_ASSERT_EQUAL_INT(0b01000000, registers[0x36]); //stop sequencer
  TEST_ASSERT_EQUAL_INT(0b00010000, registers[0x3f]);
  TEST_ASSERT_EQUAL_INT(0b00000000, registers[0x31]);
}

void test_fsk_ook_tx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  sx127x_tx_set_callback(tx_callback, device, device);

  uint8_t payload[2048];
  for (int i = 1; i < (sizeof(payload) - 1); i++) {
    payload[i] = i - 1;
  }

  // 1. Small payload that should fit into FIFO
  payload[0] = 63;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(payload + 1, payload[0], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, payload[0] + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 2. Max payload
  transmitted = 0;
  spi_mock_write(SX127X_OK);  // reset mock buffers
  payload[0] = 255;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(payload + 1, payload[0], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00000000;  // fifolevel goes down. request for refill
  // there is protection for refill more than in the actual packet, so it is safe to call the same interrupt multiple times
  for (int i = 0; i < 10; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, payload[0] + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 3. Small payload with address
  transmitted = 0;
  spi_mock_write(SX127X_OK);  // reset mock buffers
  payload[0] = 62;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission_with_address(payload + 2, payload[0], payload[1], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00001000;
  sx127x_handle_interrupt(device);
  payload[0] = 63;  // the actual sent payload length is 1 more
  spi_assert_write(payload, payload[0] + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 4. Max payload with address
  transmitted = 0;
  spi_mock_write(SX127X_OK);
  payload[0] = 254;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission_with_address(payload + 2, payload[0], payload[1], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00000000;
  for (int i = 0; i < 10; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00001000;
  sx127x_handle_interrupt(device);
  payload[0] = 255;
  spi_assert_write(payload, payload[0] + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 5. Fixed packet with small payload
  uint16_t packet_length = 64;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  transmitted = 0;
  spi_mock_write(SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(payload, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, packet_length);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 6. Fixed packet with small payload and specific address
  transmitted = 0;
  packet_length = 63;
  spi_mock_write(SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission_with_address(payload + 1, packet_length, payload[0], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, packet_length + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 7. Fixed packet with max payload
  packet_length = 2047;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, packet_length, device));
  transmitted = 0;
  spi_mock_write(SX127X_OK);  // reset mock buffers
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission(payload, packet_length, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00000000;  // fifolevel goes down. request for refill
  // there is protection for refill more than in the actual packet, so it is safe to call the same interrupt multiple times
  for (int i = 0; i < 80; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, packet_length);
  TEST_ASSERT_EQUAL_INT(1, transmitted);

  // 8. Fixed packet with max payload and specific address
  packet_length = 2046;
  transmitted = 0;
  spi_mock_write(SX127X_OK);  // reset mock buffers
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_set_for_transmission_with_address(payload + 1, packet_length, payload[0], device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
  registers[0x3f] = 0b00000000;  // fifolevel goes down. request for refill
  // there is protection for refill more than in the actual packet, so it is safe to call the same interrupt multiple times
  for (int i = 0; i < 80; i++) {
    sx127x_handle_interrupt(device);
  }
  registers[0x3f] = 0b00001000;  // packet_sent
  sx127x_handle_interrupt(device);
  spi_assert_write(payload, packet_length + 1);
  TEST_ASSERT_EQUAL_INT(1, transmitted);
}

void test_lora_tx() {
  sx127x_tx_set_callback(tx_callback, device, device);

  transmitted = 0;

  uint8_t payload[255];
  for (int i = 0; i < sizeof(payload); i++) {
    payload[i] = i;
  }
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(payload, sizeof(payload), device));
  TEST_ASSERT_EQUAL_INT(0x00, registers[0x0d]);
  TEST_ASSERT_EQUAL_INT(sizeof(payload), registers[0x22]);
  spi_assert_write(payload, sizeof(payload));

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(0b01010000, registers[0x40]);

  // simulate interrupt
  registers[0x12] = 0b00001000;  // tx done
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(1, transmitted);
}

void test_lora_rx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(0b00000000, registers[0x40]);
  uint8_t payload[255];
  for (int i = 0; i < sizeof(payload); i++) {
    payload[i] = i;
  }
  sx127x_rx_set_callback(rx_callback, device, device);
  spi_mock_fifo(payload, sizeof(payload), SX127X_OK);
  registers[0x12] = 0b01000000;  // rx done
  registers[0x13] = sizeof(payload);
  sx127x_handle_interrupt(device);

  TEST_ASSERT_EQUAL_INT(sizeof(payload), rx_callback_data_length);
  TEST_ASSERT_EQUAL_MEMORY(payload, rx_callback_data, rx_callback_data_length);

  sx127x_implicit_header_t header = {
      .coding_rate = SX127x_CR_4_5,
      .enable_crc = true,
      .length = 2};
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(&header, device));
  TEST_ASSERT_EQUAL_INT(0b00000011, registers[0x1d]);
  TEST_ASSERT_EQUAL_INT(header.length, registers[0x22]);
  TEST_ASSERT_EQUAL_INT(0b00000100, registers[0x1e]);
  spi_mock_fifo(payload, sizeof(payload), SX127X_OK);
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(header.length, rx_callback_data_length);
}

void test_lora_cad() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(0b10000000, registers[0x40]);
  sx127x_lora_cad_set_callback(cad_callback, device, device);
  registers[0x12] = 0b00000101;  // cad detected
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(1, cad_status);

  registers[0x12] = 0b00000100;  // cad not detected
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(0, cad_status);
}

void test_fsk_ook_rssi() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));

  int16_t rssi;
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_NOT_FOUND, sx127x_rx_get_packet_rssi(device, &rssi));

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(0b11000001, registers[0x41]);

  // simulate interrupt
  registers[0x3e] = 0b00000010;
  registers[0x11] = 30;
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  TEST_ASSERT_EQUAL_INT(-15, rssi);

  // test sync address won't override rssi from preamble
  registers[0x3e] = 0b00000010;
  registers[0x11] = 30;
  sx127x_handle_interrupt(device);
  registers[0x3e] = 0b00000001;
  registers[0x11] = 33;
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  TEST_ASSERT_EQUAL_INT(-15, rssi);

  // test sync address can also be used to measure rssi
  registers[0x3e] = 0b00000001;
  registers[0x11] = 30;
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  TEST_ASSERT_EQUAL_INT(-15, rssi);
}

void test_fsk_ook() {
  test_registers_fsk_ook(device);

  registers[0x42] = SX1272_VERSION;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_create(NULL, device));
  test_registers_fsk_ook(device);

  registers[0x1b] = 0xFF;
  registers[0x1c] = 0xF0;
  int32_t frequency_error;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  TEST_ASSERT_EQUAL_INT(-976, frequency_error);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_STATE, sx127x_fsk_ook_rx_calibrate(device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_calibrate(device));
  TEST_ASSERT_EQUAL_INT(0b01000000, registers[0x3b]); // start calibration attempted

  registers[0x3c] = 22;
  int8_t raw_temperature;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_get_raw_temperature(device, &raw_temperature));
  TEST_ASSERT_EQUAL_INT(-22, raw_temperature);
  registers[0x3c] = 244;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_get_raw_temperature(device, &raw_temperature));
  TEST_ASSERT_EQUAL_INT(11, raw_temperature);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_temp_monitor(false, device));
  TEST_ASSERT_EQUAL_INT(0b01000001, registers[0x3b]); // + previous configuration
}

void test_lora() {
  test_registers_lora(device);

  registers[0x42] = SX1272_VERSION;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_create(NULL, device));
  test_registers_lora(device);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_reset_fifo(device));
  TEST_ASSERT_EQUAL_INT(0x00, registers[0x0e]);
  TEST_ASSERT_EQUAL_INT(0x00, registers[0x0f]);

  registers[0x19] = (uint8_t) (-21);
  float snr;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_rx_get_packet_snr(device, &snr));
  TEST_ASSERT_EQUAL_FLOAT(-5.25, snr);

  registers[0x1a] = 134;
  int16_t rssi;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  TEST_ASSERT_EQUAL_INT(-28, rssi);

  registers[0x28] = 0x0F;
  registers[0x29] = 0xFF;
  registers[0x2a] = 0xF0;
  int32_t frequency_error;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  TEST_ASSERT_EQUAL_INT(-2, frequency_error);
}

void test_init_failure() {
  spi_mock_registers(registers, SX127X_ERR_INVALID_ARG);
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_ARG, sx127x_create(NULL, device));
  registers[0x42] = 0x13;
  spi_mock_registers(registers, SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_VERSION, sx127x_create(NULL, device));
}

void tearDown() {
  if (device != NULL) {
    free(device);
    device = NULL;
  }
  if (registers != NULL) {
    free(registers);
    registers = NULL;
  }
  cad_status = 0;
  transmitted = 0;
  rx_callback_data = NULL;
  rx_callback_data_length = 0;
}

void setUp() {
  registers = (uint8_t *) malloc(registers_length * sizeof(uint8_t));
  memset(registers, 0, registers_length);
  spi_mock_registers(registers, SX127X_OK);
  device = malloc(sizeof(struct sx127x_t));
  registers[0x42] = SX1276_VERSION;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_create(NULL, device));
  spi_mock_write(SX127X_OK);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_lora);
  RUN_TEST(test_init_failure);
  RUN_TEST(test_fsk_ook);
  RUN_TEST(test_fsk_ook_rssi);
  RUN_TEST(test_lora_tx);
  RUN_TEST(test_lora_rx);
  RUN_TEST(test_lora_cad);
  RUN_TEST(test_fsk_ook_tx);
  RUN_TEST(test_fsk_ook_beacon);
  RUN_TEST(test_fsk_ook_rx);
  return UNITY_END();
}
