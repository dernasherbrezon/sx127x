#include <stdlib.h>
#include <sx127x.h>
#include <string.h>
#include "unity.h"

#include "sx127x_mock_spi.h"

sx127x *device = NULL;
int transmitted = 0;
int cad_status = 0;
uint8_t *registers = NULL;
uint8_t registers_length = 255;

uint8_t *rx_callback_data = NULL;
uint16_t rx_callback_data_length = 0;

void tx_callback(sx127x *local_device) {
  transmitted = 1;
}

void rx_callback(sx127x *local_device, uint8_t *data, uint16_t data_length) {
  rx_callback_data = data;
  rx_callback_data_length = data_length;
}

void cad_callback(sx127x *local_device, int cad_detected) {
  cad_status = cad_detected;
}

void test_fsk_ook_rx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  sx127x_rx_set_callback(rx_callback, device);

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
  TEST_ASSERT_EQUAL_INT(registers[0x3f], 0b00010000);  // fifo_overrun
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
  TEST_ASSERT_EQUAL_INT(registers[0x39], 243);
  TEST_ASSERT_EQUAL_INT(registers[0x3a], 57);
  TEST_ASSERT_EQUAL_INT(registers[0x38], 0b00001001);

  TEST_ASSERT_EQUAL_INT(registers[0x35], 0b10011111);
  TEST_ASSERT_EQUAL_INT(registers[0x3f], 0b00010000);
  TEST_ASSERT_EQUAL_INT(registers[0x31], 0b00001000);
  TEST_ASSERT_EQUAL_INT(registers[0x36], 0b10100100); // sequencer

  // test different timer settings
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 15, device));
  TEST_ASSERT_EQUAL_INT(registers[0x39], 234);
  TEST_ASSERT_EQUAL_INT(registers[0x3a], 0);
  TEST_ASSERT_EQUAL_INT(registers[0x38], 0b00000111);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_start_beacon(data, sizeof(data), 20, device));
  TEST_ASSERT_EQUAL_INT(registers[0x39], 156);
  TEST_ASSERT_EQUAL_INT(registers[0x3a], 156);
  TEST_ASSERT_EQUAL_INT(registers[0x38], 0b00000101);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_tx_stop_beacon(device));
  TEST_ASSERT_EQUAL_INT(registers[0x36], 0b01000000); //stop sequencer
  TEST_ASSERT_EQUAL_INT(registers[0x3f], 0b00010000);
  TEST_ASSERT_EQUAL_INT(registers[0x31], 0b00000000);
}

void test_fsk_ook_tx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  sx127x_tx_set_callback(tx_callback, device);

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
  sx127x_tx_set_callback(tx_callback, device);

  transmitted = 0;

  uint8_t payload[255];
  for (int i = 0; i < sizeof(payload); i++) {
    payload[i] = i;
  }
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_for_transmission(payload, sizeof(payload), device));
  TEST_ASSERT_EQUAL_INT(registers[0x0d], 0x00);
  TEST_ASSERT_EQUAL_INT(registers[0x22], sizeof(payload));
  spi_assert_write(payload, sizeof(payload));

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(registers[0x40], 0b01010000);

  // simulate interrupt
  registers[0x12] = 0b00001000;  // tx done
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(1, transmitted);
}

void test_lora_rx() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(registers[0x40], 0b00000000);
  uint8_t payload[255];
  for (int i = 0; i < sizeof(payload); i++) {
    payload[i] = i;
  }
  sx127x_rx_set_callback(rx_callback, device);
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
  TEST_ASSERT_EQUAL_INT(registers[0x1d], 0b00000011);
  TEST_ASSERT_EQUAL_INT(registers[0x22], header.length);
  TEST_ASSERT_EQUAL_INT(registers[0x1e], 0b00000100);
  spi_mock_fifo(payload, sizeof(payload), SX127X_OK);
  sx127x_handle_interrupt(device);
  TEST_ASSERT_EQUAL_INT(header.length, rx_callback_data_length);
}

void test_lora_cad() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(registers[0x40], 0b10000000);
  sx127x_lora_cad_set_callback(cad_callback, device);
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
  TEST_ASSERT_EQUAL_INT(registers[0x41], 0b11000001);

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
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(registers[0x01], 0b00000000);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_frequency(437200012, device));
  TEST_ASSERT_EQUAL_INT(registers[0x06], 0x6d);
  TEST_ASSERT_EQUAL_INT(registers[0x07], 0x4c);
  TEST_ASSERT_EQUAL_INT(registers[0x08], 0xcd);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_bitrate(4800.0, device));
  TEST_ASSERT_EQUAL_INT(registers[0x02], 0x1A);
  TEST_ASSERT_EQUAL_INT(registers[0x03], 0x0A);
  TEST_ASSERT_EQUAL_INT(registers[0x5d], 0x0A);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_set_fdev(5000.0, device));
  TEST_ASSERT_EQUAL_INT(registers[0x04], 0x00);
  TEST_ASSERT_EQUAL_INT(registers[0x05], 0x51);
  uint8_t syncWord[] = {0x12, 0xAD};
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_syncword(syncWord, 2, device));
  TEST_ASSERT_EQUAL_INT(registers[0x28], 0x12);
  TEST_ASSERT_EQUAL_INT(registers[0x29], 0xAD);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_encoding(SX127X_SCRAMBLED, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NODE_AND_BROADCAST, 0x11, 0x12, device));
  TEST_ASSERT_EQUAL_INT(registers[0x33], 0x11);
  TEST_ASSERT_EQUAL_INT(registers[0x34], 0x12);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  TEST_ASSERT_EQUAL_INT(registers[0x31], 0b00000000);
  TEST_ASSERT_EQUAL_INT(registers[0x32], 0xFF);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_preamble_type(SX127X_PREAMBLE_55, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_afc_auto(true, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, device));
  TEST_ASSERT_EQUAL_INT(registers[0x13], 0x14);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_bandwidth(5000.0, device));
  TEST_ASSERT_EQUAL_INT(registers[0x12], 0x16);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_collision_restart(true, 10, device));
  TEST_ASSERT_EQUAL_INT(registers[0x0f], 10);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, device));
  TEST_ASSERT_EQUAL_INT(registers[0x30], 0b11011100);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_packet_format(SX127X_FIXED, 2047, device));
  TEST_ASSERT_EQUAL_INT(registers[0x31], 0b00000111);
  TEST_ASSERT_EQUAL_INT(registers[0x32], 0xFF);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_OOK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_ARG, sx127x_fsk_ook_set_bitrate(800, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_bitrate(4800.0, device));
  TEST_ASSERT_EQUAL_INT(registers[0x02], 0x1A);
  TEST_ASSERT_EQUAL_INT(registers[0x03], 0x0A);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_ook_rx_set_peak_mode(SX127X_0_5_DB, 0x0C, SX127X_1_1_CHIP, device));

  TEST_ASSERT_EQUAL_INT(registers[0x0d], 0b10010111);
  TEST_ASSERT_EQUAL_INT(registers[0x0c], 0b10000000);
  TEST_ASSERT_EQUAL_INT(registers[0x27], 0b01110001);
  TEST_ASSERT_EQUAL_INT(registers[0x0a], 0b01001001);
  TEST_ASSERT_EQUAL_INT(registers[0x0e], 0b00000010);
  TEST_ASSERT_EQUAL_INT(registers[0x1f], 0b10101010);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_ook_set_data_shaping(SX127X_1_BIT_RATE, SX127X_PA_RAMP_10, device));
  TEST_ASSERT_EQUAL_INT(registers[0x0a], 0b00101001);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_ook_rx_set_fixed_mode(0x11, device));
  TEST_ASSERT_EQUAL_INT(registers[0x14], 0b00000000);
  TEST_ASSERT_EQUAL_INT(registers[0x15], 0x11);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_ook_rx_set_avg_mode(SX127X_2_DB, SX127X_4_PI, device));
  TEST_ASSERT_EQUAL_INT(registers[0x14], 0b00010000);
  TEST_ASSERT_EQUAL_INT(registers[0x16], 0b00000110);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_preamble_length(8, device));
  TEST_ASSERT_EQUAL_INT(registers[0x25], 0x00);
  TEST_ASSERT_EQUAL_INT(registers[0x26], 0x08);

  registers[0x1b] = 0xFF;
  registers[0x1c] = 0xF0;
  int32_t frequency_error;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  TEST_ASSERT_EQUAL_INT(-976, frequency_error);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_STATE, sx127x_fsk_ook_rx_calibrate(device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_rx_calibrate(device));
  TEST_ASSERT_EQUAL_INT(registers[0x3b], 0b01000000); // start calibration attempted

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_AUTO, device));
  TEST_ASSERT_EQUAL_INT(registers[0x0d], 0b10011111); // + previous configuration

  registers[0x3c] = 22;
  int8_t raw_temperature;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_get_raw_temperature(device, &raw_temperature));
  TEST_ASSERT_EQUAL_INT(-22, raw_temperature);
  registers[0x3c] = 244;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_get_raw_temperature(device, &raw_temperature));
  TEST_ASSERT_EQUAL_INT(11, raw_temperature);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_fsk_ook_set_temp_monitor(false, device));
  TEST_ASSERT_EQUAL_INT(registers[0x3b], 0b01000001); // + previous configuration
}

void test_lora() {
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  TEST_ASSERT_EQUAL_INT(registers[0x01], 0b10000000);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_frequency(437200012, device));
  TEST_ASSERT_EQUAL_INT(registers[0x06], 0x6d);
  TEST_ASSERT_EQUAL_INT(registers[0x07], 0x4c);
  TEST_ASSERT_EQUAL_INT(registers[0x08], 0xcd);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_reset_fifo(device));
  TEST_ASSERT_EQUAL_INT(registers[0x0e], 0x00);
  TEST_ASSERT_EQUAL_INT(registers[0x0f], 0x00);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_bandwidth(SX127x_BW_125000, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_implicit_header(NULL, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_modem_config_2(SX127x_SF_9, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_syncword(18, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_set_preamble_length(8, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_low_datarate_optimization(true, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_set_lna_boost_hf(true, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 4, device));

  TEST_ASSERT_EQUAL_INT(registers[0x1d], 0b01110000);
  TEST_ASSERT_EQUAL_INT(registers[0x31], 0xc3);
  TEST_ASSERT_EQUAL_INT(registers[0x37], 0x0a);
  TEST_ASSERT_EQUAL_INT(registers[0x1e], 0b10010000);
  TEST_ASSERT_EQUAL_INT(registers[0x39], 18);
  TEST_ASSERT_EQUAL_INT(registers[0x20], 0);
  TEST_ASSERT_EQUAL_INT(registers[0x21], 8);
  TEST_ASSERT_EQUAL_INT(registers[0x26], 0b00001000);
  TEST_ASSERT_EQUAL_INT(registers[0x0c], 0b10000011);
  TEST_ASSERT_EQUAL_INT(registers[0x4d], 0b10000100);
  TEST_ASSERT_EQUAL_INT(registers[0x09], 0b10000010);
  TEST_ASSERT_EQUAL_INT(registers[0x0b], 0x28);

  uint32_t bandwidth;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_get_bandwidth(device, &bandwidth));
  TEST_ASSERT_EQUAL_INT(125000, bandwidth);

  registers[0x19] = (uint8_t) (-21);
  float snr;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_rx_get_packet_snr(device, &snr));
  TEST_ASSERT_EQUAL_FLOAT(-5.25, snr);

  registers[0x1a] = 134;
  int16_t rssi;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_packet_rssi(device, &rssi));
  TEST_ASSERT_EQUAL_INT(-35, rssi);

  registers[0x28] = 0x0F;
  registers[0x29] = 0xFF;
  registers[0x2a] = 0xF0;
  int32_t frequency_error;
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_get_frequency_error(device, &frequency_error));
  TEST_ASSERT_EQUAL_INT(-2, frequency_error);

  sx127x_tx_header_t header = {
      .enable_crc = true,
      .coding_rate = SX127x_CR_4_5};
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_tx_set_explicit_header(&header, device));
  TEST_ASSERT_EQUAL_INT(registers[0x1d], 0b01110010);
  TEST_ASSERT_EQUAL_INT(registers[0x1e], 0b10010100);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_lora_set_ppm_offset(4000, device));
  TEST_ASSERT_EQUAL_INT(registers[0x27], 8);

  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_AUTO, device));
  TEST_ASSERT_EQUAL_INT(registers[0x26], 0b00001100); // + previous config
}

void test_init_failure() {
  spi_mock_registers(registers, SX127X_ERR_INVALID_ARG);
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_ARG, sx127x_create(NULL, &device));
  registers[0x42] = 0x13;
  spi_mock_registers(registers, SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_ERR_INVALID_VERSION, sx127x_create(NULL, &device));
}

void tearDown() {
  if (device != NULL) {
    sx127x_destroy(device);
    sx127x_destroy(NULL); // no-op. check no panic or access violation
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
  registers[0x42] = 0x12;
  spi_mock_registers(registers, SX127X_OK);
  TEST_ASSERT_EQUAL_INT(SX127X_OK, sx127x_create(NULL, &device));
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
