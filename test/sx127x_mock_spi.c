#include "unity.h"
#include <string.h>
#include <sx127x_spi.h>

uint8_t sx127x_mock_expected_response[4096];
size_t sx127x_mock_expected_response_length = 0;
size_t sx127x_mock_expected_response_current = 0;
int sx127x_mock_expected_code = 0;

uint8_t actual_request[4096];
size_t sx127x_mock_actual_request_length = 0;
int sx127x_mock_expected_write_code = 0;

uint8_t *sx127x_mock_registers;

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  *result = 0;
  if (reg == 0) {
    for (int i = 0; i < data_length; i++) {
      *result = ((*result) << 8);
      *result = (*result) + sx127x_mock_expected_response[sx127x_mock_expected_response_current + i];
    }
    sx127x_mock_expected_response_current += data_length;
    if (sx127x_mock_expected_response_current == sx127x_mock_expected_response_length) {
      sx127x_mock_registers[0x3f] = 0b01000000;  // fifo empty
    }
  } else {
    for (int i = 0; i < data_length; i++) {
      *result = ((*result) << 8);
      *result = (*result) + sx127x_mock_registers[reg + i];
    }
  }
  return sx127x_mock_expected_code;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  TEST_ASSERT_EQUAL_INT(sx127x_mock_expected_response_current + buffer_length <= sx127x_mock_expected_response_length, 1);
  memcpy(buffer, sx127x_mock_expected_response + sx127x_mock_expected_response_current, buffer_length);
  sx127x_mock_expected_response_current += buffer_length;
  if (sx127x_mock_expected_response_current == sx127x_mock_expected_response_length) {
    sx127x_mock_registers[0x3f] = 0b01000000;  // fifo empty
  }
  return sx127x_mock_expected_code;
}

int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device) {
  if (reg == 0) {
    return sx127x_spi_write_buffer(reg, data, data_length, spi_device);
  }
  for (size_t i = 0; i < data_length; i++) {
    sx127x_mock_registers[reg + i] = data[i];
  }
  return sx127x_mock_expected_write_code;
}

int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  // not fifo
  if (reg != 0) {
    return sx127x_spi_write_register(reg, buffer, buffer_length, spi_device);
  }
  memcpy(actual_request + sx127x_mock_actual_request_length, buffer, buffer_length);
  sx127x_mock_actual_request_length += buffer_length;
  return sx127x_mock_expected_write_code;
}

void spi_mock_registers(uint8_t *expected, int code) {
  sx127x_mock_registers = expected;
  sx127x_mock_expected_code = code;
}

void spi_mock_fifo(uint8_t *expected, size_t expected_length, int code) {
  memcpy(sx127x_mock_expected_response, expected, expected_length);
  sx127x_mock_expected_response_length = expected_length;
  sx127x_mock_expected_response_current = 0;
  sx127x_mock_expected_code = code;
}

void spi_assert_write(uint8_t *expected, size_t expected_length) {
  TEST_ASSERT_EQUAL_INT(expected_length, sx127x_mock_actual_request_length);
  for (size_t i = 0; i < expected_length; i++) {
    TEST_ASSERT_EQUAL_INT(expected[i], actual_request[i]);
  }
}

void spi_mock_write(int code) {
  sx127x_mock_expected_write_code = code;
  sx127x_mock_actual_request_length = 0;
}
