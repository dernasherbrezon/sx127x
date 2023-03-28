#ifndef sx127x_mock_spi_h
#define sx127x_mock_spi_h

#include <stddef.h>
#include <stdint.h>

void spi_mock_registers(uint8_t *expected, int code);

void spi_mock_fifo(uint8_t *expected, size_t expected_length, int code);

void spi_mock_write(int code);

void spi_assert_write(uint8_t *expected, size_t expected_length);

#endif