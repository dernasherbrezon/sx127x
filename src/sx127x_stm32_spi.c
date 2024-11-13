#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define SPI_TIMEOUT 10000

// stm32 doesn't have a common header where SPI functions defined
// thus define them as "extern" here
extern int HAL_SPI_Transmit(void *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
extern int HAL_SPI_TransmitReceive(void *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  uint64_t tx_buf = ((uint8_t)reg & 0x7F);
  uint64_t rx_buf = 0;
  int code = HAL_SPI_TransmitReceive(spi_device, (uint8_t *)&tx_buf, (uint8_t *)&rx_buf, data_length + 1, SPI_TIMEOUT);
  if (code != 0) {
    *result = 0;
    return code;
  }
  *result = __builtin_bswap32(rx_buf >> 8) >> (4 - data_length) * 8;
  return 0;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  if (buffer_length < 1) {
    return 0;
  }
  uint8_t *tmp = malloc(buffer_length + 1);
  if (tmp == NULL) {
    return ENOMEM;
  }
  uint64_t tx_buf = ((uint8_t)reg & 0x7F);
  int code = HAL_SPI_TransmitReceive(spi_device, (uint8_t *)&tx_buf, (uint8_t *)&tmp, buffer_length + 1, SPI_TIMEOUT);
  if (code != 0) {
    free(tmp);
    return code;
  }
  // shift 1 byte to the right
  memcpy(buffer, tmp + 1, buffer_length);
  free(tmp);
  return 0;
}

int sx127x_spi_write_register(int reg, const uint8_t *data, size_t data_length, void *spi_device) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  uint8_t tmp[5] = {0};
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, data, data_length);
  return HAL_SPI_Transmit(spi_device, tmp, data_length + 1, SPI_TIMEOUT);
}

int sx127x_spi_write_buffer(int reg, const uint8_t *buffer, size_t buffer_length, void *spi_device) {
  uint8_t *tmp = malloc(buffer_length + 1);
  if (tmp == NULL) {
    return ENOMEM;
  }
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, buffer, buffer_length);
  int code = HAL_SPI_Transmit(spi_device, tmp, buffer_length + 1, SPI_TIMEOUT);
  free(tmp);
  return code;
}
