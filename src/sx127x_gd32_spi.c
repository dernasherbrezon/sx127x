#if defined(GD32F10x)
#include "gd32f10x.h"
#elif defined(GD32F1x0)
#include "gd32f1x0.h"
#elif defined(GD32F20x)
#include "gd32f20x.h"
#elif defined(GD32F3x0)
#include "gd32f3x0.h"
#elif defined(GD32F30x)
#include "gd32f30x.h"
#elif defined(GD32F4xx)
#include "gd32f4xx.h"
#elif defined(GD32F403)
#include "gd32f403.h"
#elif defined(GD32E10X)
#include "gd32e10x.h"
#elif defined(GD32E23x)
#include "gd32e23x.h"
#elif defined(GD32E50X)
#include "gd32e50x.h"
#elif defined(GD32L23x)
#include "gd32l23x.h"
#elif defined(GD32W51x)
#include "gd32w51x.h"
#else
#error "Unknown chip series"
#endif
#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define SPI_TIMEOUT 10000

#define ERROR_CHECK(x)                             \
  do {                                             \
    int __err_rc = (x);                            \
    if (__err_rc != 0) {                           \
      spi_nss_internal_high((uint32_t)spi_device); \
      return __err_rc;                             \
    }                                              \
  } while (0)

int spi_master_write(void *spi_device, uint8_t value, uint8_t *result) {
  int count = 0;
  /* wait the SPI transmit buffer is empty */
  // FIXME potentially remove count++?
  while ((0 == spi_i2s_flag_get((uint32_t)spi_device, SPI_FLAG_TBE)) && (count++ < 1000));
  if (count >= 1000) {
    *result = 0;
    return -1;
  }
  spi_i2s_data_transmit((uint32_t)spi_device, value);

  count = 0;
  /* wait the SPI receive buffer is not empty */
  while ((0 == spi_i2s_flag_get((uint32_t)spi_device, SPI_FLAG_RBNE)) && (count++ < 1000));
  if (count >= 1000) {
    *result = 0;
    return -1;
  }
  *result = spi_i2s_data_receive((uint32_t)spi_device);
  return 0;
}

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  spi_nss_internal_low((uint32_t)spi_device);
  uint8_t response;
  ERROR_CHECK(spi_master_write(spi_device, ((uint8_t)reg & 0x7F), &response));
  for (size_t i = 0; i < data_length; i++) {
    ERROR_CHECK(spi_master_write(spi_device, 0, &response));
    *result |= (response << (8 * i));
  }
  spi_nss_internal_high((uint32_t)spi_device);
  return 0;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  if (buffer_length < 1) {
    return 0;
  }
  spi_nss_internal_low((uint32_t)spi_device);
  uint8_t response;
  ERROR_CHECK(spi_master_write(spi_device, ((uint8_t)reg & 0x7F), &response));
  for (size_t i = 0; i < buffer_length; i++) {
    ERROR_CHECK(spi_master_write(spi_device, 0, &response));
    buffer[i] = response;
  }
  spi_nss_internal_high((uint32_t)spi_device);
  return 0;
}

int sx127x_spi_write_register(int reg, const uint8_t *data, size_t data_length, void *spi_device) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  spi_nss_internal_low((uint32_t)spi_device);
  uint8_t response;
  ERROR_CHECK(spi_master_write(spi_device, ((uint8_t)reg | 0x80), &response));
  for (size_t i = 0; i < data_length; i++) {
    ERROR_CHECK(spi_master_write(spi_device, data[i], &response));
  }
  spi_nss_internal_high((uint32_t)spi_device);
  return 0;
}

int sx127x_spi_write_buffer(int reg, const uint8_t *buffer, size_t buffer_length, void *spi_device) {
  if (buffer_length < 1) {
    return 0;
  }
  spi_nss_internal_low((uint32_t)spi_device);
  uint8_t response;
  ERROR_CHECK(spi_master_write(spi_device, ((uint8_t)reg | 0x80), &response));
  for (size_t i = 0; i < buffer_length; i++) {
    ERROR_CHECK(spi_master_write(spi_device, buffer[i], &response));
  }
  spi_nss_internal_high((uint32_t)spi_device);
  return 0;
}
