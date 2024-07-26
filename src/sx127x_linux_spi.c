// Copyright 2022 Andrey Rodionov <dernasherbrezon@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <arpa/inet.h>
#include <errno.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <sx127x_spi.h>
#include <sys/ioctl.h>
#include <stdlib.h>

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  uint64_t tx_buf = ((uint8_t) reg & 0x7F);
  uint64_t rx_buf = 0;
  tr.tx_buf = (__u64) &tx_buf;
  tr.rx_buf = (__u64) &rx_buf;
  tr.len = data_length + 1;
  int code = ioctl(*(int *) spi_device, SPI_IOC_MESSAGE(1), &tr);
  if (code == -1) {
    *result = 0;
    return errno;
  }
  // first >> 8 shift is to remove garbage received during tx
  // second shift is to actually trim uint32 to the expected length
  *result = ntohl(rx_buf >> 8) >> (4 - data_length) * 8;
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
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  uint64_t tx_buf = ((uint8_t) reg & 0x7F);
  tr.tx_buf = (__u64) &tx_buf;
  tr.rx_buf = (__u64) tmp;
  tr.len = buffer_length + 1;
  int code = ioctl(*(int *) spi_device, SPI_IOC_MESSAGE(1), &tr);
  if (code == -1) {
    free(tmp);
    return errno;
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
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  uint8_t tmp[5] = {0};
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, data, data_length);
  tr.tx_buf = (unsigned long) tmp;
  tr.len = data_length + 1;
  int code = ioctl(*(int *) spi_device, SPI_IOC_MESSAGE(1), &tr);
  if (code == -1) {
    return errno;
  }
  return 0;
}

int sx127x_spi_write_buffer(int reg, const uint8_t *buffer, size_t buffer_length, void *spi_device) {
  uint8_t *tmp = malloc(buffer_length + 1);
  if (tmp == NULL) {
    return ENOMEM;
  }
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, buffer, buffer_length);
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = (__u64) tmp;
  tr.len = buffer_length + 1;
  int code = ioctl(*(int *) spi_device, SPI_IOC_MESSAGE(1), &tr);
  free(tmp);
  if (code == -1) {
    return errno;
  }
  return 0;
}
