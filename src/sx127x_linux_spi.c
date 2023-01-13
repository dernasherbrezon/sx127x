#include <errno.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sx127x_spi.h>
#include <sys/ioctl.h>

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  *result = 0;
  struct spi_ioc_transfer tr[2];
  memset(&tr, 0, sizeof(tr));
  reg = reg & 0x7F;
  tr[0].tx_buf = (unsigned long)&reg;
  tr[0].len = 1;
  tr[1].rx_buf = (unsigned long)result;
  tr[1].len = data_length;
  int code = ioctl(*(int *)spi_device, SPI_IOC_MESSAGE(2), &tr);
  printf("requested: %zu received: 0x%x code: %d to: %d\n", data_length, (*result), code, *((int *)spi_device));
  if (code == -1) {
    return errno;
  }
  return 0;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  struct spi_ioc_transfer tr[2];
  memset(&tr, 0, sizeof(tr));
  reg = reg & 0x7F;
  tr[0].tx_buf = (unsigned long)&reg;
  tr[0].len = 1;
  tr[1].rx_buf = (unsigned long)buffer;
  tr[1].len = buffer_length;
  int code = ioctl(*(int *)spi_device, SPI_IOC_MESSAGE(2), &tr);
  printf("requested: %zu code: %d to: %d\n", buffer_length, code, *((int *)spi_device));
  if (code == -1) {
    return errno;
  }
  return 0;
}

int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device) {
  struct spi_ioc_transfer tr[2];
  memset(&tr, 0, sizeof(tr));
  reg = reg | 0x80;
  tr[0].tx_buf = (unsigned long)&reg;
  tr[0].len = 1;
  tr[1].tx_buf = (unsigned long)data;
  tr[1].len = data_length;
  int code = ioctl(*(int *)spi_device, SPI_IOC_MESSAGE(2), &tr);
  printf("requested: %zu written: 0x%x code: %d to: %d\n", data_length, data[0], code, *((int *)spi_device));
  if (code == -1) {
    return errno;
  }
  return 0;
}

int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  struct spi_ioc_transfer tr[2];
  memset(&tr, 0, sizeof(tr));
  reg = reg | 0x80;
  tr[0].tx_buf = (unsigned long)&reg;
  tr[0].len = 1;
  tr[1].tx_buf = (unsigned long)buffer;
  tr[1].len = buffer_length;
  int code = ioctl(*(int *)spi_device, SPI_IOC_MESSAGE(2), &tr);
  printf("requested: %zu code: %d to: %d\n", buffer_length, code, *((int *)spi_device));
  if (code == -1) {
    return errno;
  }
  return 0;
}
