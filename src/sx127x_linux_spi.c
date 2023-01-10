#include <linux/spi/spidev.h>
#include <sx127x_spi.h>
#include <sys/ioctl.h>

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  // FIXME
  return 1;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  struct spi_ioc_transfer tr = {
      .tx_buf = NULL,
      .rx_buf = buffer,
      .len = buffer_length,
      .bits_per_word = 8};
  return ioctl((int)(*spi_device), SPI_IOC_MESSAGE(1), &tr);
}

int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device) {
  // FIXME
  return 1;
}

int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  struct spi_ioc_transfer tr = {
      .tx_buf = buffer,
      .rx_buf = NULL,
      .len = buffer_length,
      .bits_per_word = 8};
  return ioctl((int)(*spi_device), SPI_IOC_MESSAGE(1), &tr);
}
