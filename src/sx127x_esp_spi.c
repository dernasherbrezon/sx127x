#include <driver/spi_master.h>
#include <esp_err.h>
#include <sx127x_spi.h>

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return ESP_ERR_INVALID_ARG;
  }
  *result = 0;
  spi_transaction_t t = {
      .addr = reg & 0x7F,
      .rx_buffer = NULL,
      .tx_buffer = NULL,
      .rxlength = data_length * 8,
      .length = data_length * 8,
      .flags = SPI_TRANS_USE_RXDATA};
  esp_err_t code = spi_device_polling_transmit(spi_device, &t);
  if (code != ESP_OK) {
    return code;
  }
  for (int i = 0; i < data_length; i++) {
    *result = ((*result) << 8);
    *result = (*result) + t.rx_data[i];
  }
  return ESP_OK;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  spi_transaction_t t = {
      .addr = reg & 0x7F,
      .rx_buffer = buffer,
      .tx_buffer = NULL,
      .rxlength = buffer_length * 8,
      .length = buffer_length * 8};
  return spi_device_polling_transmit(spi_device, &t);
}

int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device) {
  if (data_length == 0 || data_length > 4) {
    return ESP_ERR_INVALID_ARG;
  }
  spi_transaction_t t = {
      .addr = reg | 0x80,
      .rx_buffer = NULL,
      .tx_buffer = NULL,
      .rxlength = data_length * 8,
      .length = data_length * 8,
      .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA};
  for (int i = 0; i < data_length; i++) {
    t.tx_data[i] = data[i];
  }
  return spi_device_polling_transmit(spi_device, &t);
}

int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  spi_transaction_t t = {
      .addr = reg | 0x80,
      .rx_buffer = NULL,
      .tx_buffer = buffer,
      .rxlength = buffer_length * 8,
      .length = buffer_length * 8};
  return spi_device_polling_transmit(spi_device, &t);
}
