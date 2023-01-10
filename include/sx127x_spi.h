#ifndef sx127x_spi_h
#define sx127x_spi_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result);
int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device);
int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device);
int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device);

#ifdef __cplusplus
}
#endif
#endif