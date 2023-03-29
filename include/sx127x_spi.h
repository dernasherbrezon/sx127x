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
#ifndef sx127x_spi_h
#define sx127x_spi_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Read up to 4 bytes from device via SPI
 * 
 * @param reg Register
 * @param spi_device Pointer to variable to hold the device handle. Can be different on different platforms
 * @param data_length Number of bytes to read into result
 * @param result Where the data will be written to
 * @return 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result);

/**
 * @brief Read up to buffer_length bytes into buffer. Normally used for reads with more than 4 bytes
 * 
 * @param reg Register
 * @param buffer Buffer to read to
 * @param buffer_length Number of bytes to read into the buffer
 * @param spi_device Pointer to variable to hold the device handle. Can be different on different platforms
 * @return 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device);

/**
 * @brief Write data_length bytes into the register. data_length cannot be more than 4 bytes
 * 
 * @param reg Register
 * @param data Buffer to write
 * @param data_length Number of bytes to write to register
 * @param spi_device Pointer to variable to hold the device handle. Can be different on different platforms
 * @return 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_spi_write_register(int reg, uint8_t *data, size_t data_length, void *spi_device);

/**
 * @brief Write data_length bytes into the register
 * 
 * @param reg Register
 * @param buffer Buffer to write
 * @param buffer_length Number of bytes to write to register
 * @param spi_device Pointer to variable to hold the device handle. Can be different on different platforms
 * @return 
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_spi_write_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device);

#ifdef __cplusplus
}
#endif
#endif