#ifndef esp_utils_h
#define esp_utils_h

#include <sx127x.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

// TTGO lora32 v2.1 1.6.1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define DIO0 26
#define RST 23
// older versions of TTGO require manual wiring of pins below
#define DIO1 33
#define DIO2 32

// Heltec lora32 v2
//#define RST 14
//#define DIO1 35
//#define DIO2 34

// Make sure radio parameters are the same across multiple tests and examples
#define TEST_FREQUENCY 868200000

void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device, gpio_int_type_t type);
void rx_callback(void *device, uint8_t *data, uint16_t data_length);
void lora_rx_callback(void *device, uint8_t *data, uint16_t data_length);
void cad_callback(void *device, int cad_detected);
void sx127x_reset();
void sx127x_init_spi(spi_device_handle_t *handle);
esp_err_t setup_task(sx127x *device);
esp_err_t setup_tx_task(sx127x *device, void (*tx_callback)(sx127x *device));

#endif