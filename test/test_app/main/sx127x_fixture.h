#ifndef SX127X_FIXTURE_H
#define SX127X_FIXTURE_H

#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <sx127x.h>
#include <stdint.h>
#include <driver/gpio.h>

#define TEST_FREQUENCY 868200000

typedef struct {
    int sck;
    int miso;
    int mosi;
    int ss;
    int rst;
    int dio0;
    int dio1;
    int dio2;
} sx127x_fixture_config_t;

typedef struct {
    sx127x *device;
    spi_device_handle_t spi_device;
    SemaphoreHandle_t tx_done;
    SemaphoreHandle_t rx_done;
    SemaphoreHandle_t xMutex;

    TaskHandle_t handle_interrupt;
    uint8_t rx_data[2048];
    uint16_t rx_data_length;
} sx127x_fixture_t;

int sx127x_fixture_create(sx127x_fixture_config_t *config, sx127x_modulation_t modulation, sx127x_fixture_t **fixture);

int sx127x_fixture_create_base(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture);

int sx127x_fixture_fsk_ook_tx_set_for_transmission(const uint8_t *data, uint16_t data_length, sx127x_fixture_t *fixture);

int sx127x_fixture_fsk_ook_tx_set_for_transmission_with_address(const uint8_t *data, uint16_t data_length, uint8_t address_to, sx127x_fixture_t *fixture);

void sx127x_fixture_destroy(sx127x_fixture_t *fixture);

void setup_gpio_interrupts(gpio_num_t gpio, sx127x_fixture_t *fixture, gpio_int_type_t type);

#endif  // SX127X_FIXTURE_H
