#ifndef SX127X_FIXTURE_H
#define SX127X_FIXTURE_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <sx127x.h>

typedef struct {
  int sck;
  int miso;
  int mosi;
  int ss;
  int rst;
  int dio0;
} sx127x_fixture_config_t;

typedef struct {
  sx127x *device;
  SemaphoreHandle_t tx_done;
  SemaphoreHandle_t rx_done;
  SemaphoreHandle_t cad_done;

  TaskHandle_t handle_interrupt;
} sx127x_fixture_t;

int sx127x_fixture_create(sx127x_fixture_config_t *config, sx127x_fixture_t **fixture);

void sx127x_fixture_destroy(sx127x_fixture_t *fixture);

void sx127x_fixture_rx_callback(sx127x *device, uint8_t *data, uint16_t data_length);

#endif  // SX127X_FIXTURE_H
