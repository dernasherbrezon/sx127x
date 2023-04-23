#ifndef SX127X_TEST_UTILS_H
#define SX127X_TEST_UTILS_H

#include <sx127x.h>
#include <stdlib.h>

//sx127x *global_device = NULL;

typedef struct {
    int sck;
    int miso;
    int mosi;
    int ss;
    int rst;
    int dio0;
} sx127x_test_spi_config_t;

int sx127x_test_create_lora(sx127x_test_spi_config_t *config, sx127x **device);

void sx127x_test_wait_for_tx();

#endif //SX127X_TEST_UTILS_H
