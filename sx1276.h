#ifndef sx1276_h
#define sx1276_h

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_master.h>
#include <stdint.h>

#define SX1276_ERROR_INVALID_CHIP   1
#define SX1276_LORA_MODE_FSK        0b00000000
#define SX1276_LORA_MODE_LORA       0b10000000
#define SX1276_MODULATION_TYPE_FSK  0b00000000
#define SX1276_MODULATION_TYPE_OOK  0b00100000
#define SX1276_MODE_SLEEP           0b00000000
#define SX1276_MODE_STANDBY         0b00000001
#define SX1276_MODE_FSTX            0b00000010
#define SX1276_MODE_TX              0b00000011
#define SX1276_MODE_FSRX            0b00000100
#define SX1276_MODE_RX              0b00000101

#define SX1276_LNA_GAIN_G1          0b00100000
#define SX1276_LNA_GAIN_G2          0b01000000
#define SX1276_LNA_GAIN_G3          0b01100000
#define SX1276_LNA_GAIN_G4          0b10000000
#define SX1276_LNA_GAIN_G5          0b10100000
#define SX1276_LNA_GAIN_G6          0b11000000
#define SX1276_LNA_GAIN_AUTO        0b00000000

#define SX1276_LNA_BOOST_HF_ON      0b00000011
#define SX1276_LNA_BOOST_HF_OFF     0b00000000

#define SX1276_BW_7800              0b00000000
#define SX1276_BW_10400             0b00010000
#define SX1276_BW_15600             0b00100000
#define SX1276_BW_20800             0b00110000
#define SX1276_BW_31250             0b01000000
#define SX1276_BW_41700             0b01010000
#define SX1276_BW_62500             0b01100000
#define SX1276_BW_125000            0b01110000
#define SX1276_BW_250000            0b10000000
#define SX1276_BW_500000            0b10010000

#define SX1276_CR_4_5               0b00000010
#define SX1276_CR_4_6               0b00000100
#define SX1276_CR_4_7               0b00000110
#define SX1276_CR_4_8               0b00001000

#define SX1276_HEADER_MODE_EXPLICIT 0b00000000
#define SX1276_HEADER_MODE_IMPLICIT 0b00000001

#define SX1276_LOW_DATARATE_OPTIMIZATION_ON  0b00001000
#define SX1276_LOW_DATARATE_OPTIMIZATION_OFF 0b00000000

#define SX1276_SF_6                 0b01100000
#define SX1276_SF_7                 0b01110000
#define SX1276_SF_8                 0b10000000
#define SX1276_SF_9                 0b10010000
#define SX1276_SF_10                0b10100000
#define SX1276_SF_11                0b10110000
#define SX1276_SF_12                0b11000000

#define SX1276_TX_MODE_SINGLE       0b00000000
#define SX1276_TX_MODE_CONTINUOUS   0b00001000

#define SX1276_RX_PAYLOAD_CRC_ON    0b00000100
#define SX1276_RX_PAYLOAD_CRC_OFF   0b00000000

typedef struct sx127x_t sx127x;

int sx127x_create(spi_host_device_t host, int cs, sx127x **result);

int sx1276_is_lora(bool *result, sx127x *device);
int sx1276_get_modulation_type(uint8_t *result, sx127x *device);
int sx1276_get_mode(uint8_t *result, sx127x *device);
int sx1276_set_opmod(uint8_t opmod, sx127x *device);
int sx1276_set_frequency(long frequency, sx127x *device);
int sx1276_reset_fifo(sx127x *device);
int sx1276_set_lna_gain(uint8_t gain, sx127x *device);
int sx1276_set_lna_boost_hf(uint8_t value, sx127x *device);
int sx1276_set_modem_config_1(uint8_t value, sx127x *device);
int sx1276_set_modem_config_2(uint8_t value, sx127x *device);
int sx1276_set_low_datarate_optimization(uint8_t value, sx127x *device);
int sx1276_set_syncword(uint8_t value, sx127x *device);
int sx1276_set_preamble_length(uint16_t value, sx127x *device);
void sx1276_handle_interrupt(void *arg);
int sx1276_receive(sx127x *device);

void sx127x_destroy(sx127x *device);

#ifdef __cplusplus
}
#endif

#endif