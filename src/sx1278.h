#ifndef sx1278_h
#define sx1278_h

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_master.h>
#include <esp_err.h>
#include <stdint.h>

typedef enum {
  SX1278_MODE_SLEEP = 0b00000000,
  SX1278_MODE_STANDBY = 0b00000001,
  SX1278_MODE_FSTX = 0b00000010,
  SX1278_MODE_TX = 0b00000011,
  SX1278_MODE_FSRX = 0b00000100,
  SX1278_MODE_RX_CONT = 0b00000101,
  SX1278_MODE_RX_SINGLE = 0b00000110,
  SX1278_MODE_CAD = 0b00000111
} sx1278_mode_t;

typedef enum {
  SX1278_LNA_GAIN_G1 = 0b00100000,
  SX1278_LNA_GAIN_G2 = 0b01000000,
  SX1278_LNA_GAIN_G3 = 0b01100000,
  SX1278_LNA_GAIN_G4 = 0b10000000,
  SX1278_LNA_GAIN_G5 = 0b10100000,
  SX1278_LNA_GAIN_G6 = 0b11000000,
  SX1278_LNA_GAIN_AUTO = 0b00000000
} sx1278_gain_t;

typedef enum {
  SX1278_LNA_BOOST_HF_ON = 0b00000011,
  SX1278_LNA_BOOST_HF_OFF = 0b00000000
} sx1278_lna_boost_hf_t;

typedef enum {
  SX1278_BW_7800 = 0b00000000,
  SX1278_BW_10400 = 0b00010000,
  SX1278_BW_15600 = 0b00100000,
  SX1278_BW_20800 = 0b00110000,
  SX1278_BW_31250 = 0b01000000,
  SX1278_BW_41700 = 0b01010000,
  SX1278_BW_62500 = 0b01100000,
  SX1278_BW_125000 = 0b01110000,
  SX1278_BW_250000 = 0b10000000,
  SX1278_BW_500000 = 0b10010000
} sx1278_bw_t;

typedef enum {
  SX1278_CR_4_5 = 0b00000010,
  SX1278_CR_4_6 = 0b00000100,
  SX1278_CR_4_7 = 0b00000110,
  SX1278_CR_4_8 = 0b00001000
} sx1278_cr_t;

typedef enum {
  SX1278_LOW_DATARATE_OPTIMIZATION_ON = 0b00001000,
  SX1278_LOW_DATARATE_OPTIMIZATION_OFF = 0b00000000
} sx1278_low_datarate_optimization_t;

typedef enum {
  SX1278_SF_6 = 0b01100000,
  SX1278_SF_7 = 0b01110000,
  SX1278_SF_8 = 0b10000000,
  SX1278_SF_9 = 0b10010000,
  SX1278_SF_10 = 0b10100000,
  SX1278_SF_11 = 0b10110000,
  SX1278_SF_12 = 0b11000000
} sx1278_sf_t;

typedef enum {
  SX1278_RX_PAYLOAD_CRC_ON = 0b00000100,
  SX1278_RX_PAYLOAD_CRC_OFF = 0b00000000
} sx1278_crc_payload_t;

typedef enum {
  SX1278_OCP_ON = 0b0010000,
  SX1278_OCP_OFF = 0b00000000
} sx1278_ocp_t;

typedef struct {
  uint8_t length;
  sx1278_crc_payload_t crc;
  sx1278_cr_t coding_rate;
} sx1278_implicit_header_t;

typedef enum {
  SX1278_DIO0_RX_DONE = 0b00000000,
  SX1278_DIO0_TX_DONE = 0b01000000,
  SX1278_DIO0_CAD_DONE = 0b10000000,
  SX1278_DIO1_RXTIMEOUT = 0b00000000,
  SX1278_DIO1_FHSS_CHANGE_CHANNEL = 0b01000000,
  SX1278_DIO1_CAD_DETECTED = 0b10000000,
  SX1278_DIO2_FHSS_CHANGE_CHANNEL = 0b00000000,
  SX1278_DIO3_CAD_DONE = 0b00000000,
  SX1278_DIO3_VALID_HEADER = 0b01000000,
  SX1278_DIO3_PAYLOAD_CRC_ERROR = 0b10000000,
} sx1278_dio_mapping1_t;

typedef enum {
  SX1278_DIO4_CAD_DETECTED  = 0b00000000,
  SX1278_DIO4_PLL_LOCK  = 0b01000000,
  SX1278_DIO5_MODE_READY = 0b00000000,
  SX1278_DIO5_CLK_OUT = 0b01000000
} sx1278_dio_mapping2_t;

typedef enum {
  SX1278_PA_PIN_RFO = 0b00000000,
  SX1278_PA_PIN_BOOST = 0b10000000
} sx1278_pa_pin_t;

typedef struct sx1278_t sx1278;

esp_err_t sx1278_create(spi_host_device_t host, int cs, sx1278 **result);
esp_err_t sx1278_set_opmod(sx1278_mode_t mode, sx1278 *device);
esp_err_t sx1278_set_frequency(uint64_t frequency, sx1278 *device);
esp_err_t sx1278_reset_fifo(sx1278 *device);
esp_err_t sx1278_set_lna_gain(sx1278_gain_t gain, sx1278 *device);
esp_err_t sx1278_set_lna_boost_hf(sx1278_lna_boost_hf_t value, sx1278 *device);
esp_err_t sx1278_set_bandwidth(sx1278_bw_t bandwidth, sx1278 *device);
esp_err_t sx1278_get_bandwidth(sx1278 *device, uint32_t *bandwidth);
esp_err_t sx1278_set_modem_config_2(sx1278_sf_t spreading_factor, sx1278 *device);
esp_err_t sx1278_set_low_datarate_optimization(sx1278_low_datarate_optimization_t value, sx1278 *device);
esp_err_t sx1278_set_syncword(uint8_t value, sx1278 *device);
esp_err_t sx1278_set_preamble_length(uint16_t value, sx1278 *device);
esp_err_t sx1278_set_implicit_header(sx1278_implicit_header_t *header, sx1278 *device);
esp_err_t sx1278_set_dio_mapping1(sx1278_dio_mapping1_t value, sx1278 *device);
esp_err_t sx1278_set_dio_mapping2(sx1278_dio_mapping2_t value, sx1278 *device);
void sx1278_handle_interrupt(void *arg);
esp_err_t sx1278_receive(sx1278 *device, uint8_t **packet, uint8_t *packet_length);
esp_err_t sx1278_get_packet_rssi(sx1278 *device, int16_t *rssi);
esp_err_t sx1278_get_packet_snr(sx1278 *device, float *snr);
esp_err_t sx1278_get_frequency_error(sx1278 *device, int32_t *frequency_error);
esp_err_t sx1278_dump_registers(sx1278 *device);
esp_err_t sx1278_set_pa_config(sx1278_pa_pin_t pin, int power, sx1278 *device);
esp_err_t sx1278_set_ocp(sx1278_ocp_t onoff, uint8_t value, sx1278 *device);

void sx1278_destroy(sx1278 *device);

#ifdef __cplusplus
}
#endif

#endif