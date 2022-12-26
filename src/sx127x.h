#ifndef sx127x_h
#define sx127x_h

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_master.h>
#include <esp_err.h>
#include <stdint.h>

/*
* This structure used to change mode
*/
typedef enum {
  SX127x_MODE_SLEEP = 0b00000000,
  SX127x_MODE_STANDBY = 0b00000001,
  SX127x_MODE_FSTX = 0b00000010,
  SX127x_MODE_TX = 0b00000011,
  SX127x_MODE_FSRX = 0b00000100,
  SX127x_MODE_RX_CONT = 0b00000101,
  SX127x_MODE_RX_SINGLE = 0b00000110,
  SX127x_MODE_CAD = 0b00000111
} sx127x_mode_t;

typedef enum {
  SX127x_LNA_GAIN_G1 = 0b00100000,
  SX127x_LNA_GAIN_G2 = 0b01000000,
  SX127x_LNA_GAIN_G3 = 0b01100000,
  SX127x_LNA_GAIN_G4 = 0b10000000,
  SX127x_LNA_GAIN_G5 = 0b10100000,
  SX127x_LNA_GAIN_G6 = 0b11000000,
  SX127x_LNA_GAIN_AUTO = 0b00000000
} sx127x_gain_t;

typedef enum {
  SX127x_LNA_BOOST_HF_ON = 0b00000011,
  SX127x_LNA_BOOST_HF_OFF = 0b00000000
} sx127x_lna_boost_hf_t;

typedef enum {
  SX127x_BW_7800 = 0b00000000,
  SX127x_BW_10400 = 0b00010000,
  SX127x_BW_15600 = 0b00100000,
  SX127x_BW_20800 = 0b00110000,
  SX127x_BW_31250 = 0b01000000,
  SX127x_BW_41700 = 0b01010000,
  SX127x_BW_62500 = 0b01100000,
  SX127x_BW_125000 = 0b01110000,
  SX127x_BW_250000 = 0b10000000,
  SX127x_BW_500000 = 0b10010000
} sx127x_bw_t;

typedef enum {
  SX127x_CR_4_5 = 0b00000010,
  SX127x_CR_4_6 = 0b00000100,
  SX127x_CR_4_7 = 0b00000110,
  SX127x_CR_4_8 = 0b00001000
} sx127x_cr_t;

typedef enum {
  SX127x_LOW_DATARATE_OPTIMIZATION_ON = 0b00001000,
  SX127x_LOW_DATARATE_OPTIMIZATION_OFF = 0b00000000
} sx127x_low_datarate_optimization_t;

typedef enum {
  SX127x_SF_6 = 0b01100000,
  SX127x_SF_7 = 0b01110000,
  SX127x_SF_8 = 0b10000000,
  SX127x_SF_9 = 0b10010000,
  SX127x_SF_10 = 0b10100000,
  SX127x_SF_11 = 0b10110000,
  SX127x_SF_12 = 0b11000000
} sx127x_sf_t;

typedef enum {
  SX127x_RX_PAYLOAD_CRC_ON = 0b00000100,
  SX127x_RX_PAYLOAD_CRC_OFF = 0b00000000
} sx127x_crc_payload_t;

typedef enum {
  SX127x_OCP_ON = 0b0010000,
  SX127x_OCP_OFF = 0b00000000
} sx127x_ocp_t;

typedef struct {
  uint8_t length;
  sx127x_crc_payload_t crc;
  sx127x_cr_t coding_rate;
} sx127x_implicit_header_t;

typedef struct {
  sx127x_crc_payload_t crc;
  sx127x_cr_t coding_rate;
} sx127x_tx_header_t;

typedef enum {
  SX127x_DIO0_RX_DONE = 0b00000000,
  SX127x_DIO0_TX_DONE = 0b01000000,
  SX127x_DIO0_CAD_DONE = 0b10000000,
  SX127x_DIO1_RXTIMEOUT = 0b00000000,
  SX127x_DIO1_FHSS_CHANGE_CHANNEL = 0b01000000,
  SX127x_DIO1_CAD_DETECTED = 0b10000000,
  SX127x_DIO2_FHSS_CHANGE_CHANNEL = 0b00000000,
  SX127x_DIO3_CAD_DONE = 0b00000000,
  SX127x_DIO3_VALID_HEADER = 0b01000000,
  SX127x_DIO3_PAYLOAD_CRC_ERROR = 0b10000000,
} sx127x_dio_mapping1_t;

typedef enum {
  SX127x_DIO4_CAD_DETECTED = 0b00000000,
  SX127x_DIO4_PLL_LOCK = 0b01000000,
  SX127x_DIO5_MODE_READY = 0b00000000,
  SX127x_DIO5_CLK_OUT = 0b01000000
} sx127x_dio_mapping2_t;

typedef enum {
  SX127x_PA_PIN_RFO = 0b00000000,
  SX127x_PA_PIN_BOOST = 0b10000000
} sx127x_pa_pin_t;

typedef struct sx127x_t sx127x;

esp_err_t sx127x_create(spi_host_device_t host, int cs, sx127x **result);
esp_err_t sx127x_set_opmod(sx127x_mode_t mode, sx127x *device);
esp_err_t sx127x_set_frequency(uint64_t frequency, sx127x *device);
esp_err_t sx127x_reset_fifo(sx127x *device);
esp_err_t sx127x_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device);
esp_err_t sx127x_get_bandwidth(sx127x *device, uint32_t *bandwidth);
esp_err_t sx127x_set_modem_config_2(sx127x_sf_t spreading_factor, sx127x *device);
esp_err_t sx127x_set_low_datarate_optimization(sx127x_low_datarate_optimization_t value, sx127x *device);
esp_err_t sx127x_set_syncword(uint8_t value, sx127x *device);
esp_err_t sx127x_set_preamble_length(uint16_t value, sx127x *device);
esp_err_t sx127x_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device);
esp_err_t sx127x_set_dio_mapping1(sx127x_dio_mapping1_t value, sx127x *device);
esp_err_t sx127x_set_dio_mapping2(sx127x_dio_mapping2_t value, sx127x *device);
esp_err_t sx127x_dump_registers(sx127x *device);
void sx127x_handle_interrupt_fromisr(void *arg);

// RX-related functions
esp_err_t sx127x_set_lna_gain(sx127x_gain_t gain, sx127x *device);
esp_err_t sx127x_set_lna_boost_hf(sx127x_lna_boost_hf_t value, sx127x *device);
void sx127x_set_rx_callback(void (*rx_callback)(sx127x *), sx127x *device);
esp_err_t sx127x_receive(sx127x *device, uint8_t **packet, uint8_t *packet_length);
esp_err_t sx127x_get_packet_rssi(sx127x *device, int16_t *rssi);
esp_err_t sx127x_get_packet_snr(sx127x *device, float *snr);
esp_err_t sx127x_get_frequency_error(sx127x *device, int32_t *frequency_error);

// TX-related functions
esp_err_t sx127x_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device);
esp_err_t sx127x_set_ocp(sx127x_ocp_t onoff, uint8_t milliamps, sx127x *device);
esp_err_t sx127x_set_tx_explcit_header(sx127x_tx_header_t *header, sx127x *device);
void sx127x_set_tx_callback(void (*tx_callback)(sx127x *), sx127x *device);
esp_err_t sx127x_set_for_transmission(uint8_t *data, uint8_t data_length, sx127x *device);

void sx127x_destroy(sx127x *device);

#ifdef __cplusplus
}
#endif
#endif