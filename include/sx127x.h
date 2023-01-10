#ifndef sx127x_h
#define sx127x_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MAX_PACKET_SIZE 256
#define SX127X_OK 0                      /*!< esp_err_t value indicating success (no error) */
#define SX127X_ERR_NO_MEM 0x101          /*!< Out of memory */
#define SX127X_ERR_INVALID_ARG 0x102     /*!< Invalid argument */
#define SX127X_ERR_NOT_FOUND 0x105       /*!< Requested resource not found */
#define SX127X_ERR_INVALID_VERSION 0x10A /*!< Version was invalid */

/*
 * This structure used to change mode
 */
typedef enum {
  SX127x_MODE_SLEEP = 0b00000000,      // SLEEP
  SX127x_MODE_STANDBY = 0b00000001,    // STDBY
  SX127x_MODE_FSTX = 0b00000010,       // Frequency synthesis TX
  SX127x_MODE_TX = 0b00000011,         // Transmit
  SX127x_MODE_FSRX = 0b00000100,       // Frequency synthesis RX
  SX127x_MODE_RX_CONT = 0b00000101,    // Receive continuous
  SX127x_MODE_RX_SINGLE = 0b00000110,  // Receive single
  SX127x_MODE_CAD = 0b00000111         // Channel activity detection
} sx127x_mode_t;

typedef enum {
  SX127x_LNA_GAIN_G1 = 0b00100000,  // Maximum gain
  SX127x_LNA_GAIN_G2 = 0b01000000,
  SX127x_LNA_GAIN_G3 = 0b01100000,
  SX127x_LNA_GAIN_G4 = 0b10000000,
  SX127x_LNA_GAIN_G5 = 0b10100000,
  SX127x_LNA_GAIN_G6 = 0b11000000,   // Minimum gain
  SX127x_LNA_GAIN_AUTO = 0b00000000  // Automatic. See 5.5.3. for details
} sx127x_gain_t;

/**
 * @brief High Frequency (RFI_HF) LNA current adjustment
 *
 */
typedef enum {
  SX127x_LNA_BOOST_HF_ON = 0b00000011,  // Default LNA current
  SX127x_LNA_BOOST_HF_OFF = 0b00000000  // Boost on, 150% LNA current
} sx127x_lna_boost_hf_t;

/**
 * @brief Signal bandwidth.
 *
 * @note In the lower band (169MHz), signal bandwidths 8&9 (250k and 500k) are not supported
 *
 */
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

/**
 * @brief SF rate (expressed as a base-2 logarithm)
 *
 */
typedef enum {
  SX127x_SF_6 = 0b01100000,   // 64 chips / symbol
  SX127x_SF_7 = 0b01110000,   // 128 chips / symbol
  SX127x_SF_8 = 0b10000000,   // 256 chips / symbol
  SX127x_SF_9 = 0b10010000,   // 512 chips / symbol
  SX127x_SF_10 = 0b10100000,  // 1024 chips / symbol
  SX127x_SF_11 = 0b10110000,  // 2048 chips / symbol
  SX127x_SF_12 = 0b11000000   // 4096 chips / symbol
} sx127x_sf_t;

typedef enum {
  SX127x_RX_PAYLOAD_CRC_ON = 0b00000100,
  SX127x_RX_PAYLOAD_CRC_OFF = 0b00000000
} sx127x_crc_payload_t;

/**
 * @brief Enable or disable overload current protection (OCP) for power amplifier.
 *
 */
typedef enum {
  SX127x_OCP_ON = 0b0010000,
  SX127x_OCP_OFF = 0b00000000
} sx127x_ocp_t;

/**
 * @brief Imlicit header for TX or RX.
 *
 */
typedef struct {
  uint8_t length;            // payload length. Cannot be more than 256 bytes.
  sx127x_crc_payload_t crc;  // Enable or disable CRC.
  sx127x_cr_t coding_rate;   // Coding rate
} sx127x_implicit_header_t;

typedef struct {
  sx127x_crc_payload_t crc;
  sx127x_cr_t coding_rate;
} sx127x_tx_header_t;

/**
 * @brief Type of interrupt. Same interrupts can happen on different digital pins.
 *
 */
typedef enum {
  SX127x_DIO0_RX_DONE = 0b00000000,              // Packet reception complete
  SX127x_DIO0_TX_DONE = 0b01000000,              // FIFO Payload transmission complete
  SX127x_DIO0_CAD_DONE = 0b10000000,             // CAD complete
  SX127x_DIO1_RXTIMEOUT = 0b00000000,            // RX timeout interrupt. Used in RX single mode
  SX127x_DIO1_FHSS_CHANGE_CHANNEL = 0b01000000,  // FHSS change channel
  SX127x_DIO1_CAD_DETECTED = 0b10000000,         // Valid Lora signal detected during CAD operation
  SX127x_DIO2_FHSS_CHANGE_CHANNEL = 0b00000000,  // FHSS change channel on digital pin 2
  SX127x_DIO3_CAD_DONE = 0b00000000,             // CAD complete on digital pin 3
  SX127x_DIO3_VALID_HEADER = 0b01000000,         // Valid header received in Rx
  SX127x_DIO3_PAYLOAD_CRC_ERROR = 0b10000000,    // Payload CRC error
} sx127x_dio_mapping1_t;

/**
 * @brief Type of interrupt. Same interrupts can happen on different digital pins.
 *
 */
typedef enum {
  SX127x_DIO4_CAD_DETECTED = 0b00000000,  // Valid Lora signal detected during CAD operation
  SX127x_DIO4_PLL_LOCK = 0b01000000,      // PLL lock
  SX127x_DIO5_MODE_READY = 0b00000000,    // Mode ready
  SX127x_DIO5_CLK_OUT = 0b01000000        // clock out
} sx127x_dio_mapping2_t;

/**
 * @brief sx127x chip has 2 pins for TX. Based on the pin below chip can produce different max power.
 *
 */
typedef enum {
  SX127x_PA_PIN_RFO = 0b00000000,   // RFO pin. Output power is limited to +14 dBm.
  SX127x_PA_PIN_BOOST = 0b10000000  // PA_BOOST pin. Output power is limited to +20 dBm
} sx127x_pa_pin_t;

/**
 * @brief Device handle
 *
 */
typedef struct sx127x_t sx127x;

/**
 * @brief Create device handle and attach to SPI bus.
 *
 * @param spi_device spi device
 * @param result Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG      if parameter is invalid
 *         - SX127X_ERR_NOT_FOUND        if host doesn't have any free CS slots
 *         - SX127X_ERR_NO_MEM           if no not enough memory
 *         - SX127X_ERR_INVALID_VERSION  if device attached to SPI bus is invalid or chip is not sx127x.
 *         - SX127X_OK                   on success
 */
int sx127x_create(void *spi_device, sx127x **result);

/**
 * @brief Set operating mode.
 *
 * @param mode Sleep, standby, rx or tx. See @ref sx127x_mode_t for details.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_opmod(sx127x_mode_t mode, sx127x *device);

/**
 * @brief Set frequency for RX or TX.
 *
 * @param frequency Frequency in hz.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_frequency(uint64_t frequency, sx127x *device);

/**
 * @brief Reset chip's memory pointers to 0. Both RX and TX.
 *
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_reset_fifo(sx127x *device);

/**
 * @brief Set signal bandwidth
 *
 * Enable low datarate optimization if the symbol length exceeds 16ms.
 *
 * @note In the lower band (169MHz), signal bandwidths 8&9 (250k and 500k) are not supported.
 * @param bandwidth Selected bandwidth
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device);

/**
 * @brief Get signal bandwidth in hz.
 *
 * @param device Pointer to variable to hold the device handle
 * @param bandwidth Signal bandwidth in hz. For example: 125000.
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_get_bandwidth(sx127x *device, uint32_t *bandwidth);

/**
 * @brief Set speading factor (SF rate). See section 4.1.1.2. for more details.
 *
 * Enable low datarate optimization if the symbol length exceeds 16ms.
 *
 * @param spreading_factor SF rate
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_modem_config_2(sx127x_sf_t spreading_factor, sx127x *device);

/**
 * @brief Enable or disable low datarate optimization.
 *
 * It should have the same value on transmitter and receiver. If unsure, don't change.
 *
 * @param value Enable or disable.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_low_datarate_optimization(sx127x_low_datarate_optimization_t value, sx127x *device);

/**
 * @brief Set syncword.
 *
 * @note Value 0x34 is reserved for LoRaWAN networks.
 * @param value Syncword.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_syncword(uint8_t value, sx127x *device);

/**
 * @brief Set preamble length. See 4.1.1 for more details.
 *
 * @param value Preamble length
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_preamble_length(uint16_t value, sx127x *device);

/**
 * @brief Set implicit header.
 *
 * sx127x can send packets for explicit header or without it (implicit). In implicit mode receiver should be configured with pre-defined values using this function.
 * In explicit mode, all information is sent in the header. Thus no configuration needed.
 *
 * @param header Pre-defined packet information. If NULL, then assume explicit header in RX mode. For TX explicit mode please use sx127x_set_tx_explcit_header function.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device);

/**
 * @brief Map different interrupts from sx127x to different digital pins output.
 *
 * Six general purpose IO pins are available on the SX1276/77/78/79, and their configuration in Continuous or Packet mode is controlled through RegDioMapping1 and RegDioMapping2.
 *
 * @param value Mapping
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_dio_mapping1(sx127x_dio_mapping1_t value, sx127x *device);

/**
 * @brief Map different interrupts from sx127x to different digital pins output.
 *
 * Six general purpose IO pins are available on the SX1276/77/78/79, and their configuration in Continuous or Packet mode is controlled through RegDioMapping1 and RegDioMapping2.
 *
 * @param value Mapping
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_dio_mapping2(sx127x_dio_mapping2_t value, sx127x *device);

int sx127x_dump_registers(sx127x *device);

/**
 * @brief Handle interrupt from DIOx pins.
 *
 * @note This function SHOULD NOT be called from ISR. Use separate ISR-safe function
 *
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_handle_interrupt(sx127x *device);

// RX-related functions
/**
 * @brief Set RX gain. Can be manual or automatic.
 *
 * @param gain G1 is maximum, G6 is minimum.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_lna_gain(sx127x_gain_t gain, sx127x *device);

/**
 * @brief Boost LNA current in high frequency mode (over 525Mhz).
 *
 * @param value Enable or disable
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_lna_boost_hf(sx127x_lna_boost_hf_t value, sx127x *device);

/**
 * @brief Set callback function for rxdone interrupt.
 *
 * @param rx_callback Callback function. Should accept pointer to variable to hold the device handle.
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_set_rx_callback(void (*rx_callback)(sx127x *), sx127x *device);

/**
 * @brief Read payload from sx127x's internal FIFO.
 *
 * @param device Pointer to variable to hold the device handle
 * @param packet Output buffer
 * @param packet_length Output buffer length
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_read_payload(sx127x *device, uint8_t **packet, uint8_t *packet_length);

/**
 * @brief RSSI of the latest packet received (dBm)
 *
 * @param device Pointer to variable to hold the device handle
 * @param rssi RSSI
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_get_packet_rssi(sx127x *device, int16_t *rssi);

/**
 * @brief Estimation of SNR on last packet received
 *
 * @param device Pointer to variable to hold the device handle
 * @param snr SNR
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_get_packet_snr(sx127x *device, float *snr);

/**
 * @brief Read estimated frequency error from modem.
 *
 * @param device Pointer to variable to hold the device handle
 * @param frequency_error Output frequency error
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_get_frequency_error(sx127x *device, int32_t *frequency_error);

// TX-related functions
/**
 * @brief Set output power for transmittion.
 *
 * Set reasonable overload current protection (OCP) based on table 6 from section 2.5.1:
 * 20dbm -> 120mA.
 * 2 ~ 17dbm -> 87mA.
 * 7 ~ 15dbm -> 29dbm.
 * -4 ~ 7dbm -> 20mA.
 *
 * @param pin Which pin to use. sx127x supports: RFO and PA_BOOST.
 * @param power Output power in dbm. When using RFO pin supported range is from -4 to 15dbm. When using PA_BOOST range is from 2 to 17dbm. And also 20dbm for high power output. See section 5.4.3.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device);

/**
 * @brief Configure overload current protection (OCP) for PA.
 *
 * @param onoff Enable or disable OCP
 * @param milliamps Maximum current in milliamps
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_ocp(sx127x_ocp_t onoff, uint8_t milliamps, sx127x *device);

/**
 * @brief Set explicit header during TX.
 *
 * @param header Transmitter will populate packet's header with this configuration
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_tx_explicit_header(sx127x_tx_header_t *header, sx127x *device);

/**
 * @brief Set callback function for txdone interrupt.
 *
 * @param tx_callback Callback function. Should accept pointer to variable to hold the device handle.
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_set_tx_callback(void (*tx_callback)(sx127x *), sx127x *device);

/**
 * @brief Write packet into sx127x's FIFO for transmittion. Once packet is written, set opmod to TX.
 *
 * @param data Packet
 * @param data_length Packet length. Cannot be more than 256 bytes or 0.
 * @param device Pointer to variable to hold the device handle
 * @return
 *         - SX127X_ERR_INVALID_ARG   if parameter is invalid
 *         - SX127X_OK                on success
 */
int sx127x_set_for_transmission(uint8_t *data, uint8_t data_length, sx127x *device);

/**
 * @brief Disconnect from SPI and release any resources assotiated. After calling this function pointer to device will be unusable.
 *
 * @param device Pointer to variable to hold the device handle
 */
void sx127x_destroy(sx127x *device);

#ifdef __cplusplus
}
#endif
#endif