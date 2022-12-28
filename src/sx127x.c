#include "sx127x.h"

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <stdint.h>
#include <stdlib.h>

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0b
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d

#define SX127x_VERSION 0x12

#define SX127x_LORA_MODE_FSK 0b00000000
#define SX127x_LORA_MODE_LORA 0b10000000

#define SX127x_OSCILLATOR_FREQUENCY 32000000
#define SX127x_FREQ_ERROR_FACTOR ((1 << 24) / SX127x_OSCILLATOR_FREQUENCY)
#define SX127x_REG_MODEM_CONFIG_3_AGC_ON 0b00000100
#define SX127x_REG_MODEM_CONFIG_3_AGC_OFF 0b00000000

#define SX127x_IRQ_FLAG_RXTIMEOUT 0b10000000
#define SX127x_IRQ_FLAG_RXDONE 0b01000000
#define SX127x_IRQ_FLAG_PAYLOAD_CRC_ERROR 0b00100000
#define SX127x_IRQ_FLAG_VALID_HEADER 0b00010000
#define SX127x_IRQ_FLAG_TXDONE 0b00001000
#define SX127x_IRQ_FLAG_CADDONE 0b00000100
#define SX127x_IRQ_FLAG_FHSSCHANGECHANNEL 0b00000010
#define SX127x_IRQ_FLAG_CAD_DETECTED 0b00000001

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define SX127x_MAX_POWER 0b01110000
#define SX127x_LOW_POWER 0b00000000

#define SX127x_HIGH_POWER_ON 0b10000111
#define SX127x_HIGH_POWER_OFF 0b10000100

#define FIFO_TX_BASE_ADDR 0b00000000
#define FIFO_RX_BASE_ADDR 0b00000000

typedef enum {
  SX127x_HEADER_MODE_EXPLICIT = 0b00000000,
  SX127x_HEADER_MODE_IMPLICIT = 0b00000001
} sx127x_header_mode_t;

struct sx127x_t {
  spi_device_handle_t spi;
  sx127x_implicit_header_t *header;
  uint8_t version;
  uint64_t frequency;
  void (*rx_callback)(sx127x *);
  void (*tx_callback)(sx127x *);
  uint8_t packet[256];
};

esp_err_t sx127x_read_registers(int reg, sx127x *device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return ESP_ERR_INVALID_ARG;
  }
  *result = 0;
  spi_transaction_t t = {
      .addr = reg & 0x7F,
      .rx_buffer = NULL,
      .tx_buffer = NULL,
      .rxlength = data_length * 8,
      .length = data_length * 8,
      .flags = SPI_TRANS_USE_RXDATA};
  esp_err_t code = spi_device_polling_transmit(device->spi, &t);
  if (code != ESP_OK) {
    return code;
  }
  for (int i = 0; i < data_length; i++) {
    *result = ((*result) << 8);
    *result = (*result) + t.rx_data[i];
  }
  return ESP_OK;
}

esp_err_t sx127x_read_register(int reg, sx127x *device, uint8_t *result) {
  uint32_t value;
  esp_err_t code = sx127x_read_registers(reg, device, 1, &value);
  if (code != ESP_OK) {
    return code;
  }
  *result = (uint8_t)value;
  return ESP_OK;
}

esp_err_t sx127x_write_register(int reg, uint8_t *data, size_t data_length, sx127x *device) {
  if (data_length == 0 || data_length > 4) {
    return ESP_ERR_INVALID_ARG;
  }
  spi_transaction_t t = {
      .addr = reg | 0x80,
      .rx_buffer = NULL,
      .tx_buffer = NULL,
      .rxlength = data_length * 8,
      .length = data_length * 8,
      .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA};
  for (int i = 0; i < data_length; i++) {
    t.tx_data[i] = data[i];
  }
  return spi_device_polling_transmit(device->spi, &t);
}

esp_err_t sx127x_append_register(int reg, uint8_t value, uint8_t mask, sx127x *device) {
  uint8_t previous = 0;
  esp_err_t code = sx127x_read_register(reg, device, &previous);
  if (code != ESP_OK) {
    return code;
  }
  uint8_t data[] = {(previous & mask) | value};
  return sx127x_write_register(reg, data, 1, device);
}

esp_err_t sx127x_set_low_datarate_optimization(sx127x_low_datarate_optimization_t value, sx127x *device) {
  return sx127x_append_register(REG_MODEM_CONFIG_3, value, 0b11110111, device);
}

esp_err_t sx127x_get_bandwidth(sx127x *device, uint32_t *bandwidth) {
  uint8_t config = 0;
  esp_err_t code = sx127x_read_register(REG_MODEM_CONFIG_1, device, &config);
  if (code != ESP_OK) {
    return code;
  }
  config = (config >> 4);
  switch (config) {
    case 0b0000:
      *bandwidth = 7800;
      break;
    case 0b0001:
      *bandwidth = 10400;
      break;
    case 0b0010:
      *bandwidth = 15600;
      break;
    case 0b0011:
      *bandwidth = 20800;
      break;
    case 0b0100:
      *bandwidth = 31250;
      break;
    case 0b0101:
      *bandwidth = 41700;
      break;
    case 0b0110:
      *bandwidth = 62500;
      break;
    case 0b0111:
      *bandwidth = 125000;
      break;
    case 0b1000:
      *bandwidth = 250000;
      break;
    case 0b1001:
      *bandwidth = 500000;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

esp_err_t sx127x_reload_low_datarate_optimization(sx127x *device) {
  uint32_t bandwidth;
  esp_err_t code = sx127x_get_bandwidth(device, &bandwidth);
  if (code != ESP_OK) {
    return code;
  }
  uint8_t config = 0;
  code = sx127x_read_register(REG_MODEM_CONFIG_2, device, &config);
  if (code != ESP_OK) {
    return code;
  }
  config = (config >> 4);

  // Section 4.1.1.5
  long symbol_duration = 1000 / (bandwidth / (1L << config));
  if (symbol_duration > 16) {
    // force low data rate optimization
    return sx127x_set_low_datarate_optimization(SX127x_LOW_DATARATE_OPTIMIZATION_ON, device);
  }
  return ESP_OK;
}

esp_err_t sx127x_create(spi_host_device_t host, int cs, sx127x **result) {
  struct sx127x_t *device = malloc(sizeof(struct sx127x_t));
  if (device == NULL) {
    return ESP_ERR_NO_MEM;
  }
  *device = (struct sx127x_t){0};
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 8E6,
      .spics_io_num = cs,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  esp_err_t code = spi_bus_add_device(host, &dev_cfg, &device->spi);
  if (code != ESP_OK) {
    sx127x_destroy(device);
    return code;
  }

  code = sx127x_read_register(REG_VERSION, device, &device->version);
  if (code != ESP_OK) {
    sx127x_destroy(device);
    return code;
  }
  if (device->version != SX127x_VERSION) {
    sx127x_destroy(device);
    return ESP_ERR_INVALID_VERSION;
  }
  *result = device;
  return ESP_OK;
}

esp_err_t sx127x_set_opmod(sx127x_mode_t opmod, sx127x *device) {
  uint8_t data[] = {0};
  // enforce DIO mappings for during RX and TX
  if (opmod == SX127x_MODE_RX_CONT || opmod == SX127x_MODE_RX_SINGLE) {
    esp_err_t code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_DIO0_RX_DONE, 0b00111111, device);
    if (code != ESP_OK) {
      return code;
    }
  } else if (opmod == SX127x_MODE_TX) {
    esp_err_t code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_DIO0_TX_DONE, 0b00111111, device);
    if (code != ESP_OK) {
      return code;
    }
  }
  data[0] = (opmod | SX127x_LORA_MODE_LORA);
  return sx127x_write_register(REG_OP_MODE, data, 1, device);
}

esp_err_t sx127x_set_frequency(uint64_t frequency, sx127x *device) {
  uint64_t adjusted = (frequency << 19) / SX127x_OSCILLATOR_FREQUENCY;
  uint8_t data[] = {(uint8_t)(adjusted >> 16), (uint8_t)(adjusted >> 8), (uint8_t)(adjusted >> 0)};
  esp_err_t result = sx127x_write_register(REG_FRF_MSB, data, 3, device);
  if (result != ESP_OK) {
    return result;
  }
  device->frequency = frequency;
  return ESP_OK;
}

esp_err_t sx127x_reset_fifo(sx127x *device) {
  // reset both RX and TX
  uint8_t data[] = {FIFO_TX_BASE_ADDR, FIFO_RX_BASE_ADDR};
  return sx127x_write_register(REG_FIFO_TX_BASE_ADDR, data, 2, device);
}

esp_err_t sx127x_set_lna_gain(sx127x_gain_t gain, sx127x *device) {
  if (gain == SX127x_LNA_GAIN_AUTO) {
    return sx127x_append_register(REG_MODEM_CONFIG_3, SX127x_REG_MODEM_CONFIG_3_AGC_ON, 0b11111011, device);
  }
  esp_err_t code = sx127x_append_register(REG_MODEM_CONFIG_3, SX127x_REG_MODEM_CONFIG_3_AGC_OFF, 0b11111011, device);
  if (code != ESP_OK) {
    return code;
  }
  return sx127x_append_register(REG_LNA, gain, 0b00011111, device);
}

esp_err_t sx127x_set_lna_boost_hf(sx127x_lna_boost_hf_t value, sx127x *device) {
  return sx127x_append_register(REG_LNA, value, 0b11111100, device);
}

esp_err_t sx127x_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device) {
  esp_err_t code = sx127x_append_register(REG_MODEM_CONFIG_1, bandwidth, 0b00001111, device);
  if (code != ESP_OK) {
    return code;
  }
  return sx127x_reload_low_datarate_optimization(device);
}

esp_err_t sx127x_set_modem_config_2(sx127x_sf_t spreading_factor, sx127x *device) {
  if (spreading_factor == SX127x_SF_6 && device->header == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t detection_optimize;
  uint8_t detection_threshold;
  if (spreading_factor == SX127x_SF_6) {
    detection_optimize = 0xc5;
    detection_threshold = 0x0c;
    // make header implicit
  } else {
    detection_optimize = 0xc3;
    detection_threshold = 0x0a;
  }
  uint8_t data[] = {detection_optimize};
  esp_err_t code = sx127x_write_register(REG_DETECTION_OPTIMIZE, data, 1, device);
  if (code != ESP_OK) {
    return code;
  }
  data[0] = detection_threshold;
  code = sx127x_write_register(REG_DETECTION_THRESHOLD, data, 1, device);
  if (code != ESP_OK) {
    return code;
  }
  code = sx127x_append_register(REG_MODEM_CONFIG_2, spreading_factor, 0b00001111, device);
  if (code != ESP_OK) {
    return code;
  }
  return sx127x_reload_low_datarate_optimization(device);
}

void sx127x_set_rx_callback(void (*rx_callback)(sx127x *), sx127x *device) {
  device->rx_callback = rx_callback;
}

esp_err_t sx127x_set_syncword(uint8_t value, sx127x *device) {
  uint8_t data[] = {value};
  return sx127x_write_register(REG_SYNC_WORD, data, 1, device);
}

esp_err_t sx127x_set_preamble_length(uint16_t value, sx127x *device) {
  uint8_t data[] = {(uint8_t)(value >> 8), (uint8_t)(value >> 0)};
  return sx127x_write_register(REG_PREAMBLE_MSB, data, 2, device);
}

esp_err_t sx127x_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device) {
  device->header = header;
  if (header == NULL) {
    return sx127x_append_register(REG_MODEM_CONFIG_1, SX127x_HEADER_MODE_EXPLICIT, 0b11111110, device);
  } else {
    esp_err_t code = sx127x_append_register(REG_MODEM_CONFIG_1, SX127x_HEADER_MODE_IMPLICIT | device->header->coding_rate, 0b11110000, device);
    if (code != ESP_OK) {
      return code;
    }
    return sx127x_append_register(REG_MODEM_CONFIG_2, header->crc, 0b11111011, device);
  }
}

void sx127x_handle_interrupt(void *arg, uint32_t arg2) {
  sx127x *device = (sx127x *)arg;
  uint8_t value;
  esp_err_t code = sx127x_read_register(REG_IRQ_FLAGS, device, &value);
  if (code != ESP_OK) {
    return;
  }
  // clear the irq
  uint8_t data[] = {value};
  code = sx127x_write_register(REG_IRQ_FLAGS, data, 1, device);
  if (code != ESP_OK) {
    return;
  }
  if ((value & SX127x_IRQ_FLAG_PAYLOAD_CRC_ERROR) != 0) {
    return;
  }
  if ((value & SX127x_IRQ_FLAG_RXDONE) != 0) {
    if (device->rx_callback != NULL) {
      device->rx_callback(device);
    }
    return;
  }
  if ((value & SX127x_IRQ_FLAG_TXDONE) != 0) {
    if (device->tx_callback != NULL) {
      device->tx_callback(device);
    }
    return;
  }
}

void IRAM_ATTR sx127x_handle_interrupt_fromisr(void *arg) {
  xTimerPendFunctionCallFromISR(sx127x_handle_interrupt, arg, 0, pdFALSE);
}

esp_err_t sx127x_read_payload(sx127x *device, uint8_t **packet, uint8_t *packet_length) {
  uint8_t length;
  if (device->header == NULL) {
    esp_err_t code = sx127x_read_register(REG_RX_NB_BYTES, device, &length);
    if (code != ESP_OK) {
      *packet_length = 0;
      *packet = NULL;
      return code;
    }
  } else {
    length = device->header->length;
  }

  uint8_t current;
  esp_err_t code = sx127x_read_register(REG_FIFO_RX_CURRENT_ADDR, device, &current);
  if (code != ESP_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }
  uint8_t data[] = {current};
  code = sx127x_write_register(REG_FIFO_ADDR_PTR, data, 1, device);
  if (code != ESP_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }

  spi_transaction_t t = {
      .addr = REG_FIFO & 0x7F,
      .rx_buffer = device->packet,
      .tx_buffer = NULL,
      .rxlength = length * 8,
      .length = length * 8};
  code = spi_device_polling_transmit(device->spi, &t);
  if (code != ESP_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }

  *packet = device->packet;
  *packet_length = length;
  return ESP_OK;
}

esp_err_t sx127x_get_packet_rssi(sx127x *device, int16_t *rssi) {
  uint8_t value;
  esp_err_t code = sx127x_read_register(REG_PKT_RSSI_VALUE, device, &value);
  if (code != ESP_OK) {
    return code;
  }
  if (device->frequency < RF_MID_BAND_THRESHOLD) {
    *rssi = value - RSSI_OFFSET_LF_PORT;
  } else {
    *rssi = value - RSSI_OFFSET_HF_PORT;
  }
  // section 5.5.5.
  float snr;
  code = sx127x_get_packet_snr(device, &snr);
  // if snr failed then rssi is not precise
  if (code == ESP_OK && snr < 0) {
    *rssi = *rssi + snr;
  }
  return ESP_OK;
}

esp_err_t sx127x_get_packet_snr(sx127x *device, float *snr) {
  uint8_t value;
  esp_err_t code = sx127x_read_register(REG_PKT_SNR_VALUE, device, &value);
  if (code != ESP_OK) {
    return code;
  }
  *snr = ((int8_t)value) * 0.25;
  return ESP_OK;
}

esp_err_t sx127x_get_frequency_error(sx127x *device, int32_t *result) {
  uint32_t frequency_error;
  esp_err_t code = sx127x_read_registers(REG_FREQ_ERROR_MSB, device, 3, &frequency_error);
  if (code != ESP_OK) {
    return code;
  }
  uint32_t bandwidth;
  code = sx127x_get_bandwidth(device, &bandwidth);
  if (code != ESP_OK) {
    return code;
  }
  if (frequency_error & 0x80000) {
    frequency_error = ~frequency_error + 1;
    *result = -1;
  } else {
    *result = 1;
  }
  *result = (*result) * frequency_error * SX127x_FREQ_ERROR_FACTOR * bandwidth / 500000.0f;
  return ESP_OK;
}

esp_err_t sx127x_dump_registers(sx127x *device) {
  uint8_t length = 0x7F;
  for (int i = 0; i < length; i++) {
    uint8_t value;
    sx127x_read_register(i, device, &value);
    printf("0x%.2x: 0x%.2x\n", i, value);
  }
  return ESP_OK;
}

esp_err_t sx127x_set_dio_mapping1(sx127x_dio_mapping1_t value, sx127x *device) {
  uint8_t data[] = {value};
  return sx127x_write_register(REG_DIO_MAPPING_1, data, 1, device);
}

esp_err_t sx127x_set_dio_mapping2(sx127x_dio_mapping2_t value, sx127x *device) {
  uint8_t data[] = {value};
  return sx127x_write_register(REG_DIO_MAPPING_2, data, 1, device);
}

void sx127x_set_tx_callback(void (*tx_callback)(sx127x *), sx127x *device) {
  device->tx_callback = tx_callback;
}

esp_err_t sx127x_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device) {
  if (pin == SX127x_PA_PIN_RFO && (power < -4 || power > 15)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (pin == SX127x_PA_PIN_BOOST && (power < 2 || power > 20)) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t data[] = {0};
  if (pin == SX127x_PA_PIN_BOOST && power == 20) {
    data[0] = SX127x_HIGH_POWER_ON;
  } else {
    data[0] = SX127x_HIGH_POWER_OFF;
  }
  esp_err_t code = sx127x_write_register(REG_PA_DAC, data, 1, device);
  if (code != ESP_OK) {
    return code;
  }
  uint8_t max_current;
  // according to 2.5.1. Power Consumption
  if (pin == SX127x_PA_PIN_BOOST) {
    if (power == 20) {
      max_current = 120;
    } else {
      max_current = 87;
    }
  } else {
    if (power > 7) {
      max_current = 29;
    } else {
      max_current = 20;
    }
  }
  code = sx127x_set_ocp(SX127x_OCP_ON, max_current, device);
  if (code != ESP_OK) {
    return code;
  }
  uint8_t value;
  if (pin == SX127x_PA_PIN_RFO) {
    if (power < 0) {
      value = SX127x_LOW_POWER | (power + 4);
    } else {
      value = SX127x_MAX_POWER | power;
    }
    value = value | SX127x_PA_PIN_RFO;
  } else {
    if (power == 20) {
      value = SX127x_PA_PIN_BOOST | 0b00001111;
    } else {
      value = SX127x_PA_PIN_BOOST | (power - 2);
    }
  }
  data[0] = value;
  return sx127x_write_register(REG_PA_CONFIG, data, 1, device);
}

esp_err_t sx127x_set_ocp(sx127x_ocp_t onoff, uint8_t max_current, sx127x *device) {
  uint8_t data[1];
  if (onoff == SX127x_OCP_OFF) {
    data[0] = SX127x_OCP_OFF;
    return sx127x_write_register(REG_OCP, data, 1, device);
  }
  // 5.4.4. Over Current Protection
  if (max_current <= 120) {
    data[0] = (max_current - 45) / 5;
  } else if (max_current <= 240) {
    data[0] = (max_current + 30) / 10;
  } else {
    data[0] = 27;
  }
  data[0] = (data[0] | onoff);
  return sx127x_write_register(REG_OCP, data, 1, device);
}

esp_err_t sx127x_set_tx_explcit_header(sx127x_tx_header_t *header, sx127x *device) {
  if (header == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t code = sx127x_append_register(REG_MODEM_CONFIG_1, header->coding_rate | SX127x_HEADER_MODE_EXPLICIT, 0b11110000, device);
  if (code != ESP_OK) {
    return code;
  }
  return sx127x_append_register(REG_MODEM_CONFIG_2, header->crc, 0b11111011, device);
}

esp_err_t sx127x_set_for_transmission(uint8_t *data, uint8_t data_length, sx127x *device) {
  // uint8_t can't be more than MAX_PACKET_SIZE
  if (data_length == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t fifo_addr[] = {FIFO_TX_BASE_ADDR};
  esp_err_t code = sx127x_write_register(REG_FIFO_ADDR_PTR, fifo_addr, 1, device);
  if (code != ESP_OK) {
    return code;
  }
  uint8_t reg_data[] = {data_length};
  code = sx127x_write_register(REG_PAYLOAD_LENGTH, reg_data, 1, device);
  if (code != ESP_OK) {
    return code;
  }

  spi_transaction_t t = {
      .addr = REG_FIFO | 0x80,
      .rx_buffer = NULL,
      .tx_buffer = data,
      .rxlength = data_length * 8,
      .length = data_length * 8};
  return spi_device_polling_transmit(device->spi, &t);
}

void sx127x_destroy(sx127x *device) {
  if (device == NULL) {
    return;
  }
  if (device->spi != NULL) {
    spi_bus_remove_device(device->spi);
  }
  free(device);
}