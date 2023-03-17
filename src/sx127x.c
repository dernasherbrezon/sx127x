#include "sx127x.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sx127x_spi.h>

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_BITRATE_MSB 0x02
#define REG_FDEV_MSB 0x04
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_PA_RAMP 0x0a
#define REG_OCP 0x0b
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_RX_CONFIG 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_RSSI_CONFIG 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_RSSI_COLLISION 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_BW 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_AFC_BW 0x13
#define REG_OOK_PEAK 0x14
#define REG_OOK_FIX 0x15
#define REG_OOK_AVG 0x16
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_FEI_MSB 0x1d
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PREAMBLE_MSB_FSK 0x25
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_SYNC_CONFIG 0x27
#define REG_FREQ_ERROR_MSB 0x28
#define REG_SYNC_VALUE1 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_PACKET_CONFIG1 0x30
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_PACKET_CONFIG2 0x31
#define REG_PAYLOAD_LENGTH_FSK 0x32
#define REG_INVERTIQ 0x33
#define REG_NODE_ADDR 0x33
#define REG_BROADCAST_ADDR 0x34
#define REG_FIFO_THRESH 0x35
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_IRQ_FLAGS_1 0x3e
#define REG_IRQ_FLAGS_2 0x3f
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d
#define REG_BITRATE_FRAC 0x5d

#define SX127x_VERSION 0x12

#define SX127x_OSCILLATOR_FREQUENCY 32000000.0f
#define SX127x_FREQ_ERROR_FACTOR ((1 << 24) / SX127x_OSCILLATOR_FREQUENCY)
#define SX127x_FSTEP (SX127x_OSCILLATOR_FREQUENCY / (1 << 19))
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

#define SX127X_FSK_IRQ_FIFO_FULL 0b10000000
#define SX127X_FSK_IRQ_FIFO_EMPTY 0b01000000
#define SX127X_FSK_IRQ_FIFO_LEVEL 0b00100000
#define SX127X_FSK_IRQ_FIFO_OVERRUN 0b00010000
#define SX127X_FSK_IRQ_PACKET_SENT 0b00001000
#define SX127X_FSK_IRQ_PAYLOAD_READY 0b00000100
#define SX127X_FSK_IRQ_CRC_OK 0b00000010
#define SX127X_FSK_IRQ_LOW_BATTERY 0b00000001

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
  void *spi_device;
  sx127x_implicit_header_t *header;
  uint8_t version;
  uint64_t frequency;

  void (*rx_callback)(sx127x *);

  void (*tx_callback)(sx127x *);

  void (*cad_callback)(sx127x *, int);

  uint8_t packet[256];
  sx127x_modulation_t active_modem;
  sx127x_packet_format_t fsk_ook_format;
};

int sx127x_read_register(int reg, void *spi_device, uint8_t *result) {
  uint32_t value;
  int code = sx127x_spi_read_registers(reg, spi_device, 1, &value);
  if (code != SX127X_OK) {
    return code;
  }
  *result = (uint8_t)value;
  return SX127X_OK;
}

int sx127x_append_register(int reg, uint8_t value, uint8_t mask, void *spi_device) {
  uint8_t previous = 0;
  int code = sx127x_read_register(reg, spi_device, &previous);
  if (code != SX127X_OK) {
    return code;
  }
  uint8_t data[] = {(previous & mask) | value};
  return sx127x_spi_write_register(reg, data, 1, spi_device);
}

int sx127x_set_low_datarate_optimization(sx127x_low_datarate_optimization_t value, sx127x *device) {
  return sx127x_append_register(REG_MODEM_CONFIG_3, value, 0b11110111, device->spi_device);
}

int sx127x_get_bandwidth(sx127x *device, uint32_t *bandwidth) {
  uint8_t config = 0;
  int code = sx127x_read_register(REG_MODEM_CONFIG_1, device->spi_device, &config);
  if (code != SX127X_OK) {
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
      return SX127X_ERR_INVALID_ARG;
  }
  return SX127X_OK;
}

int sx127x_reload_low_datarate_optimization(sx127x *device) {
  uint32_t bandwidth;
  int code = sx127x_get_bandwidth(device, &bandwidth);
  if (code != SX127X_OK) {
    return code;
  }
  uint8_t config = 0;
  code = sx127x_read_register(REG_MODEM_CONFIG_2, device->spi_device, &config);
  if (code != SX127X_OK) {
    return code;
  }
  config = (config >> 4);

  // Section 4.1.1.5
  uint32_t symbol_duration = 1000 / (bandwidth / (1L << config));
  if (symbol_duration > 16) {
    // force low data rate optimization
    return sx127x_set_low_datarate_optimization(SX127x_LOW_DATARATE_OPTIMIZATION_ON, device);
  }
  return SX127X_OK;
}

void sx127x_fsk_ook_handle_interrupt(sx127x *device) {
  uint8_t value;
  int code = sx127x_read_register(REG_IRQ_FLAGS_2, device->spi_device, &value);
  if (code != SX127X_OK) {
    return;
  }
  // clear the irq
  code = sx127x_spi_write_register(REG_IRQ_FLAGS_2, &value, 1, device->spi_device);
  if (code != SX127X_OK) {
    return;
  }
  // FIXME handle FIFO LEVEL for long packets
  if ((value & SX127X_FSK_IRQ_CRC_OK) != 0) {
    if (device->rx_callback != NULL) {
      device->rx_callback(device);
    }
    return;
  }
  if ((value & SX127X_FSK_IRQ_PACKET_SENT) != 0) {
    if (device->tx_callback != NULL) {
      device->tx_callback(device);
    }
    return;
  }
}

void sx127x_lora_handle_interrupt(sx127x *device) {
  uint8_t value;
  int code = sx127x_read_register(REG_IRQ_FLAGS, device->spi_device, &value);
  if (code != SX127X_OK) {
    return;
  }
  code = sx127x_spi_write_register(REG_IRQ_FLAGS, &value, 1, device->spi_device);
  if (code != SX127X_OK) {
    return;
  }
  if ((value & SX127x_IRQ_FLAG_CADDONE) != 0) {
    if (device->cad_callback != NULL) {
      device->cad_callback(device, value & SX127x_IRQ_FLAG_CAD_DETECTED);
    }
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

void sx127x_handle_interrupt(sx127x *device) {
  if (device->active_modem == SX127x_MODULATION_LORA) {
    sx127x_lora_handle_interrupt(device);
  } else if (device->active_modem == SX127x_MODULATION_FSK || device->active_modem == SX127x_MODULATION_OOK) {
    sx127x_fsk_ook_handle_interrupt(device);
  }
}

int sx127x_create(void *spi_device, sx127x **result) {
  struct sx127x_t *device = malloc(sizeof(struct sx127x_t));
  if (device == NULL) {
    return SX127X_ERR_NO_MEM;
  }
  *device = (struct sx127x_t){0};
  device->spi_device = spi_device;

  int code = sx127x_read_register(REG_VERSION, device->spi_device, &device->version);
  if (code != SX127X_OK) {
    sx127x_destroy(device);
    return code;
  }
  if (device->version != SX127x_VERSION) {
    sx127x_destroy(device);
    return SX127X_ERR_INVALID_VERSION;
  }
  device->active_modem = SX127x_MODULATION_LORA;
  device->fsk_ook_format = SX127X_VARIABLE;
  *result = device;
  return SX127X_OK;
}

int sx127x_set_opmod(sx127x_mode_t opmod, sx127x_modulation_t modulation, sx127x *device) {
  // enforce DIO mappings for RX and TX
  if (modulation == SX127x_MODULATION_LORA) {
    if (opmod == SX127x_MODE_RX_CONT || opmod == SX127x_MODE_RX_SINGLE) {
      int code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_DIO0_RX_DONE, 0b00111111, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    } else if (opmod == SX127x_MODE_TX) {
      int code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_DIO0_TX_DONE, 0b00111111, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    } else if (opmod == SX127x_MODE_CAD) {
      int code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_DIO0_CAD_DONE, 0b00111111, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    }
  } else if (modulation == SX127x_MODULATION_FSK || modulation == SX127x_MODULATION_OOK) {
    if (opmod == SX127x_MODE_RX_CONT || opmod == SX127x_MODE_RX_SINGLE) {
      int code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_FSK_DIO0_CRC_OK | SX127x_FSK_DIO1_FIFO_LEVEL | SX127x_FSK_DIO2_FIFO_FULL, 0b00000011, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    } else if (opmod == SX127x_MODE_TX) {
      int code = sx127x_append_register(REG_DIO_MAPPING_1, SX127x_FSK_DIO0_PACKET_SENT | SX127x_FSK_DIO1_FIFO_LEVEL | SX127x_FSK_DIO2_FIFO_FULL, 0b00000011, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
      // start tx as soon as first byte in FIFO available
      uint8_t data = 0b10001111;
      code = sx127x_spi_write_register(REG_FIFO_THRESH, &data, 1, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    } else {
      return SX127X_ERR_INVALID_ARG;
    }
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value = (opmod | modulation);
  int result = sx127x_spi_write_register(REG_OP_MODE, &value, 1, device->spi_device);
  if (result == SX127X_OK) {
    device->active_modem = modulation;
  }
  return result;
}

int sx127x_set_frequency(uint64_t frequency, sx127x *device) {
  uint64_t adjusted = (frequency << 19) / SX127x_OSCILLATOR_FREQUENCY;
  uint8_t data[] = {(uint8_t)(adjusted >> 16), (uint8_t)(adjusted >> 8), (uint8_t)(adjusted >> 0)};
  int result = sx127x_spi_write_register(REG_FRF_MSB, data, 3, device->spi_device);
  if (result != SX127X_OK) {
    return result;
  }
  device->frequency = frequency;
  return SX127X_OK;
}

int sx127x_reset_fifo(sx127x *device) {
  // reset both RX and TX
  uint8_t data[] = {FIFO_TX_BASE_ADDR, FIFO_RX_BASE_ADDR};
  return sx127x_spi_write_register(REG_FIFO_TX_BASE_ADDR, data, 2, device->spi_device);
}

int sx127x_set_lna_gain(sx127x_gain_t gain, sx127x *device) {
  if (device->active_modem == SX127x_MODULATION_LORA) {
    if (gain == SX127x_LNA_GAIN_AUTO) {
      return sx127x_append_register(REG_MODEM_CONFIG_3, SX127x_REG_MODEM_CONFIG_3_AGC_ON, 0b11111011, device->spi_device);
    }
    int code = sx127x_append_register(REG_MODEM_CONFIG_3, SX127x_REG_MODEM_CONFIG_3_AGC_OFF, 0b11111011, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    return sx127x_append_register(REG_LNA, gain, 0b00011111, device->spi_device);
  } else if (device->active_modem == SX127x_MODULATION_FSK || device->active_modem == SX127x_MODULATION_OOK) {
    if (gain == SX127x_LNA_GAIN_AUTO) {
      return sx127x_append_register(REG_RX_CONFIG, 0b00001000, 0b11110111, device->spi_device);
    }
    int code = sx127x_append_register(REG_RX_CONFIG, 0b00000000, 0b11110111, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    return sx127x_append_register(REG_LNA, gain, 0b00011111, device->spi_device);
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_set_lna_boost_hf(sx127x_lna_boost_hf_t value, sx127x *device) {
  return sx127x_append_register(REG_LNA, value, 0b11111100, device->spi_device);
}

int sx127x_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device) {
  int code = sx127x_append_register(REG_MODEM_CONFIG_1, bandwidth, 0b00001111, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_reload_low_datarate_optimization(device);
}

int sx127x_set_modem_config_2(sx127x_sf_t spreading_factor, sx127x *device) {
  if (spreading_factor == SX127x_SF_6 && device->header == NULL) {
    return SX127X_ERR_INVALID_ARG;
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
  int code = sx127x_spi_write_register(REG_DETECTION_OPTIMIZE, &detection_optimize, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  code = sx127x_spi_write_register(REG_DETECTION_THRESHOLD, &detection_threshold, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  code = sx127x_append_register(REG_MODEM_CONFIG_2, spreading_factor, 0b00001111, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_reload_low_datarate_optimization(device);
}

void sx127x_set_rx_callback(void (*rx_callback)(sx127x *), sx127x *device) {
  device->rx_callback = rx_callback;
}

int sx127x_set_syncword(uint8_t value, sx127x *device) {
  return sx127x_spi_write_register(REG_SYNC_WORD, &value, 1, device->spi_device);
}

int sx127x_set_preamble_length(uint16_t value, sx127x *device) {
  uint8_t data[] = {(uint8_t)(value >> 8), (uint8_t)(value >> 0)};
  if (device->active_modem == SX127x_MODULATION_LORA) {
    return sx127x_spi_write_register(REG_PREAMBLE_MSB, data, 2, device->spi_device);
  } else if (device->active_modem == SX127x_MODULATION_FSK || device->active_modem == SX127x_MODULATION_OOK) {
    return sx127x_spi_write_register(REG_PREAMBLE_MSB_FSK, data, 2, device->spi_device);
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device) {
  device->header = header;
  if (header == NULL) {
    return sx127x_append_register(REG_MODEM_CONFIG_1, SX127x_HEADER_MODE_EXPLICIT, 0b11111110, device->spi_device);
  } else {
    int code = sx127x_append_register(REG_MODEM_CONFIG_1, SX127x_HEADER_MODE_IMPLICIT | device->header->coding_rate, 0b11110000, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    code = sx127x_spi_write_register(REG_PAYLOAD_LENGTH, &(header->length), 1, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    return sx127x_append_register(REG_MODEM_CONFIG_2, header->crc, 0b11111011, device->spi_device);
  }
}

int sx127x_read_payload(sx127x *device, uint8_t **packet, uint8_t *packet_length) {
  uint8_t length;
  if (device->header == NULL) {
    int code = sx127x_read_register(REG_RX_NB_BYTES, device->spi_device, &length);
    if (code != SX127X_OK) {
      *packet_length = 0;
      *packet = NULL;
      return code;
    }
  } else {
    length = device->header->length;
  }

  uint8_t current;
  int code = sx127x_read_register(REG_FIFO_RX_CURRENT_ADDR, device->spi_device, &current);
  if (code != SX127X_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }
  code = sx127x_spi_write_register(REG_FIFO_ADDR_PTR, &current, 1, device->spi_device);
  if (code != SX127X_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }

  code = sx127x_spi_read_buffer(REG_FIFO, device->packet, length, device->spi_device);
  if (code != SX127X_OK) {
    *packet_length = 0;
    *packet = NULL;
    return code;
  }

  *packet = device->packet;
  *packet_length = length;
  return SX127X_OK;
}

int sx127x_get_packet_rssi(sx127x *device, int16_t *rssi) {
  uint8_t value;
  int code = sx127x_read_register(REG_PKT_RSSI_VALUE, device->spi_device, &value);
  if (code != SX127X_OK) {
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
  if (code == SX127X_OK && snr < 0) {
    *rssi = *rssi + snr;
  }
  return SX127X_OK;
}

int sx127x_get_packet_snr(sx127x *device, float *snr) {
  uint8_t value;
  int code = sx127x_read_register(REG_PKT_SNR_VALUE, device->spi_device, &value);
  if (code != SX127X_OK) {
    return code;
  }
  *snr = ((int8_t)value) * 0.25f;
  return SX127X_OK;
}

int sx127x_get_frequency_error(sx127x *device, int32_t *result) {
  if (device->active_modem == SX127x_MODULATION_LORA) {
    uint32_t frequency_error;
    int code = sx127x_spi_read_registers(REG_FREQ_ERROR_MSB, device->spi_device, 3, &frequency_error);
    if (code != SX127X_OK) {
      return code;
    }
    uint32_t bandwidth;
    code = sx127x_get_bandwidth(device, &bandwidth);
    if (code != SX127X_OK) {
      return code;
    }
    if (frequency_error & 0x80000) {
      // keep within original 2.5 bytes
      frequency_error = ((~frequency_error) + 1) & 0xFFFFF;
      *result = -1;
    } else {
      *result = 1;
    }
    *result = (*result) * (frequency_error * SX127x_FREQ_ERROR_FACTOR * bandwidth / 500000.0f);
    return SX127X_OK;
  } else if (device->active_modem == SX127x_MODULATION_FSK || device->active_modem == SX127x_MODULATION_OOK) {
    uint32_t frequency_error;
    int code = sx127x_spi_read_registers(REG_FEI_MSB, device->spi_device, 2, &frequency_error);
    if (code != SX127X_OK) {
      return code;
    }
    if (frequency_error & 0x8000) {
      // keep within original 2 bytes
      frequency_error = ((~frequency_error) + 1) & 0xFFFF;
      *result = -1;
    } else {
      *result = 1;
    }
    *result = (*result) * SX127x_FSTEP * frequency_error;
    return SX127X_OK;
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_dump_registers(sx127x *device) {
  uint8_t length = 0x7F;
  for (int i = 1; i < length; i++) {
    uint8_t value;
    sx127x_read_register(i, device->spi_device, &value);
    printf("0x%2x: 0x%2x\n", i, value);
  }
  return SX127X_OK;
}

int sx127x_set_dio_mapping1(sx127x_dio_mapping1_t value, sx127x *device) {
  return sx127x_spi_write_register(REG_DIO_MAPPING_1, (uint8_t *)&value, 1, device->spi_device);
}

int sx127x_set_dio_mapping2(sx127x_dio_mapping2_t value, sx127x *device) {
  return sx127x_spi_write_register(REG_DIO_MAPPING_2, (uint8_t *)&value, 1, device->spi_device);
}

void sx127x_set_tx_callback(void (*tx_callback)(sx127x *), sx127x *device) {
  device->tx_callback = tx_callback;
}

int sx127x_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device) {
  if (pin == SX127x_PA_PIN_RFO && (power < -4 || power > 15)) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (pin == SX127x_PA_PIN_BOOST && (power < 2 || power > 20)) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t data[] = {0};
  if (pin == SX127x_PA_PIN_BOOST && power == 20) {
    data[0] = SX127x_HIGH_POWER_ON;
  } else {
    data[0] = SX127x_HIGH_POWER_OFF;
  }
  int code = sx127x_spi_write_register(REG_PA_DAC, data, 1, device->spi_device);
  if (code != SX127X_OK) {
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
  if (code != SX127X_OK) {
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
  return sx127x_spi_write_register(REG_PA_CONFIG, &value, 1, device->spi_device);
}

int sx127x_set_ocp(sx127x_ocp_t onoff, uint8_t max_current, sx127x *device) {
  uint8_t data[1];
  if (onoff == SX127x_OCP_OFF) {
    data[0] = SX127x_OCP_OFF;
    return sx127x_spi_write_register(REG_OCP, data, 1, device->spi_device);
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
  return sx127x_spi_write_register(REG_OCP, data, 1, device->spi_device);
}

int sx127x_set_tx_explicit_header(sx127x_tx_header_t *header, sx127x *device) {
  if (header == NULL) {
    return SX127X_ERR_INVALID_ARG;
  }
  int code = sx127x_append_register(REG_MODEM_CONFIG_1, header->coding_rate | SX127x_HEADER_MODE_EXPLICIT, 0b11110000, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_append_register(REG_MODEM_CONFIG_2, header->crc, 0b11111011, device->spi_device);
}

int sx127x_set_for_transmission(uint8_t *data, uint8_t data_length, sx127x *device) {
  // uint8_t can't be more than MAX_PACKET_SIZE
  if (data_length == 0) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (device->active_modem == SX127x_MODULATION_LORA) {
    uint8_t fifo_addr[] = {FIFO_TX_BASE_ADDR};
    int code = sx127x_spi_write_register(REG_FIFO_ADDR_PTR, fifo_addr, 1, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    uint8_t reg_data[] = {data_length};
    code = sx127x_spi_write_register(REG_PAYLOAD_LENGTH, reg_data, 1, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
    return sx127x_spi_write_buffer(REG_FIFO, data, data_length, device->spi_device);
  } else if (device->active_modem == SX127x_MODULATION_FSK || device->active_modem == SX127x_MODULATION_OOK) {
    if (device->fsk_ook_format == SX127X_VARIABLE && data_length > (MAX_PACKET_SIZE_FSK_OOK - 1)) {
      return SX127X_ERR_INVALID_ARG;
    }
    if (device->fsk_ook_format == SX127X_FIXED && data_length > MAX_PACKET_SIZE_FSK_OOK) {
      return SX127X_ERR_INVALID_ARG;
    }
    if (device->fsk_ook_format == SX127X_VARIABLE) {
      int code = sx127x_spi_write_register(REG_FIFO, &data_length, 1, device->spi_device);
      if (code != SX127X_OK) {
        return code;
      }
    }
    // if address filtering is required for tx, then it should be part of data
    return sx127x_spi_write_buffer(REG_FIFO, data, data_length, device->spi_device);
  }
  return SX127X_ERR_INVALID_ARG;
}

void sx127x_set_cad_callback(void (*cad_callback)(sx127x *, int), sx127x *device) {
  device->cad_callback = cad_callback;
}

int sx127x_fsk_ook_set_bitrate(float bitrate, sx127x *device) {
  uint16_t bitrate_value;
  uint8_t bitrate_fractional;
  if (device->active_modem == SX127x_MODULATION_FSK) {
    uint32_t value = (uint32_t)(SX127x_OSCILLATOR_FREQUENCY * 16.0 / bitrate);
    bitrate_value = (value >> 4) & 0xFFFF;
    bitrate_fractional = value & 0x0F;
  } else if (device->active_modem == SX127x_MODULATION_OOK) {
    bitrate_value = (uint16_t)(SX127x_OSCILLATOR_FREQUENCY / bitrate);
    bitrate_fractional = 0;
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t data[] = {(uint8_t)(bitrate_value >> 8), (uint8_t)(bitrate_value >> 0)};
  int code = sx127x_spi_write_register(REG_BITRATE_MSB, data, 2, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_spi_write_register(REG_BITRATE_FRAC, &bitrate_fractional, 1, device->spi_device);
}

int sx127x_fsk_ook_set_fdev(float frequency_deviation, sx127x *device) {
  uint16_t value = (uint16_t)(frequency_deviation / SX127x_FSTEP);
  uint8_t data[] = {(uint8_t)(value >> 8), (uint8_t)(value >> 0)};
  return sx127x_spi_write_register(REG_FDEV_MSB, data, 2, device->spi_device);
}

int sx127x_ook_set_peak_mode(sx127x_ook_peak_thresh_step_t step, uint8_t floor_threshold, sx127x_ook_peak_thresh_dec_t decrement, sx127x *device) {
  int code = sx127x_spi_write_register(REG_OOK_FIX, &floor_threshold, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  code = sx127x_append_register(REG_OOK_AVG, decrement, 0b00011111, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_append_register(REG_OOK_PEAK, (0b00001000 | step), 0b11100000, device->spi_device);
}

int sx127x_ook_set_fixed_mode(uint8_t fixed_threshold, sx127x *device) {
  int code = sx127x_spi_write_register(REG_OOK_FIX, &fixed_threshold, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_append_register(REG_OOK_PEAK, 0b00000000, 0b11100111, device->spi_device);
}

int sx127x_ook_set_avg_mode(sx127x_ook_avg_offset_t avg_offset, sx127x_ook_avg_thresh_t avg_thresh, sx127x *device) {
  int code = sx127x_append_register(REG_OOK_AVG, (avg_offset | avg_thresh), 0b11110000, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_append_register(REG_OOK_PEAK, 0b00010000, 0b11100111, device->spi_device);
}

int sx127x_fsk_ook_rx_collision_restart(int enable, uint8_t threshold, sx127x *device) {
  int code = sx127x_spi_write_register(REG_RSSI_COLLISION, &threshold, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_append_register(REG_RX_CONFIG, (enable << 7), 0b01111111, device->spi_device);
}

int sx127x_fsk_ook_set_afc_auto(sx127x_afc_auto_t afc_auto, sx127x *device) {
  return sx127x_append_register(REG_RX_CONFIG, afc_auto, 0b11101111, device->spi_device);
}

uint8_t sx127x_fsk_ook_calculate_bw_register(float bandwidth) {
  for (uint8_t e = 7; e >= 1; e--) {
    for (int8_t m = 2; m >= 0; m--) {
      float point = SX127x_OSCILLATOR_FREQUENCY / (((4 * m) + 16) * ((uint32_t)1 << (e + 2)));
      if (fabs(bandwidth - (point + 0.05)) <= 0.5) {
        return ((m << 3) | e);
      }
    }
  }
  return 0;
}

int sx127x_fsk_ook_set_afc_bandwidth(float bandwidth, sx127x *device) {
  uint8_t value = sx127x_fsk_ook_calculate_bw_register(bandwidth);
  return sx127x_spi_write_register(REG_AFC_BW, &value, 1, device->spi_device);
}

int sx127x_fsk_ook_set_rx_bandwidth(float bandwidth, sx127x *device) {
  uint8_t value = sx127x_fsk_ook_calculate_bw_register(bandwidth);
  return sx127x_spi_write_register(REG_RX_BW, &value, 1, device->spi_device);
}

int sx127x_fsk_ook_set_rx_trigger(sx127x_rx_trigger_t trigger, sx127x *device) {
  return sx127x_append_register(REG_RX_CONFIG, trigger, 0b11111000, device->spi_device);
}

int sx127x_fsk_ook_set_syncword(uint8_t *syncword, uint8_t syncword_length, sx127x *device) {
  if (syncword_length == 0 || syncword_length > 8) {
    return SX127X_ERR_INVALID_ARG;
  }
  for (uint8_t i = 0; i < syncword_length; i++) {
    if (syncword[i] == 0x00) {
      return SX127X_ERR_INVALID_ARG;
    }
  }
  // SYNC_ON
  int code = sx127x_append_register(REG_SYNC_CONFIG, 0b00010000 | (syncword_length - 1), 0b11101000, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  return sx127x_spi_write_buffer(REG_SYNC_VALUE1, syncword, syncword_length, device->spi_device);
}

int sx127x_fsk_ook_set_rssi_config(sx127x_rssi_smoothing_t smoothing, int8_t offset, sx127x *device) {
  if (offset < -16 || offset > 15) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value = (offset << 3) | smoothing;
  return sx127x_spi_write_register(REG_RSSI_CONFIG, &value, 1, device->spi_device);
}

int sx127x_fsk_ook_set_packet_encoding(sx127x_packet_encoding_t encoding, sx127x *device) {
  return sx127x_append_register(REG_PACKET_CONFIG1, encoding, 0b10011111, device->spi_device);
}

int sx127x_fsk_ook_set_crc(sx127x_crc_type_t crc_type, sx127x *device) {
  return sx127x_append_register(REG_PACKET_CONFIG1, crc_type, 0b11100110, device->spi_device);
}

int sx127x_fsk_ook_set_packet_format(sx127x_packet_format_t format, uint16_t max_payload_length, sx127x *device) {
  if (format == SX127X_FIXED && (max_payload_length == 0 || max_payload_length > 2047)) {
    return SX127X_ERR_INVALID_ARG;
  }
  // max_payload_length = 2047 in variable packet mode will disable max payload length check
  if (format == SX127X_VARIABLE && (max_payload_length == 0 || (max_payload_length > 255 && max_payload_length != 2047))) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t msb_bits = ((max_payload_length >> 8) & 0b111);
  int code = sx127x_append_register(REG_PACKET_CONFIG2, msb_bits, 0b11111000, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  uint8_t lsb_bits = (max_payload_length & 0xFF);
  code = sx127x_spi_write_register(REG_PAYLOAD_LENGTH_FSK, &lsb_bits, 1, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  code = sx127x_append_register(REG_PACKET_CONFIG1, format, 0b01111111, device->spi_device);
  if (code != SX127X_OK) {
    return code;
  }
  device->fsk_ook_format = format;
  return SX127X_OK;
}

int sx127x_fsk_ook_set_address_filtering(sx127x_address_filtering_t type, uint8_t node_address, uint8_t broadcast_address, sx127x *device) {
  if (type == SX127X_FILTER_NODE_AND_BROADCAST) {
    int code = sx127x_spi_write_register(REG_BROADCAST_ADDR, &broadcast_address, 1, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
  }
  if (type == SX127X_FILTER_NODE_AND_BROADCAST || type == SX127X_FILTER_NODE_ADDRESS) {
    int code = sx127x_spi_write_register(REG_NODE_ADDR, &node_address, 1, device->spi_device);
    if (code != SX127X_OK) {
      return code;
    }
  }
  return sx127x_append_register(REG_PACKET_CONFIG1, type, 0b11111001, device->spi_device);
}

int sx127x_fsk_set_data_shaping(sx127x_fsk_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device) {
  uint8_t value = (data_shaping | pa_ramp);
  return sx127x_spi_write_register(REG_PA_RAMP, &value, 1, device->spi_device);
}

int sx127x_ook_set_data_shaping(sx127x_ook_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device) {
  uint8_t value = (data_shaping | pa_ramp);
  return sx127x_spi_write_register(REG_PA_RAMP, &value, 1, device->spi_device);
}

int sx127x_fsk_ook_set_preamble_type(sx127x_preamble_type_t type, sx127x *device) {
  return sx127x_append_register(REG_SYNC_CONFIG, type, 0b11011111, device->spi_device);
}

void sx127x_destroy(sx127x *device) {
  if (device == NULL) {
    return;
  }
  free(device);
}