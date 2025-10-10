// Copyright 2022 Andrey Rodionov <dernasherbrezon@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "sx127x.h"

#include <math.h>
#include <string.h>
#include <sx127x_spi.h>
#include <sx127x_registers.h>

#define SX127X_OSCILLATOR_FREQUENCY 32000000.0f
#define SX127X_FREQ_ERROR_FACTOR ((1 << 24) / SX127X_OSCILLATOR_FREQUENCY)
#define SX127X_FSTEP (SX127X_OSCILLATOR_FREQUENCY / (1 << 19))
#define SX127X_REG_MODEM_CONFIG_3_AGC_ON 0b00000100
#define SX127X_REG_MODEM_CONFIG_3_AGC_OFF 0b00000000

#define SX127X_IRQ_FLAG_RXTIMEOUT 0b10000000
#define SX127X_IRQ_FLAG_RXDONE 0b01000000
#define SX127X_IRQ_FLAG_PAYLOAD_CRC_ERROR 0b00100000
#define SX127X_IRQ_FLAG_VALID_HEADER 0b00010000
#define SX127X_IRQ_FLAG_TXDONE 0b00001000
#define SX127X_IRQ_FLAG_CADDONE 0b00000100
#define SX127X_IRQ_FLAG_FHSSCHANGECHANNEL 0b00000010
#define SX127X_IRQ_FLAG_CAD_DETECTED 0b00000001

#define SX127X_FSK_IRQ_FIFO_FULL 0b10000000
#define SX127X_FSK_IRQ_FIFO_EMPTY 0b01000000
#define SX127X_FSK_IRQ_FIFO_LEVEL 0b00100000
#define SX127X_FSK_IRQ_FIFO_OVERRUN 0b00010000
#define SX127X_FSK_IRQ_PACKET_SENT 0b00001000
#define SX127X_FSK_IRQ_PAYLOAD_READY 0b00000100
#define SX127X_FSK_IRQ_CRC_OK 0b00000010
#define SX127X_FSK_IRQ_LOW_BATTERY 0b00000001
#define SX127X_FSK_IRQ_PREAMBLE_DETECT 0b00000010
#define SX127X_FSK_IRQ_SYNC_ADDRESS_MATCH 0b00000001

#define RF_MID_BAND_THRESHOLD 525000000
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define SX127X_MAX_POWER 0b01110000
#define SX127X_LOW_POWER 0b00000000

#define SX127X_HIGH_POWER_ON 0b10000111
#define SX127X_HIGH_POWER_OFF 0b10000100

#define FIFO_TX_BASE_ADDR 0b00000000
#define FIFO_RX_BASE_ADDR 0b00000000

#define FIFO_SIZE_FSK 64
#define MAX_FIFO_THRESHOLD 0b00111111
#define HALF_MAX_FIFO_THRESHOLD (MAX_FIFO_THRESHOLD >> 1)
#define TX_START_CONDITION_FIFO_LEVEL 0b00000000
#define TX_START_CONDITION_FIFO_EMPTY 0b10000000

#define SHADOW_EMPTY 0
#define SHADOW_CACHED 1
#define SHADOW_IGNORE 2

#define ERROR_CHECK(x)           \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return __err_rc;           \
    }                            \
  } while (0)

#define ERROR_CHECK_NOCODE(x)    \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return;                    \
    }                            \
  } while (0)

#define CHECK_MODULATION(x, y)         \
  do {                                 \
    if (x->active_modem != y) {        \
      return SX127X_ERR_INVALID_STATE; \
    }                                  \
  } while (0)

#define CHECK_FSK_OOK_MODULATION(x)                                                             \
  do {                                                                                          \
    if (x->active_modem != SX127X_MODULATION_FSK && x->active_modem != SX127X_MODULATION_OOK) { \
      return SX127X_ERR_INVALID_STATE;                                                          \
    }                                                                                           \
  } while (0)

static uint8_t sx1276_bw[] = {0b00000000, 0b00010000, 0b00100000, 0b00110000, 0b01000000, 0b01010000, 0b01100000, 0b01110000, 0b10000000, 0b10010000};
static uint8_t sx1272_bw[] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01000000, 0b10000000};
static uint8_t sx1276_cr[] = {0b00000010, 0b00000100, 0b00000110, 0b00001000};
static uint8_t sx1272_cr[] = {0b00001000, 0b00010000, 0b00011000, 0b00100000};

static int sx127x_shadow_spi_read_registers(int reg, shadow_spi_device_t *spi_device, size_t data_length, uint32_t *result) {
#ifdef CONFIG_SX127X_DISABLE_SPI_CACHE
  return sx127x_spi_read_registers(reg, spi_device->spi_device, data_length, result);
#else
  if (spi_device->shadow_registers_sync[reg] == SHADOW_IGNORE) {
    return sx127x_spi_read_registers(reg, spi_device->spi_device, data_length, result);
  }
  size_t cached_length = 0;
  uint32_t cached = 0;
  for (size_t i = 0; i < data_length; i++) {
    if (spi_device->shadow_registers_sync[reg + i] != SHADOW_CACHED) {
      break;
    }
    cached = (cached << 8);
    cached = cached | spi_device->shadow_registers[reg + i];
    cached_length++;
  }
  if (cached_length == data_length) {
    *result = cached;
    return SX127X_OK;
  }

  int code = sx127x_spi_read_registers(reg, spi_device->spi_device, data_length, result);
  if (code != SX127X_OK) {
    return code;
  }

  const uint8_t *pointer = ((uint8_t *) result) + (sizeof(uint32_t) - data_length);
  memcpy(spi_device->shadow_registers + reg, pointer, data_length);
  memset(spi_device->shadow_registers_sync + reg, SHADOW_CACHED, data_length);
  return code;
#endif
}

static int sx127x_shadow_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, shadow_spi_device_t *spi_device) {
  // it's always REG_FIFO
  return sx127x_spi_read_buffer(reg, buffer, buffer_length, spi_device->spi_device);
}

static int sx127x_shadow_spi_write_register(int reg, const uint8_t *data, size_t data_length, shadow_spi_device_t *spi_device) {
  int code = sx127x_spi_write_register(reg, data, data_length, spi_device->spi_device);
#ifndef CONFIG_SX127X_DISABLE_SPI_CACHE
  if (code != SX127X_OK || spi_device->shadow_registers_sync[reg] == SHADOW_IGNORE) {
    return code;
  }
  memcpy(spi_device->shadow_registers + reg, data, data_length);
  memset(spi_device->shadow_registers_sync + reg, SHADOW_CACHED, data_length);
#endif
  return code;
}

static int sx127x_shadow_spi_write_buffer(int reg, const uint8_t *buffer, size_t buffer_length, shadow_spi_device_t *spi_device) {
  int code = sx127x_spi_write_buffer(reg, buffer, buffer_length, spi_device->spi_device);
#ifndef CONFIG_SX127X_DISABLE_SPI_CACHE
  if (code != SX127X_OK || spi_device->shadow_registers_sync[reg] == SHADOW_IGNORE) {
    return code;
  }
  memcpy(spi_device->shadow_registers + reg, buffer, buffer_length);
  memset(spi_device->shadow_registers_sync + reg, SHADOW_CACHED, buffer_length);
#endif
  return code;
}

int sx127x_write_register(int reg, uint8_t value, shadow_spi_device_t *spi_device) {
  return sx127x_shadow_spi_write_register(reg, &value, 1, spi_device);
}

int sx127x_read_register(int reg, shadow_spi_device_t *spi_device, uint8_t *result) {
#ifdef CONFIG_SX127X_DISABLE_SPI_CACHE
  uint32_t value;
  ERROR_CHECK(sx127x_spi_read_registers(reg, spi_device->spi_device, 1, &value));
  *result = (uint8_t) value;
  return SX127X_OK;
#else
  if (spi_device->shadow_registers_sync[reg] == SHADOW_IGNORE) {
    uint32_t value;
    ERROR_CHECK(sx127x_spi_read_registers(reg, spi_device->spi_device, 1, &value));
    *result = (uint8_t) value;
    return SX127X_OK;
  }
  if (spi_device->shadow_registers_sync[reg] == SHADOW_CACHED) {
    *result = spi_device->shadow_registers[reg];
    return SX127X_OK;
  }
  uint32_t value;
  ERROR_CHECK(sx127x_spi_read_registers(reg, spi_device->spi_device, 1, &value));
  *result = (uint8_t) value;
  spi_device->shadow_registers_sync[reg] = SHADOW_CACHED;
  spi_device->shadow_registers[reg] = *result;
  return SX127X_OK;
#endif
}

static int sx127x_append_register(int reg, uint8_t value, uint8_t mask, shadow_spi_device_t *spi_device) {
  uint8_t previous = 0;
  ERROR_CHECK(sx127x_read_register(reg, spi_device, &previous));
  uint8_t data[] = {(previous & mask) | value};
  return sx127x_shadow_spi_write_register(reg, data, 1, spi_device);
}

int sx127x_lora_set_low_datarate_optimization(bool enable, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (device->chip_version == SX1276_VERSION) {
    uint8_t value = (enable ? 0b00001000 : 0b00000000);
    ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG3, value, 0b11110111, &device->spi_device));
  } else {
    uint8_t value = (enable ? 0b00000001 : 0b00000000);
    ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, value, 0b11111110, &device->spi_device));
  }
  return SX127X_OK;
}

int sx127x_lora_get_low_datarate_optimization(sx127x *device, bool *enabled) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t raw;
  if (device->chip_version == SX1276_VERSION) {
    ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG3, &device->spi_device, &raw));
    *enabled = (raw & 0b00001000) > 0;
  } else {
    ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG1, &device->spi_device, &raw));
    *enabled = (raw & 0b00000001) > 0;
  }
  return SX127X_OK;
}

int sx127x_lora_get_bandwidth(sx127x *device, sx127x_bw_t *bandwidth) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG1, &device->spi_device, &raw));
  if (device->chip_version == SX1272_VERSION) {
    *bandwidth = 7 + ((raw & 0b11000000) >> 6);
  } else {
    *bandwidth = (raw & 0b11110000) >> 4;
  }
  return SX127X_OK;
}

uint32_t sx127x_bandwidth_to_hz(sx127x_bw_t bandwidth) {
  switch (bandwidth) {
    case 0:
      return 7800;
    case 1:
      return 10400;
    case 2:
      return 15600;
    case 3:
      return 20800;
    case 4:
      return 31250;
    case 5:
      return 41700;
    case 6:
      return 62500;
    case 7:
      return 125000;
    case 8:
      return 250000;
    case 9:
      return 500000;
    default:
      return 0;
  }
}

sx127x_bw_t sx127x_hz_to_bandwidth(uint32_t bandwidth_hz) {
  switch (bandwidth_hz) {
    case 7800:
      return SX127X_BW_7800;
    case 10400:
      return SX127X_BW_10400;
    case 15600:
      return SX127X_BW_15600;
    case 20800:
      return SX127X_BW_20800;
    case 31250:
      return SX127X_BW_31250;
    case 41700:
      return SX127X_BW_41700;
    case 62500:
      return SX127X_BW_62500;
    case 125000:
      return SX127X_BW_125000;
    case 250000:
      return SX127X_BW_250000;
    case 500000:
      return SX127X_BW_500000;
    default:
      return -1;
  }
}

int sx127x_reload_low_datarate_optimization(sx127x *device) {
  sx127x_bw_t bandwidth_enum;
  ERROR_CHECK(sx127x_lora_get_bandwidth(device, &bandwidth_enum));
  uint32_t bandwidth = sx127x_bandwidth_to_hz(bandwidth_enum);
  uint8_t spreading_factor = 0;
  ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG2, &device->spi_device, &spreading_factor));
  spreading_factor = (spreading_factor >> 4);

  // Section 4.1.1.5
  uint32_t symbol_duration = 1000 / (bandwidth / (1L << spreading_factor));
  if (symbol_duration > 16) {
    // force low data rate optimization
    return sx127x_lora_set_low_datarate_optimization(true, device);
  } else {
    return sx127x_lora_set_low_datarate_optimization(false, device);
  }
}

int sx127x_fsk_ook_read_fixed_packet_length(sx127x *device, uint16_t *packet_length) {
  uint16_t result;
  uint8_t value;
  ERROR_CHECK(sx127x_read_register(REGPACKETCONFIG2, &device->spi_device, &value));
  result = ((value & 0b00000111) << 8);
  ERROR_CHECK(sx127x_read_register(REGPAYLOADLENGTH_FSK, &device->spi_device, &value));
  result += value;
  *packet_length = result;
  return SX127X_OK;
}

int sx127x_fsk_ook_is_address_filtered(sx127x *device, bool *address_filtered) {
  uint8_t value;
  ERROR_CHECK(sx127x_read_register(REGPACKETCONFIG1, &device->spi_device, &value));
  value = (value & 0b00000110);
  *address_filtered = (value == SX127X_FILTER_NODE_ADDRESS || value == SX127X_FILTER_NODE_AND_BROADCAST);
  return SX127X_OK;
}

// ignore status code here. it will be returned in the read_payload function
void sx127x_fsk_ook_read_payload_batch(bool read_batch, sx127x *device) {
  uint8_t remaining_fifo = FIFO_SIZE_FSK;
  if (device->expected_packet_length == 0) {
    uint16_t packet_length;
    if (device->fsk_ook_format == SX127X_FIXED) {
      int code = sx127x_fsk_ook_read_fixed_packet_length(device, &packet_length);
      if (code != SX127X_OK) {
        return;
      }
    } else if (device->fsk_ook_format == SX127X_VARIABLE) {
      uint8_t value;
      int code = sx127x_read_register(REGFIFO, &device->spi_device, &value);
      if (code != SX127X_OK) {
        return;
      }
      packet_length = value;
      remaining_fifo--;
    } else {
      return;
    }
    device->expected_packet_length = packet_length;
    bool address_filtered;
    int code = sx127x_fsk_ook_is_address_filtered(device, &address_filtered);
    if (code != SX127X_OK) {
      return;
    }
    // if node filtering is enabled, then skip next byte because it will be node id
    if (address_filtered) {
      uint8_t value;
      code = sx127x_read_register(REGFIFO, &device->spi_device, &value);
      if (code != SX127X_OK) {
        return;
      }
      device->expected_packet_length--;
      remaining_fifo--;
    }
  }

  // safe check
  if (device->expected_packet_length == device->fsk_ook_packet_sent_received) {
    return;
  }

  uint8_t batch_size = HALF_MAX_FIFO_THRESHOLD - 1;
  if (read_batch && device->fsk_ook_packet_sent_received + batch_size < device->expected_packet_length) {
    int code = sx127x_shadow_spi_read_buffer(REGFIFO, device->packet + device->fsk_ook_packet_sent_received, batch_size, &device->spi_device);
    if (code != SX127X_OK) {
      return;
    }
    device->fsk_ook_packet_sent_received += batch_size;
  } else {
    // shortcut here for packets less than max fifo size
    if (device->fsk_ook_packet_sent_received == 0 && device->expected_packet_length <= remaining_fifo) {
      int code = sx127x_shadow_spi_read_buffer(REGFIFO, device->packet, device->expected_packet_length, &device->spi_device);
      if (code != SX127X_OK) {
        return;
      }
      device->fsk_ook_packet_sent_received = device->expected_packet_length;
    } else {
      // else read remaining bytes one by one and check FIFO_EMPTY irq
      uint8_t irq;
      do {
        uint8_t value;
        int code = sx127x_read_register(REGFIFO, &device->spi_device, &value);
        if (code != SX127X_OK) {
          return;
        }
        device->packet[device->fsk_ook_packet_sent_received] = value;
        device->fsk_ook_packet_sent_received++;
        code = sx127x_read_register(REGIRQFLAGS2, &device->spi_device, &irq);
        if (code != SX127X_OK) {
          return;
        }
      } while ((irq & SX127X_FSK_IRQ_FIFO_EMPTY) == 0);
    }
  }
}

int sx127x_fsk_ook_get_rssi(sx127x *device) {
  uint8_t value;
  ERROR_CHECK(sx127x_read_register(REGRSSIVALUE_FSK, &device->spi_device, &value));
  device->fsk_rssi_available = true;
  device->fsk_rssi = -value / 2;
  // TODO read offset and add here?
  return SX127X_OK;
}

static void sx127x_fsk_ook_reset_state(sx127x *device) {
  device->expected_packet_length = 0;
  device->fsk_ook_packet_sent_received = 0;
  device->fsk_rssi = 0;
  device->fsk_rssi_available = false;
}

static void sx127x_fsk_ook_handle_interrupt(sx127x *device) {
  uint8_t irq;
  ERROR_CHECK_NOCODE(sx127x_read_register(REGIRQFLAGS2, &device->spi_device, &irq));
  if ((irq & SX127X_FSK_IRQ_PAYLOAD_READY) != 0) {
    if (device->fsk_crc_type != SX127X_CRC_NONE && (irq & SX127X_FSK_IRQ_CRC_OK) != SX127X_FSK_IRQ_CRC_OK) {
      irq = SX127X_FSK_IRQ_FIFO_OVERRUN;
      ERROR_CHECK_NOCODE(sx127x_shadow_spi_write_register(REGIRQFLAGS2, &irq, 1, &device->spi_device));
    } else {
      // read remaining of FIFO into the packet
      sx127x_fsk_ook_read_payload_batch(false, device);
      if (device->rx_callback != NULL) {
        device->rx_callback(device->rx_callback_ctx, device->packet, device->expected_packet_length);
      }
    }
    sx127x_fsk_ook_reset_state(device);
    return;
  }
  if ((irq & SX127X_FSK_IRQ_PACKET_SENT) != 0) {
    sx127x_fsk_ook_reset_state(device);
    if (device->tx_callback != NULL) {
      device->tx_callback(device->tx_callback_ctx);
    }
    return;
  }
  if (device->opmod == SX127X_MODE_TX) {
    if ((irq & SX127X_FSK_IRQ_FIFO_EMPTY) != 0) {
      // TX sequencer clears PACKET_SENT IRQ so only FIFO_EMPTY interrupt can be used to detect if message was actually sent
      sx127x_fsk_ook_reset_state(device);
      if (device->tx_callback != NULL) {
        device->tx_callback(device->tx_callback_ctx);
      }
      return;
    }
    // FIFO_LEVEL == 0 - below level
    if ((irq & SX127X_FSK_IRQ_FIFO_LEVEL) == 0 && (irq & SX127X_FSK_IRQ_FIFO_FULL) == 0) {
      uint8_t to_send;
      if (device->expected_packet_length - device->fsk_ook_packet_sent_received > (HALF_MAX_FIFO_THRESHOLD - 1)) {
        to_send = HALF_MAX_FIFO_THRESHOLD - 1;
      } else {
        to_send = (uint8_t) (device->expected_packet_length - device->fsk_ook_packet_sent_received);
      }
      // tx still sending the data
      // ignore interrupt
      // this can happen when exactly 63 bytes sent and FIFO_LEVEL is ~30bytes
      if (to_send == 0) {
        return;
      }
      // remaining bits not written to FIFO but modulator will eventually trigger SX127X_FSK_IRQ_PACKET_SENT
      ERROR_CHECK_NOCODE(sx127x_shadow_spi_write_buffer(REGFIFO, device->packet + device->fsk_ook_packet_sent_received, to_send, &device->spi_device));
      device->fsk_ook_packet_sent_received += to_send;
    }
  } else if (device->opmod == SX127X_MODE_RX_CONT || device->opmod == SX127X_MODE_RX_SINGLE) {
    if ((irq & SX127X_FSK_IRQ_FIFO_LEVEL) != 0 && (irq & SX127X_FSK_IRQ_FIFO_FULL) == 0) {
      sx127x_fsk_ook_read_payload_batch(true, device);
    } else {
      // if not RX irq, then try preamble detect
      ERROR_CHECK_NOCODE(sx127x_read_register(REGIRQFLAGS1, &device->spi_device, &irq));
      if ((irq & SX127X_FSK_IRQ_PREAMBLE_DETECT) != 0 && !device->fsk_rssi_available) {
        sx127x_fsk_ook_get_rssi(device);
        return;
      }
      // if preamble dio not attached, then try sync_address match
      if ((irq & SX127X_FSK_IRQ_SYNC_ADDRESS_MATCH) != 0 && !device->fsk_rssi_available) {
        sx127x_fsk_ook_get_rssi(device);
        return;
      }
    }
  }
}

static int sx127x_lora_rx_read_payload(sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t length;
  if (device->expected_packet_length == 0) {
    ERROR_CHECK(sx127x_read_register(REGRXNBBYTES, &device->spi_device, &length));
  } else {
    length = (uint8_t) device->expected_packet_length;
  }
  device->expected_packet_length = length;

  uint8_t current;
  ERROR_CHECK(sx127x_read_register(REGFIFORXCURRENTADDR, &device->spi_device, &current));
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGFIFOADDRPTR, &current, 1, &device->spi_device));
  return sx127x_shadow_spi_read_buffer(REGFIFO, device->packet, device->expected_packet_length, &device->spi_device);
}

static void sx127x_lora_handle_interrupt(sx127x *device) {
  uint8_t value;
  ERROR_CHECK_NOCODE(sx127x_read_register(REGIRQFLAGS, &device->spi_device, &value));
  ERROR_CHECK_NOCODE(sx127x_shadow_spi_write_register(REGIRQFLAGS, &value, 1, &device->spi_device));
  if ((value & SX127X_IRQ_FLAG_CADDONE) != 0) {
    if (device->cad_callback != NULL) {
      device->cad_callback(device->cad_callback_ctx, value & SX127X_IRQ_FLAG_CAD_DETECTED);
    }
    return;
  }
  if ((value & SX127X_IRQ_FLAG_PAYLOAD_CRC_ERROR) != 0) {
    device->current_frequency = 0;
    return;
  }
  if ((value & SX127X_IRQ_FLAG_RXDONE) != 0) {
    ERROR_CHECK_NOCODE(sx127x_lora_rx_read_payload(device));
    if (device->rx_callback != NULL) {
      device->rx_callback(device->rx_callback_ctx, device->packet, device->expected_packet_length);
    }
    device->expected_packet_length = 0;
    device->current_frequency = 0;
    return;
  }
  if ((value & SX127X_IRQ_FLAG_TXDONE) != 0) {
    device->current_frequency = 0;
    if (device->tx_callback != NULL) {
      device->tx_callback(device->tx_callback_ctx);
    }
    return;
  }
  // always last because if message was sent or received,
  // then no need to change freq
  if ((value & SX127X_IRQ_FLAG_FHSSCHANGECHANNEL) != 0) {
    if (device->current_frequency >= device->frequencies_length) {
      device->current_frequency = 0;
    }
    ERROR_CHECK_NOCODE(sx127x_set_frequency(device->frequencies[device->current_frequency], device));
    device->current_frequency++;
    return;
  }
}

void sx127x_handle_interrupt(sx127x *device) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    sx127x_lora_handle_interrupt(device);
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    sx127x_fsk_ook_handle_interrupt(device);
  }
}

int sx127x_create(void *spi_device, sx127x *result) {
  memset(result, 0, sizeof(struct sx127x_t));
  result->spi_device.spi_device = spi_device;
#ifndef CONFIG_SX127X_DISABLE_SPI_CACHE
  result->spi_device.shadow_registers_sync[REGFIFO] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGOPMODE] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGFIFORXCURRENTADDR] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGRSSIVALUE_FSK] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGIRQFLAGS] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGRXNBBYTES] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGPKTSNRVALUE] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGPKTRSSIVALUE] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGFEIMSB] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGAFCMSB] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGSEQCONFIG1] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGIMAGECAL] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGTEMP] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGIRQFLAGS1] = SHADOW_IGNORE;
  result->spi_device.shadow_registers_sync[REGIRQFLAGS2] = SHADOW_IGNORE;
#endif

  int code = sx127x_read_register(REGVERSION, &result->spi_device, &result->chip_version);
  if (code != SX127X_OK) {
    return code;
  }
  if (result->chip_version != SX1276_VERSION && result->chip_version != SX1272_VERSION) {
    return SX127X_ERR_INVALID_VERSION;
  }
  // read current modem state
  // there is no way for application to determine it's state on startup or after deep sleep
  code = sx127x_get_opmod(result, &result->opmod, &result->active_modem);
  if (code != SX127X_OK) {
    return code;
  }
  sx127x_reset(result);
  return SX127X_OK;
}

int sx127x_set_opmod(sx127x_mode_t opmod, sx127x_modulation_t modulation, sx127x *device) {
  // go to sleep if requested modulation changes LongRangeMode
  if (((device->active_modem & 0b10000000) != (modulation & 0b10000000)) && device->opmod != SX127X_MODE_SLEEP) {
    ERROR_CHECK(sx127x_append_register(REGOPMODE, SX127X_MODE_SLEEP, 0b11111000, &device->spi_device));
    ERROR_CHECK(sx127x_append_register(REGOPMODE, modulation, 0b00011111, &device->spi_device));
  }
  // enforce DIO mappings for RX and TX
  if (modulation == SX127X_MODULATION_LORA) {
    if (opmod == SX127X_MODE_RX_CONT || opmod == SX127X_MODE_RX_SINGLE) {
      uint8_t data = (SX127X_DIO0_RX_DONE | SX127X_DIO1_RXTIMEOUT | SX127X_DIO2_FHSS_CHANGE_CHANNEL | SX127X_DIO3_CAD_DONE);
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGDIOMAPPING1, &data, 1, &device->spi_device));
    } else if (opmod == SX127X_MODE_TX) {
      uint8_t data = (SX127X_DIO0_TX_DONE | SX127X_DIO1_FHSS_CHANGE_CHANNEL | SX127X_DIO2_FHSS_CHANGE_CHANNEL | SX127X_DIO3_CAD_DONE);
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGDIOMAPPING1, &data, 1, &device->spi_device));
    } else if (opmod == SX127X_MODE_CAD) {
      ERROR_CHECK(sx127x_append_register(REGDIOMAPPING1, SX127X_DIO0_CAD_DONE, 0b00111111, &device->spi_device));
    }
  } else if (modulation == SX127X_MODULATION_FSK || modulation == SX127X_MODULATION_OOK) {
    if (opmod == SX127X_MODE_RX_CONT || opmod == SX127X_MODE_RX_SINGLE) {
      ERROR_CHECK(sx127x_append_register(REGDIOMAPPING1, SX127X_FSK_DIO0_PAYLOAD_READY | SX127X_FSK_DIO1_FIFO_LEVEL | SX127X_FSK_DIO2_SYNCADDRESS, 0b00000011, &device->spi_device));
      ERROR_CHECK(sx127x_append_register(REGDIOMAPPING2, SX127X_FSK_DIO4_PREAMBLE_DETECT | 0b00000001, 0b00111110, &device->spi_device));
      // configure fifo level threshold for rx
      uint8_t data = HALF_MAX_FIFO_THRESHOLD;
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGFIFOTHRESH, &data, 1, &device->spi_device));
    } else if (opmod == SX127X_MODE_TX) {
      uint8_t data = (SX127X_FSK_DIO0_PACKET_SENT | SX127X_FSK_DIO1_FIFO_LEVEL | SX127X_FSK_DIO2_FIFO_FULL | SX127X_FSK_DIO3_FIFO_EMPTY);
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGDIOMAPPING1, &data, 1, &device->spi_device));
      // start tx as soon as first byte in FIFO available
      data = (TX_START_CONDITION_FIFO_EMPTY | HALF_MAX_FIFO_THRESHOLD);
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGFIFOTHRESH, &data, 1, &device->spi_device));
      // use sequencer to send single packet and stop carrier
      uint8_t value = 0b10010000;
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGSEQCONFIG1, &value, 1, &device->spi_device));
      device->active_modem = modulation;
      device->opmod = opmod;
      return SX127X_OK;
    }
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value = (opmod | modulation);
  int result = sx127x_shadow_spi_write_register(REGOPMODE, &value, 1, &device->spi_device);
  if (result == SX127X_OK) {
    device->active_modem = modulation;
    device->opmod = opmod;
  }
  return result;
}

int sx127x_get_opmod(sx127x *device, sx127x_mode_t *mode, sx127x_modulation_t *modulation) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register((REGOPMODE), &device->spi_device, &raw));
  *mode = raw & 0b111;
  *modulation = raw & 0b11100000;
  return SX127X_OK;
}

int sx127x_set_frequency(uint64_t frequency, sx127x *device) {
  uint64_t min_frequency = device->chip_version == SX1276_VERSION ? SX1276_MIN_FREQUENCY : SX1272_MIN_FREQUENCY;
  if (frequency < min_frequency || frequency > SX127X_MAX_FREQUENCY) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint64_t adjusted = (frequency << 19) / SX127X_OSCILLATOR_FREQUENCY;
  uint8_t data[] = {(uint8_t) (adjusted >> 16), (uint8_t) (adjusted >> 8), (uint8_t) (adjusted >> 0)};
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGFRFMSB, data, 3, &device->spi_device));
  return SX127X_OK;
}

int sx127x_get_frequency(sx127x *device, uint64_t *frequency) {
  uint32_t frequency_raw;
  ERROR_CHECK(sx127x_shadow_spi_read_registers(REGFRFMSB, &device->spi_device, 3, &frequency_raw));
  *frequency = (uint64_t) (frequency_raw * SX127X_OSCILLATOR_FREQUENCY) >> 19;
  return SX127X_OK;
}

int sx127x_lora_reset_fifo(sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  // reset both RX and TX
  uint8_t data[] = {FIFO_TX_BASE_ADDR, FIFO_RX_BASE_ADDR};
  return sx127x_shadow_spi_write_register(REGFIFOTXBASEADDR, data, 2, &device->spi_device);
}

int sx127x_rx_set_lna_gain(sx127x_gain_t gain, sx127x *device) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    if (device->chip_version == SX1276_VERSION) {
      if (gain == SX127X_LNA_GAIN_AUTO) {
        return sx127x_append_register(REGMODEMCONFIG3, SX127X_REG_MODEM_CONFIG_3_AGC_ON, 0b11111011, &device->spi_device);
      }
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG3, SX127X_REG_MODEM_CONFIG_3_AGC_OFF, 0b11111011, &device->spi_device));
    } else {
      if (gain == SX127X_LNA_GAIN_AUTO) {
        return sx127x_append_register(REGMODEMCONFIG2, SX127X_REG_MODEM_CONFIG_3_AGC_ON, 0b11111011, &device->spi_device);
      }
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG2, SX127X_REG_MODEM_CONFIG_3_AGC_OFF, 0b11111011, &device->spi_device));
    }
    return sx127x_append_register(REGLNA, gain, 0b00011111, &device->spi_device);
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    if (gain == SX127X_LNA_GAIN_AUTO) {
      return sx127x_append_register(REGRXCONFIG, 0b00001000, 0b11110111, &device->spi_device);
    }
    // gain manual
    ERROR_CHECK(sx127x_append_register(REGRXCONFIG, 0b00000000, 0b11110111, &device->spi_device));
    return sx127x_append_register(REGLNA, gain, 0b00011111, &device->spi_device);
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_rx_get_lna_gain(sx127x *device, sx127x_gain_t *gain) {
  uint8_t raw;
  if (device->active_modem == SX127X_MODULATION_LORA) {
    bool gain_auto;
    if (device->chip_version == SX1276_VERSION) {
      ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG3, &device->spi_device, &raw));
    } else {
      ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG2, &device->spi_device, &raw));
    }
    gain_auto = (raw & 0b00000100) > 0;
    if (gain_auto) {
      *gain = SX127X_LNA_GAIN_AUTO;
    } else {
      ERROR_CHECK(sx127x_read_register(REGLNA, &device->spi_device, &raw));
      *gain = (raw & 0b11100000);
    }
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    ERROR_CHECK(sx127x_read_register(REGRXCONFIG, &device->spi_device, &raw));
    if ((raw & 0b00001000) > 0) {
      *gain = SX127X_LNA_GAIN_AUTO;
    } else {
      ERROR_CHECK(sx127x_read_register(REGLNA, &device->spi_device, &raw));
      *gain = (raw & 0b11100000);
    }
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  return SX127X_OK;
}

int sx127x_rx_set_lna_boost_hf(bool enable, sx127x *device) {
  uint8_t value = (enable ? 0b00000011 : 0b00000000);
  return sx127x_append_register(REGLNA, value, 0b11111100, &device->spi_device);
}

int sx127x_rx_get_lna_boost_hf(sx127x *device, bool *value) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGLNA, &device->spi_device, &raw));
  *value = (raw & 0b00000011) > 0;
  return SX127X_OK;
}

int sx127x_lora_set_bandwidth(sx127x_bw_t bandwidth, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (device->chip_version == SX1272_VERSION) {
    if (((uint8_t) bandwidth) < (uint8_t) SX127X_BW_125000 || ((uint8_t) bandwidth) > (uint8_t) SX127X_BW_500000) {
      return SX127X_ERR_INVALID_ARG;
    }
    ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, sx1272_bw[bandwidth], 0b00111111, &device->spi_device));
  } else {
    ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, sx1276_bw[bandwidth], 0b00001111, &device->spi_device));
  }
  return sx127x_reload_low_datarate_optimization(device);
}

int sx127x_lora_set_spreading_factor(sx127x_sf_t spreading_factor, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (spreading_factor == SX127X_SF_6 && !device->use_implicit_header) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t detection_optimize;
  uint8_t detection_threshold;
  if (spreading_factor == SX127X_SF_6) {
    detection_optimize = 0xc5;
    detection_threshold = 0x0c;
    // make header implicit
  } else {
    detection_optimize = 0xc3;
    detection_threshold = 0x0a;
  }
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGDETECTOPTIMIZE, &detection_optimize, 1, &device->spi_device));
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGDETECTIONTHRESHOLD, &detection_threshold, 1, &device->spi_device));
  ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG2, spreading_factor, 0b00001111, &device->spi_device));
  return sx127x_reload_low_datarate_optimization(device);
}

int sx127x_lora_get_spreading_factor(sx127x *device, sx127x_sf_t *spreading_factor) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG2, &device->spi_device, &raw));
  *spreading_factor = raw & 0b11110000;
  return SX127X_OK;
}

void sx127x_rx_set_callback(void (*rx_callback)(void *, uint8_t *, uint16_t), void *ctx, sx127x *device) {
  device->rx_callback = rx_callback;
  device->rx_callback_ctx = ctx;
}

int sx127x_lora_set_syncword(uint8_t value, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  return sx127x_shadow_spi_write_register(REGSYNCWORD, &value, 1, &device->spi_device);
}

int sx127x_lora_get_syncword(sx127x *device, uint8_t *value) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  ERROR_CHECK(sx127x_read_register(REGSYNCWORD, &device->spi_device, value));
  return SX127X_OK;
}

int sx127x_set_preamble_length(uint16_t value, sx127x *device) {
  uint8_t data[] = {(uint8_t) (value >> 8), (uint8_t) (value >> 0)};
  if (device->active_modem == SX127X_MODULATION_LORA) {
    return sx127x_shadow_spi_write_register(REGPREAMBLEMSB, data, 2, &device->spi_device);
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    return sx127x_shadow_spi_write_register(REGPREAMBLEMSB_FSK, data, 2, &device->spi_device);
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_get_preamble_length(sx127x *device, uint16_t *value) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    ERROR_CHECK(sx127x_shadow_spi_read_registers(REGPREAMBLEMSB, &device->spi_device, 2, (uint32_t *) value));
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    ERROR_CHECK(sx127x_shadow_spi_read_registers(REGPREAMBLEMSB_FSK, &device->spi_device, 2, (uint32_t *) value));
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  return SX127X_OK;
}

int sx127x_lora_set_implicit_header(sx127x_implicit_header_t *header, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (device->chip_version == SX1276_VERSION) {
    if (header == NULL) {
      device->expected_packet_length = 0;
      device->use_implicit_header = false;
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, 0b00000000, 0b11111110, &device->spi_device));
    } else {
      device->expected_packet_length = header->length;
      device->use_implicit_header = true;
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, 0b00000001 | sx1276_cr[header->coding_rate], 0b11110000, &device->spi_device));
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGPAYLOADLENGTH, &header->length, 1, &device->spi_device));
      uint8_t value = (header->enable_crc ? 0b00000100 : 0b00000000);
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG2, value, 0b11111011, &device->spi_device));
    }
  } else {
    if (header == NULL) {
      device->expected_packet_length = 0;
      device->use_implicit_header = false;
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, 0b00000000, 0b11111011, &device->spi_device));
    } else {
      device->expected_packet_length = header->length;
      device->use_implicit_header = true;
      ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, 0b00000100 | sx1272_cr[header->coding_rate] | (header->enable_crc ? 0b00000010 : 0b00000000), 0b11000001, &device->spi_device));
      ERROR_CHECK(sx127x_shadow_spi_write_register(REGPAYLOADLENGTH, &header->length, 1, &device->spi_device));
    }
  }

  return SX127X_OK;
}

int sx127x_lora_get_implicit_header(sx127x *device, sx127x_implicit_header_t *header, bool *enabled) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t raw;
  if (device->chip_version == SX1276_VERSION) {
    ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG1, &device->spi_device, &raw));
    *enabled = (raw & 0b00000001) > 0;
    if (*enabled) {
      header->coding_rate = ((raw & 0b1110) >> 1) - 1;
      ERROR_CHECK(sx127x_read_register(REGPAYLOADLENGTH, &device->spi_device, &header->length));
      ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG2, &device->spi_device, &raw));
      header->enable_crc = (raw & 0b00000100) > 0;
    }
  } else {
    ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG1, &device->spi_device, &raw));
    *enabled = (raw & 0b00000100) > 0;
    if (*enabled) {
      header->coding_rate = ((raw & 0b111000) >> 3) - 1;
      header->enable_crc = (raw & 0b10) > 0;
      ERROR_CHECK(sx127x_read_register(REGPAYLOADLENGTH, &device->spi_device, &header->length));
    }
  }
  return SX127X_OK;
}

int sx127x_lora_set_frequency_hopping(uint8_t period, uint64_t *frequencies, uint8_t frequencies_length, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (period != 0 && (frequencies == NULL || frequencies_length == 0)) {
    return SX127X_ERR_INVALID_ARG;
  }
  device->frequencies = frequencies;
  device->frequencies_length = frequencies_length;
  return sx127x_shadow_spi_write_register(REGHOPPERIOD, &period, 1, &device->spi_device);
}

int sx127x_lora_get_frequency_hopping(sx127x *device, uint8_t *period) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  ERROR_CHECK(sx127x_read_register(REGHOPPERIOD, &device->spi_device, period));
  return SX127X_OK;
}

int sx127x_rx_get_packet_rssi(sx127x *device, int16_t *rssi) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    uint8_t value;
    ERROR_CHECK(sx127x_read_register(REGPKTRSSIVALUE, &device->spi_device, &value));
    uint64_t frequency;
    ERROR_CHECK(sx127x_get_frequency(device, &frequency));
    if (frequency < RF_MID_BAND_THRESHOLD) {
      *rssi = value - RSSI_OFFSET_LF_PORT;
    } else {
      *rssi = value - RSSI_OFFSET_HF_PORT;
    }
    // section 5.5.5.
    float snr;
    int code = sx127x_lora_rx_get_packet_snr(device, &snr);
    // if snr failed then rssi is not precise
    if (code == SX127X_OK && snr < 0) {
      *rssi = *rssi + snr;
    }
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    if (!device->fsk_rssi_available) {
      *rssi = 0;
      return SX127X_ERR_NOT_FOUND;
    }
    *rssi = device->fsk_rssi;
    // reset internal rssi storage
    device->fsk_rssi = 0;
    device->fsk_rssi_available = false;
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  return SX127X_OK;
}

int sx127x_lora_rx_get_packet_snr(sx127x *device, float *snr) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint8_t value;
  ERROR_CHECK(sx127x_read_register(REGPKTSNRVALUE, &device->spi_device, &value));
  *snr = (float) ((int8_t) value) * 0.25f;
  return SX127X_OK;
}

int sx127x_rx_get_frequency_error(sx127x *device, int32_t *result) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    uint32_t frequency_error;
    ERROR_CHECK(sx127x_shadow_spi_read_registers(REGFEIMSB, &device->spi_device, 3, &frequency_error));
    sx127x_bw_t bandwidth_enum;
    ERROR_CHECK(sx127x_lora_get_bandwidth(device, &bandwidth_enum));
    uint32_t bandwidth = sx127x_bandwidth_to_hz(bandwidth_enum);
    if (frequency_error & 0x80000) {
      // keep within original 2.5 bytes
      frequency_error = ((~frequency_error) + 1) & 0xFFFFF;
      *result = -1;
    } else {
      *result = 1;
    }
    *result = (*result) * (frequency_error * SX127X_FREQ_ERROR_FACTOR * bandwidth / 500000.0f);
    return SX127X_OK;
  } else if (device->active_modem == SX127X_MODULATION_FSK || device->active_modem == SX127X_MODULATION_OOK) {
    uint32_t frequency_error;
    // for some reason register FEI always contains 0
    ERROR_CHECK(sx127x_shadow_spi_read_registers(REGAFCMSB, &device->spi_device, 2, &frequency_error));
    if (frequency_error & 0x8000) {
      // keep within original 2 bytes
      frequency_error = ((~frequency_error) + 1) & 0xFFFF;
      *result = -1;
    } else {
      *result = 1;
    }
    *result = (*result) * SX127X_FSTEP * frequency_error;
    return SX127X_OK;
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
}

int sx127x_dump_registers(uint8_t *output, sx127x *device) {
  // Reading from 0x00 register will actually read from fifo
  // skip it
  output[0] = 0x00;
  // bypass shadow registers
  return sx127x_spi_read_buffer(0x01, output + 1, MAX_NUMBER_OF_REGISTERS - 1, device->spi_device.spi_device);
}

void sx127x_reset(sx127x *device) {
  device->fsk_ook_format = SX127X_VARIABLE;
  device->fsk_rssi_available = false;
  device->fsk_crc_type = SX127X_CRC_CCITT;
  device->use_implicit_header = false;
  device->expected_packet_length = 0;

#ifndef CONFIG_SX127X_DISABLE_SPI_CACHE
  for (size_t i = 0; i < MAX_NUMBER_OF_REGISTERS; i++) {
    if (device->spi_device.shadow_registers_sync[i] == SHADOW_CACHED) {
      device->spi_device.shadow_registers_sync[i] = SHADOW_EMPTY;
    }
  }
#endif
}

void sx127x_tx_set_callback(void (*tx_callback)(void *), void *ctx, sx127x *device) {
  device->tx_callback = tx_callback;
  device->tx_callback_ctx = ctx;
}

int sx127x_tx_set_pa_config(sx127x_pa_pin_t pin, int power, sx127x *device) {
  if (pin == SX127X_PA_PIN_RFO && (power < -4 || power > 15)) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (pin == SX127X_PA_PIN_BOOST && (power < 2 || power > 20 || power == 18 || power == 19)) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t data[] = {0};
  if (pin == SX127X_PA_PIN_BOOST && power == 20) {
    data[0] = SX127X_HIGH_POWER_ON;
  } else {
    data[0] = SX127X_HIGH_POWER_OFF;
  }
  int reg = device->chip_version == SX1276_VERSION ? SX1276_REGPADAC : SX1272_REGPADAC;
  ERROR_CHECK(sx127x_shadow_spi_write_register(reg, data, 1, &device->spi_device));
  uint8_t max_current;
  // according to 2.5.1. Power Consumption
  if (pin == SX127X_PA_PIN_BOOST) {
    if (power == 20) {
      max_current = 120;
    } else {
      max_current = 87;
    }
  } else {
    max_current = 45;
  }
  ERROR_CHECK(sx127x_tx_set_ocp(true, max_current, device));
  uint8_t value;
  if (pin == SX127X_PA_PIN_RFO) {
    if (power < 0) {
      value = SX127X_LOW_POWER | (power + 4);
    } else {
      value = SX127X_MAX_POWER | power;
    }
    value = value | SX127X_PA_PIN_RFO;
  } else {
    if (power == 20) {
      value = SX127X_PA_PIN_BOOST | 0b00001111;
    } else {
      value = SX127X_PA_PIN_BOOST | (power - 2);
    }
  }
  return sx127x_shadow_spi_write_register(REGPACONFIG, &value, 1, &device->spi_device);
}

int sx127x_tx_get_pa_config(sx127x *device, sx127x_pa_pin_t *pin, int *power) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPACONFIG, &device->spi_device, &raw));
  *pin = raw & 0b10000000;
  uint8_t max_power = (raw & 0b1110000) >> 4;
  uint8_t output_power = (raw & 0b1111);
  if (*pin == SX127X_PA_PIN_BOOST) {
    int reg = device->chip_version == SX1276_VERSION ? SX1276_REGPADAC : SX1272_REGPADAC;
    ERROR_CHECK(sx127x_read_register(reg, &device->spi_device, &raw));
    if (raw == SX127X_HIGH_POWER_ON) {
      *power = 20;
    } else {
      *power = 17 - (15 - output_power);
    }
  } else {
    *power = (int) (10.8 + 0.6 * max_power - (15 - output_power));
  }
  return SX127X_OK;
}

int sx127x_tx_set_ocp(bool enable, uint8_t max_current, sx127x *device) {
  if (!enable) {
    uint8_t value = 0b00000000;
    return sx127x_shadow_spi_write_register(REGOCP, &value, 1, &device->spi_device);
  }
  if (max_current < 45) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value;
  // 5.4.4. Over Current Protection
  if (max_current <= 120) {
    value = (max_current - 45) / 5;
  } else if (max_current <= 240) {
    value = (max_current + 30) / 10;
  } else {
    value = 27;
  }
  value |= 0b00100000;
  return sx127x_shadow_spi_write_register(REGOCP, &value, 1, &device->spi_device);
}

int sx127x_tx_get_ocp(sx127x *device, bool *enable, uint8_t *milliamps) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGOCP, &device->spi_device, &raw));
  *enable = (raw & 0b00100000) > 0;
  uint8_t ocp_trim = raw & 0b11111;
  if (*enable) {
    if (ocp_trim <= 15) {
      *milliamps = 45 + 5 * ocp_trim;
    } else if (ocp_trim <= 27) {
      *milliamps = -30 + 10 * ocp_trim;
    } else {
      *milliamps = 240;
    }
  }
  return SX127X_OK;
}

int sx127x_lora_tx_set_explicit_header(sx127x_tx_header_t *header, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  if (header == NULL) {
    return SX127X_ERR_INVALID_ARG;
  }
  device->use_implicit_header = false;
  device->expected_packet_length = 0;
  if (device->chip_version == SX1276_VERSION) {
    ERROR_CHECK(sx127x_append_register(REGMODEMCONFIG1, sx1276_cr[header->coding_rate] | 0b00000000, 0b11110000, &device->spi_device));
    uint8_t value = (header->enable_crc ? 0b00000100 : 0b00000000);
    return sx127x_append_register(REGMODEMCONFIG2, value, 0b11111011, &device->spi_device);
  } else {
    uint8_t value = (header->enable_crc ? 0b00000010 : 0b00000000);
    return sx127x_append_register(REGMODEMCONFIG1, sx1272_cr[header->coding_rate] | 0b00000000 | value, 0b11000001, &device->spi_device);
  }
}

int sx127x_lora_tx_get_explicit_header(sx127x *device, bool *enabled, sx127x_tx_header_t *header) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG1, &device->spi_device, &raw));
  if (device->chip_version == SX1276_VERSION) {
    *enabled = (raw & 0b00000001) == 0;
    if (*enabled) {
      header->coding_rate = ((raw & 0b1110) >> 1) - 1;
      ERROR_CHECK(sx127x_read_register(REGMODEMCONFIG2, &device->spi_device, &raw));
      header->enable_crc = (raw & 0b00000100) > 0;
    }
  } else {
    *enabled = (raw & 0b00000100) == 0;
    if (*enabled) {
      header->coding_rate = ((raw & 0b111000) >> 3) - 1;
      header->enable_crc = (raw & 0b10) > 0;
    }
  }
  return SX127X_OK;
}

int sx127x_lora_tx_set_for_transmission(const uint8_t *data, uint8_t data_length, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  // uint8_t can't be more than MAX_PACKET_SIZE
  if (data_length == 0) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t fifo_addr[] = {FIFO_TX_BASE_ADDR};
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGFIFOADDRPTR, fifo_addr, 1, &device->spi_device));
  uint8_t reg_data[] = {data_length};
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGPAYLOADLENGTH, reg_data, 1, &device->spi_device));
  return sx127x_shadow_spi_write_buffer(REGFIFO, data, data_length, &device->spi_device);
}

int sx127x_lora_set_ppm_offset(int32_t frequency_error, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint64_t frequency;
  ERROR_CHECK(sx127x_get_frequency(device, &frequency));
  uint8_t value = (uint8_t) (0.95f * ((float) frequency_error / ((double) frequency / 1E6)));
  return sx127x_shadow_spi_write_register(0x27, &value, 1, &device->spi_device);
}

int sx127x_lora_get_ppm_offset(sx127x *device, int32_t *frequency_error) {
  CHECK_MODULATION(device, SX127X_MODULATION_LORA);
  uint64_t frequency;
  ERROR_CHECK(sx127x_get_frequency(device, &frequency));
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(0x27, &device->spi_device, &raw));
  *frequency_error = (int32_t) (((double) frequency * (double) raw) / (0.95 * 1E6));
  return SX127X_OK;
}

static int sx127x_fsk_ook_tx_set_for_transmission_with_remaining(uint16_t data_length, sx127x *device) {
  uint8_t to_send;
  if (data_length > FIFO_SIZE_FSK) {
    to_send = FIFO_SIZE_FSK;
  } else {
    to_send = data_length;
  }
  device->expected_packet_length = data_length;
  device->fsk_ook_packet_sent_received = to_send;
  return sx127x_shadow_spi_write_buffer(REGFIFO, device->packet, to_send, &device->spi_device);
}

int sx127x_fsk_ook_tx_set_for_transmission(const uint8_t *data, uint16_t data_length, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (device->fsk_ook_format == SX127X_VARIABLE && data_length > MAX_PACKET_SIZE) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (device->fsk_ook_format == SX127X_FIXED && data_length > MAX_PACKET_SIZE_FSK_FIXED) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (device->fsk_ook_format == SX127X_VARIABLE) {
    device->packet[0] = (uint8_t) data_length;
    // packet length is always more than 255
    memcpy(device->packet + 1, data, sizeof(uint8_t) * data_length);
    data_length++;
  } else {
    memcpy(device->packet, data, sizeof(uint8_t) * data_length);
  }
  return sx127x_fsk_ook_tx_set_for_transmission_with_remaining(data_length, device);
}

int sx127x_fsk_ook_tx_set_for_transmission_with_address(const uint8_t *data, uint16_t data_length, uint8_t address_to, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (device->fsk_ook_format == SX127X_VARIABLE && data_length > (MAX_PACKET_SIZE - 1)) {
    return SX127X_ERR_INVALID_ARG;
  }
  if (device->fsk_ook_format == SX127X_FIXED && data_length > (MAX_PACKET_SIZE_FSK_FIXED - 1)) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint16_t offset = 0;
  uint16_t packet_length = data_length + 1;
  if (device->fsk_ook_format == SX127X_VARIABLE) {
    device->packet[offset] = (uint8_t) packet_length;
    offset++;
    packet_length++;
  }
  device->packet[offset] = address_to;
  offset++;
  memcpy(device->packet + offset, data, sizeof(uint8_t) * data_length);
  return sx127x_fsk_ook_tx_set_for_transmission_with_remaining(packet_length, device);
}

int sx127x_fsk_ook_tx_start_beacon(const uint8_t *data, uint8_t data_length, uint32_t interval_ms, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (device->fsk_ook_format != SX127X_FIXED) {
    return SX127X_ERR_INVALID_STATE;
  }
  if (data_length > FIFO_SIZE_FSK) {
    return SX127X_ERR_INVALID_ARG;
  }

  float p1 = 0.064f;
  float p2 = 4.1f;
  float p3 = 262;

  float timer1_resolution = 0.0f;
  uint8_t timer1_coefficient = 0;
  float timer2_resolution = 0.0f;
  uint8_t timer2_coefficient = 0;

  if (interval_ms <= 255 * p1 * 2) {
    timer1_resolution = p1;
    timer2_resolution = p1;
    timer1_coefficient = (uint8_t) (interval_ms / p1 / 2);
  } else if (interval_ms <= (255 * p2 + 255 * p1)) {
    timer1_resolution = p2;
    timer2_resolution = p1;
    timer1_coefficient = (uint8_t) (interval_ms / p2);
  } else if (interval_ms <= 255 * p2 * 2) {
    timer1_resolution = p2;
    timer2_resolution = p2;
    timer1_coefficient = (uint8_t) (interval_ms / p2 / 2);
  } else if (interval_ms <= (255 * p3 + 255 * p1)) {
    timer1_resolution = p3;
    timer2_resolution = p1;
    timer1_coefficient = (uint8_t) (interval_ms / p3);
  } else if (interval_ms <= (255 * p3 + 255 * p2)) {
    timer1_resolution = p3;
    timer2_resolution = p2;
    timer1_coefficient = (uint8_t) (interval_ms / p3);
  } else {
    timer1_resolution = p3;
    timer2_resolution = p3;
    timer1_coefficient = (uint8_t) (interval_ms / p3 / 2);
  }
  timer2_coefficient = (uint8_t) ((interval_ms - timer1_resolution * timer1_coefficient) / timer2_resolution);

  uint8_t timer_resolution = 0b00000000;
  if (timer1_resolution == p1) {
    timer_resolution = 0b00000100;
  } else if (timer1_resolution == p2) {
    timer_resolution = 0b00001000;
  } else {
    timer_resolution = 0b00001100;
  }
  if (timer2_resolution == p1) {
    timer_resolution += 0b00000001;
  } else if (timer2_resolution == p2) {
    timer_resolution += 0b00000010;
  } else {
    timer_resolution += 0b00000011;
  }
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGTIMER1COEF, &timer1_coefficient, 1, &device->spi_device));
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGTIMER2COEF, &timer2_coefficient, 1, &device->spi_device));
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGTIMERRESOL, &timer_resolution, 1, &device->spi_device));

  // start tx as soon as first byte in FIFO available
  uint8_t value = (0b10000000 | HALF_MAX_FIFO_THRESHOLD);
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGFIFOTHRESH, &value, 1, &device->spi_device));
  // reset FIFO if something was there
  value = 0b00010000;
  ERROR_CHECK(sx127x_shadow_spi_write_register(0x3f, &value, 1, &device->spi_device));
  ERROR_CHECK(sx127x_fsk_ook_tx_set_for_transmission(data, data_length, device));
  value = 0b00001000;  // beacon on
  ERROR_CHECK(sx127x_append_register(REGPACKETCONFIG2, value, 0b11110111, &device->spi_device));
  // start sequencer
  value = 0b10100100;
  return sx127x_shadow_spi_write_register(REGSEQCONFIG1, &value, 1, &device->spi_device);
}

int sx127x_fsk_ook_tx_stop_beacon(sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  // stop sequencer
  uint8_t value = 0b01000000;
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGSEQCONFIG1, &value, 1, &device->spi_device));
  value = 0b00010000;
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGIRQFLAGS2, &value, 1, &device->spi_device));
  value = 0b00000000;  // beacon off
  return sx127x_append_register(REGPACKETCONFIG2, value, 0b11110111, &device->spi_device);
}

void sx127x_lora_cad_set_callback(void (*cad_callback)(void *, int), void *ctx, sx127x *device) {
  device->cad_callback = cad_callback;
  device->cad_callback_ctx = ctx;
}

int sx127x_fsk_ook_set_bitrate(float bitrate, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  uint16_t bitrate_value;
  uint8_t bitrate_fractional;
  if (device->active_modem == SX127X_MODULATION_FSK) {
    if (bitrate < 1200 || bitrate > 300000) {
      return SX127X_ERR_INVALID_ARG;
    }
    uint32_t value = (uint32_t) (SX127X_OSCILLATOR_FREQUENCY * 16.0 / bitrate);
    bitrate_value = (value >> 4) & 0xFFFF;
    bitrate_fractional = value & 0x0F;
  } else if (device->active_modem == SX127X_MODULATION_OOK) {
    if (bitrate < 1200 || bitrate > 25000) {
      return SX127X_ERR_INVALID_ARG;
    }
    bitrate_value = (uint16_t) (SX127X_OSCILLATOR_FREQUENCY / bitrate);
    bitrate_fractional = 0;
  } else {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t data[] = {(uint8_t) (bitrate_value >> 8), (uint8_t) (bitrate_value >> 0)};
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGBITRATEMSB, data, 2, &device->spi_device));
  int reg = device->chip_version == SX1276_VERSION ? SX1276_REGBITRATEFRAC : SX1272_REGBITRATEFRAC;
  return sx127x_shadow_spi_write_register(reg, &bitrate_fractional, 1, &device->spi_device);
}

int sx127x_fsk_ook_get_bitrate(sx127x *device, float *bitrate) {
  if (device->active_modem == SX127X_MODULATION_LORA) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint32_t bitrate_raw;
  ERROR_CHECK(sx127x_shadow_spi_read_registers(REGBITRATEMSB, &device->spi_device, 2, &bitrate_raw));
  int reg = device->chip_version == SX1276_VERSION ? SX1276_REGBITRATEFRAC : SX1272_REGBITRATEFRAC;

  float value = (float) bitrate_raw;
  if (device->active_modem == SX127X_MODULATION_FSK) {
    uint32_t bitrate_fractional;
    ERROR_CHECK(sx127x_shadow_spi_read_registers(reg, &device->spi_device, 1, &bitrate_fractional));
    value += (float) bitrate_fractional / 16.0f;
  }
  *bitrate = SX127X_OSCILLATOR_FREQUENCY / value;
  return SX127X_OK;
}

int sx127x_fsk_set_fdev(float frequency_deviation, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_FSK);
  if (frequency_deviation < 600 || frequency_deviation > 200000) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint16_t value = (uint16_t) (frequency_deviation / SX127X_FSTEP);
  uint8_t data[] = {(uint8_t) (value >> 8), (uint8_t) (value >> 0)};
  return sx127x_shadow_spi_write_register(REGFDEVMSB, data, 2, &device->spi_device);
}

int sx127x_fsk_get_fdev(sx127x *device, float *frequency_deviation) {
  CHECK_MODULATION(device, SX127X_MODULATION_FSK);
  uint32_t raw;
  ERROR_CHECK(sx127x_shadow_spi_read_registers(REGFDEVMSB, &device->spi_device, 2, &raw));
  *frequency_deviation = (float) raw * SX127X_FSTEP;
  return SX127X_OK;
}

int sx127x_ook_get_ook_thresh_type(sx127x *device, sx127x_ook_thresh_type_t *type) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGOOKPEAK, &device->spi_device, &raw));
  *type = raw & 0b00011000;
  return SX127X_OK;
}

int sx127x_ook_rx_set_peak_mode(sx127x_ook_peak_thresh_step_t step, uint8_t floor_threshold, sx127x_ook_peak_thresh_dec_t decrement, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGOOKFIX, &floor_threshold, 1, &device->spi_device));
  ERROR_CHECK(sx127x_append_register(REGOOKAVG, decrement, 0b00011111, &device->spi_device));
  return sx127x_append_register(REGOOKPEAK, (0b00001000 | step), 0b11100000, &device->spi_device);
}

int sx127x_ook_rx_get_peak_mode(sx127x *device, sx127x_ook_peak_thresh_step_t *step, uint8_t *floor_threshold, sx127x_ook_peak_thresh_dec_t *decrement) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  ERROR_CHECK(sx127x_read_register(REGOOKFIX, &device->spi_device, floor_threshold));
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGOOKAVG, &device->spi_device, &raw));
  *decrement = raw & 0b11100000;
  ERROR_CHECK(sx127x_read_register(REGOOKPEAK, &device->spi_device, &raw));
  *step = raw & 0b111;
  return SX127X_OK;
}

int sx127x_ook_rx_set_fixed_mode(uint8_t fixed_threshold, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGOOKFIX, &fixed_threshold, 1, &device->spi_device));
  return sx127x_append_register(REGOOKPEAK, 0b00000000, 0b11100111, &device->spi_device);
}

int sx127x_ook_rx_get_fixed_mode(sx127x *device, uint8_t *fixed_threshold) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  ERROR_CHECK(sx127x_read_register(REGOOKFIX, &device->spi_device, fixed_threshold));
  return SX127X_OK;
}

int sx127x_ook_rx_set_avg_mode(sx127x_ook_avg_offset_t avg_offset, sx127x_ook_avg_thresh_t avg_thresh, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  ERROR_CHECK(sx127x_append_register(REGOOKAVG, (avg_offset | avg_thresh), 0b11110000, &device->spi_device));
  return sx127x_append_register(REGOOKPEAK, 0b00010000, 0b11100111, &device->spi_device);
}

int sx127x_ook_rx_get_avg_mode(sx127x *device, sx127x_ook_avg_offset_t *avg_offset, sx127x_ook_avg_thresh_t *avg_thresh) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGOOKAVG, &device->spi_device, &raw));
  *avg_offset = raw & 0b1100;
  *avg_thresh = raw & 0b0011;
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_collision_restart(bool enable, uint8_t threshold, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGRSSICOLLISION, &threshold, 1, &device->spi_device));
  uint8_t value = (enable ? 0b10000000 : 0b00000000);
  return sx127x_append_register(REGRXCONFIG, value, 0b01111111, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_collision_restart(sx127x *device, bool *enable, uint8_t *threshold) {
  CHECK_FSK_OOK_MODULATION(device);
  ERROR_CHECK(sx127x_read_register(REGRSSICOLLISION, &device->spi_device, threshold));
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGRXCONFIG, &device->spi_device, &raw));
  *enable = (raw & 0b10000000) > 0;
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_afc_auto(bool afc_auto, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t value = (afc_auto ? 0b00010000 : 0b00000000);
  return sx127x_append_register(REGRXCONFIG, value, 0b11101111, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_afc_auto(sx127x *device, bool *afc_auto) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGRXCONFIG, &device->spi_device, &raw));
  *afc_auto = (raw & 0b00010000) > 0;
  return SX127X_OK;
}

static uint8_t sx127x_fsk_ook_calculate_bw_register(float bandwidth) {
  float min_tolerance = bandwidth;
  uint8_t result = 0;
  for (uint8_t e = 7; e >= 1; e--) {
    for (int8_t m = 2; m >= 0; m--) {
      float point = SX127X_OSCILLATOR_FREQUENCY / (float) (((4 * m) + 16) * ((uint32_t) 1 << (e + 2)));
      float current_tolerance = fabsf(bandwidth - point);
      if (current_tolerance < min_tolerance) {
        result = ((m << 3) | e);
        min_tolerance = current_tolerance;
      }
    }
  }
  return result;
}

static float sx127x_fsk_ook_calculate_bw(uint8_t value) {
  uint8_t mantissa;
  switch (((value & 0b11000) >> 3)) {
    case 0b10:
      mantissa = 24;
      break;
    case 0b01:
      mantissa = 20;
      break;
    case 0b00:
      mantissa = 16;
      break;
    case 0b11:
    default:
      // invalid - should fail
      mantissa = 0;
      break;
  }
  return SX127X_OSCILLATOR_FREQUENCY / (mantissa * ((uint32_t) 1 << ((value & 0b111) + 2)));
}

int sx127x_fsk_ook_rx_set_afc_bandwidth(float bandwidth, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t value = sx127x_fsk_ook_calculate_bw_register(bandwidth);
  return sx127x_shadow_spi_write_register(REGAFCBW, &value, 1, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_afc_bandwidth(sx127x *device, float *bandwidth) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGAFCBW, &device->spi_device, &raw));
  *bandwidth = sx127x_fsk_ook_calculate_bw(raw);
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_bandwidth(float bandwidth, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t value = sx127x_fsk_ook_calculate_bw_register(bandwidth);
  return sx127x_shadow_spi_write_register(REGRXBW, &value, 1, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_bandwidth(sx127x *device, float *bandwidth) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGRXBW, &device->spi_device, &raw));
  *bandwidth = sx127x_fsk_ook_calculate_bw(raw);
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_trigger(sx127x_rx_trigger_t trigger, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  return sx127x_append_register(REGRXCONFIG, trigger, 0b11111000, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_trigger(sx127x *device, sx127x_rx_trigger_t *trigger) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGRXCONFIG, &device->spi_device, &raw));
  *trigger = raw & 0b111;
  return SX127X_OK;
}

int sx127x_fsk_ook_set_syncword(const uint8_t *syncword, uint8_t syncword_length, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (syncword_length == 0 || syncword_length > 8) {
    return SX127X_ERR_INVALID_ARG;
  }
  for (uint8_t i = 0; i < syncword_length; i++) {
    if (syncword[i] == 0x00) {
      return SX127X_ERR_INVALID_ARG;
    }
  }
  // SYNC_ON + On, without waiting for the PLL to re-lock
  ERROR_CHECK(sx127x_append_register(REGSYNCCONFIG, 0b01010000 | (syncword_length - 1), 0b00101000, &device->spi_device));
  return sx127x_shadow_spi_write_buffer(REGSYNCVALUE1, syncword, syncword_length, &device->spi_device);
}

int sx127x_fsk_ook_get_syncword(sx127x *device, uint8_t *syncword, uint8_t *syncword_length) {
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGSYNCCONFIG, &device->spi_device, &raw));
  *syncword_length = (raw & 0b111) + 1;
  ERROR_CHECK(sx127x_shadow_spi_read_buffer(REGSYNCVALUE1, syncword, *syncword_length, &device->spi_device));
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_rssi_config(sx127x_rssi_smoothing_t smoothing, int8_t offset, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (offset < -16 || offset > 15) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value = (offset << 3) | smoothing;
  return sx127x_shadow_spi_write_register(REGRSSICONFIG, &value, 1, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_rssi_config(sx127x *device, sx127x_rssi_smoothing_t *smoothing, int8_t *offset) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGRSSICONFIG, &device->spi_device, &raw));
  *smoothing = raw & 0b111;
  *offset = (int8_t) (raw >> 3);
  return SX127X_OK;
}

int sx127x_fsk_ook_set_packet_encoding(sx127x_packet_encoding_t encoding, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  return sx127x_append_register(REGPACKETCONFIG1, encoding, 0b10011111, &device->spi_device);
}

int sx127x_fsk_ook_get_packet_encoding(sx127x *device, sx127x_packet_encoding_t *encoding) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPACKETCONFIG1, &device->spi_device, &raw));
  *encoding = raw & 0b01100000;
  return SX127X_OK;
}

int sx127x_fsk_ook_set_crc(sx127x_crc_type_t crc_type, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  //  + Do not clear FIFO
  int result = sx127x_append_register(REGPACKETCONFIG1, (uint8_t) crc_type | 0b00001000, 0b11100110, &device->spi_device);
  if (result == SX127X_OK) {
    device->fsk_crc_type = crc_type;
  }
  return result;
}

int sx127x_fsk_ook_get_crc(sx127x *device, sx127x_crc_type_t *crc_type) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPACKETCONFIG1, &device->spi_device, &raw));
  *crc_type = raw & 0b10001;
  return SX127X_OK;
}

int sx127x_fsk_ook_set_packet_format(sx127x_packet_format_t format, uint16_t max_payload_length, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (format == SX127X_FIXED && (max_payload_length == 0 || max_payload_length > MAX_PACKET_SIZE_FSK_FIXED)) {
    return SX127X_ERR_INVALID_ARG;
  }
  // max_payload_length = 2047 in variable packet mode will disable max payload length check
  if (format == SX127X_VARIABLE && (max_payload_length == 0 || (max_payload_length > MAX_PACKET_SIZE && max_payload_length != MAX_PACKET_SIZE_FSK_FIXED))) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t msb_bits = ((max_payload_length >> 8) & 0b111);
  ERROR_CHECK(sx127x_append_register(REGPACKETCONFIG2, msb_bits, 0b11111000, &device->spi_device));
  uint8_t lsb_bits = (max_payload_length & 0xFF);
  ERROR_CHECK(sx127x_shadow_spi_write_register(REGPAYLOADLENGTH_FSK, &lsb_bits, 1, &device->spi_device));
  ERROR_CHECK(sx127x_append_register(REGPACKETCONFIG1, format, 0b01111111, &device->spi_device));
  device->fsk_ook_format = format;
  return SX127X_OK;
}

int sx127x_fsk_ook_get_packet_format(sx127x *device, sx127x_packet_format_t *format, uint16_t *max_payload_length) {
  CHECK_FSK_OOK_MODULATION(device);
  uint32_t raw;
  ERROR_CHECK(sx127x_shadow_spi_read_registers(REGPACKETCONFIG1, &device->spi_device, 3, &raw));
  *format = (raw >> 16) & 0b10000000;
  *max_payload_length = raw & 0x7FF; //11 bit
  return SX127X_OK;
}

int sx127x_fsk_ook_set_address_filtering(sx127x_address_filtering_t type, uint8_t node_address, uint8_t broadcast_address, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (type == SX127X_FILTER_NODE_AND_BROADCAST) {
    ERROR_CHECK(sx127x_shadow_spi_write_register(REGBROADCASTADRS, &broadcast_address, 1, &device->spi_device));
  }
  if (type == SX127X_FILTER_NODE_AND_BROADCAST || type == SX127X_FILTER_NODE_ADDRESS) {
    ERROR_CHECK(sx127x_shadow_spi_write_register(REGNODEADRS, &node_address, 1, &device->spi_device));
  }
  return sx127x_append_register(REGPACKETCONFIG1, type, 0b11111001, &device->spi_device);
}

int sx127x_fsk_ook_get_address_filtering(sx127x *device, sx127x_address_filtering_t *type, uint8_t *node_address, uint8_t *broadcast_address) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPACKETCONFIG1, &device->spi_device, &raw));
  *type = raw & 0b110;
  if (*type == SX127X_FILTER_NONE) {
    *node_address = 0;
    *broadcast_address = 0;
    return SX127X_OK;
  }
  ERROR_CHECK(sx127x_read_register(REGNODEADRS, &device->spi_device, node_address));
  if (*type == SX127X_FILTER_NODE_AND_BROADCAST) {
    ERROR_CHECK(sx127x_read_register(REGBROADCASTADRS, &device->spi_device, broadcast_address));
  }
  return SX127X_OK;
}

int sx127x_fsk_set_data_shaping(sx127x_fsk_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_FSK);
  uint8_t value = (data_shaping | pa_ramp);
  return sx127x_shadow_spi_write_register(REGPARAMP, &value, 1, &device->spi_device);
}

int sx127x_fsk_get_data_shaping(sx127x *device, sx127x_fsk_data_shaping_t *data_shaping, sx127x_pa_ramp_t *pa_ramp) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPARAMP, &device->spi_device, &raw));
  *data_shaping = raw & 0b01100000;
  *pa_ramp = raw & 0b1111;
  return SX127X_OK;
}

int sx127x_ook_set_data_shaping(sx127x_ook_data_shaping_t data_shaping, sx127x_pa_ramp_t pa_ramp, sx127x *device) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  uint8_t value = (data_shaping | pa_ramp);
  return sx127x_shadow_spi_write_register(REGPARAMP, &value, 1, &device->spi_device);
}

int sx127x_ook_get_data_shaping(sx127x *device, sx127x_ook_data_shaping_t *data_shaping, sx127x_pa_ramp_t *pa_ramp) {
  CHECK_MODULATION(device, SX127X_MODULATION_OOK);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPARAMP, &device->spi_device, &raw));
  *data_shaping = raw & 0b01100000;
  *pa_ramp = raw & 0b1111;
  return SX127X_OK;
}

int sx127x_fsk_ook_set_preamble_type(sx127x_preamble_type_t type, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  return sx127x_append_register(REGSYNCCONFIG, type, 0b11011111, &device->spi_device);
}

int sx127x_fsk_ook_get_preamble_type(sx127x *device, sx127x_preamble_type_t *type) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGSYNCCONFIG, &device->spi_device, &raw));
  *type = raw & 0b00100000;
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_set_preamble_detector(bool enable, uint8_t detector_size, uint8_t detector_tolerance, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (detector_size > 3 || detector_size < 1) {
    return SX127X_ERR_INVALID_ARG;
  }
  uint8_t value = (enable ? 0b10000000 : 0b00000000);
  value = value | ((detector_size - 1) << 5) | (detector_tolerance & 0b00011111);
  return sx127x_shadow_spi_write_register(REGPREAMBLEDETECT, &value, 1, &device->spi_device);
}

int sx127x_fsk_ook_rx_get_preamble_detector(sx127x *device, bool *enable, uint8_t *detector_size, uint8_t *detector_tolerance) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t raw;
  ERROR_CHECK(sx127x_read_register(REGPREAMBLEDETECT, &device->spi_device, &raw));
  *enable = (raw & 0b10000000) > 0;
  *detector_tolerance = raw & 0b11111;
  *detector_size = ((raw >> 5) & 0b11) + 1;
  return SX127X_OK;
}

int sx127x_fsk_ook_rx_calibrate(sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  if (device->opmod != SX127X_MODE_STANDBY) {
    return SX127X_ERR_INVALID_STATE;
  }

  uint8_t value = 0b01000000;  // ImageCalStart
  ERROR_CHECK(sx127x_append_register(REGIMAGECAL, value, 0b10111111, &device->spi_device));

  uint8_t calibration_running = 0b00100000;  // ImageCalRunning
  do {
    ERROR_CHECK(sx127x_read_register(REGIMAGECAL, &device->spi_device, &value));
  } while ((value & calibration_running) == calibration_running);
  return SX127X_OK;
}

int sx127x_fsk_ook_get_raw_temperature(sx127x *device, int8_t *raw_temperature) {
  CHECK_FSK_OOK_MODULATION(device);
  uint8_t value;
  ERROR_CHECK(sx127x_read_register(REGTEMP, &device->spi_device, &value));
  if ((value & 0x80) == 0x80) {
    *raw_temperature = 255 - value;
  } else {
    *raw_temperature = -1 * value;
  }
  return SX127X_OK;
}

int sx127x_fsk_ook_set_temp_monitor(bool enable, sx127x *device) {
  CHECK_FSK_OOK_MODULATION(device);
  // the field is called TempMonitorOff, thus inverted
  uint8_t value = (enable ? 0b00000000 : 0b00000001);
  return sx127x_append_register(REGIMAGECAL, value, 0b11111110, &device->spi_device);
}
