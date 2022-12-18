#include "sx1276.h"

#include <errno.h>
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
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d

#define SX1276_VERSION 0x12

#define SX1276_FSK_OOK 0b00000000                //  7     7     FSK/OOK mode
#define SX1276_LORA 0b10000000                   //  7     7     LoRa mode
#define SX1276_ACCESS_SHARED_REG_OFF 0b00000000  //  6     6     access LoRa registers (0x0D:0x3F) in LoRa mode
#define SX1276_ACCESS_SHARED_REG_ON 0b01000000   //  6     6     access FSK registers (0x0D:0x3F) in LoRa mode
#define SX1276_SLEEP 0b00000000                  //  2     0     sleep
#define SX1276_STANDBY 0b00000001                //  2     0     standby
#define SX1276_FSTX 0b00000010                   //  2     0     frequency synthesis TX
#define SX1276_TX 0b00000011                     //  2     0     transmit
#define SX1276_FSRX 0b00000100                   //  2     0     frequency synthesis RX
#define SX1276_RXCONTINUOUS 0b00000101           //  2     0     receive continuous

#define SX1276_OSCILLATOR_FREQUENCY 32000000
#define SX1276_REG_MODEM_CONFIG_3_AGC_ON 0b00000100
#define SX1276_REG_MODEM_CONFIG_3_AGC_OFF 0b00000000

#define SX1276_IRQ_FLAG_RXTIMEOUT 0b10000000
#define SX1276_IRQ_FLAG_RXDONE 0b01000000
#define SX1276_IRQ_FLAG_PAYLOAD_CRC_ERROR 0b00100000
#define SX1276_IRQ_FLAG_VALID_HEADER 0b00010000
#define SX1276_IRQ_FLAG_TXDONE 0b00001000
#define SX1276_IRQ_FLAG_CADDONE 0b00000100
#define SX1276_IRQ_FLAG_FHSSCHANGECHANNEL 0b00000010
#define SX1276_IRQ_FLAG_CAD_DETECTED 0b00000001

struct sx127x_t {
  spi_device_handle_t spi;
  uint8_t version;
  uint8_t opmode;
  uint8_t header_mode;

  uint8_t irq_received;
};

int sx127x_read_register(int reg, sx127x *device, uint8_t *result) {
  spi_transaction_t t = {
      .addr = reg & 0x7F,
      .rx_buffer = NULL,
      .tx_buffer = NULL,
      .rxlength = 1 * 8,
      .length = 1 * 8,
      .flags = SPI_TRANS_USE_RXDATA};
  esp_err_t code = spi_device_polling_transmit(device->spi, &t);
  if (code != ESP_OK) {
    *result = 0;
    return code;
  }
  *result = t.rx_data[0];
  return 0;
}

int sx127x_write_register(int reg, uint8_t *data, size_t data_length, sx127x *device) {
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

int sx1276_set_low_datarate_optimization(uint8_t value, sx127x *device) {
  uint8_t previous = 0;
  int code = sx127x_read_register(REG_MODEM_CONFIG_3, device, &previous);
  if (code != 0) {
    return code;
  }
  uint8_t data[] = {previous | value};
  return sx127x_write_register(REG_MODEM_CONFIG_3, data, 1, device);
}

int sx1276_reload_low_datarate_optimization(sx127x *device) {
  uint8_t config = 0;
  int code = sx127x_read_register(REG_MODEM_CONFIG_1, device, &config);
  if (code != 0) {
    return code;
  }
  config = (config >> 4);
  long bandwidth = 0;
  switch (config) {
    case 0b0000:
      bandwidth = 7800;
      break;
    case 0b0001:
      bandwidth = 10400;
      break;
    case 0b0010:
      bandwidth = 15600;
      break;
    case 0b0011:
      bandwidth = 20800;
      break;
    case 0b0100:
      bandwidth = 31250;
      break;
    case 0b0101:
      bandwidth = 41700;
      break;
    case 0b0110:
      bandwidth = 62500;
      break;
    case 0b0111:
      bandwidth = 125000;
      break;
    case 0b1000:
      bandwidth = 250000;
      break;
    case 0b1001:
      bandwidth = 500000;
      break;
  }
  config = 0;
  code = sx127x_read_register(REG_MODEM_CONFIG_2, device, &config);
  if (code != 0) {
    return code;
  }
  config = (config >> 4);

  // Section 4.1.1.5
  long symbol_duration = 1000 / (bandwidth / (1L << config));
  if (symbol_duration > 16) {
    return sx1276_set_low_datarate_optimization(SX1276_LOW_DATARATE_OPTIMIZATION_ON, device);
  }
  return 0;
}

int sx127x_create(spi_host_device_t host, int cs, sx127x **result) {
  struct sx127x_t *device = malloc(sizeof(struct sx127x_t));
  if (device == NULL) {
    return -ENOMEM;
  }
  *device = (struct sx127x_t){0};
  device->header_mode = SX1276_HEADER_MODE_EXPLICIT;
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
  if (device->version != SX1276_VERSION) {
    sx127x_destroy(device);
    return SX1276_ERROR_INVALID_CHIP;
  }
  code = sx127x_read_register(REG_OP_MODE, device, &device->opmode);
  if (code != ESP_OK) {
    sx127x_destroy(device);
    return code;
  }
  *result = device;
  return 0;
}

int sx1276_is_lora(bool *result, sx127x *device) {
  *result = (device->opmode & 0b10000000) == SX1276_LORA_MODE_LORA;
  return 0;
}

int sx1276_get_modulation_type(uint8_t *result, sx127x *device) {
  *result = (device->opmode & 0b01100000);
  return 0;
}

int sx1276_get_mode(uint8_t *result, sx127x *device) {
  *result = device->opmode & 0b00000111;
  return 0;
}

int sx1276_set_opmod(uint8_t opmod, sx127x *device) {
  uint8_t data[] = {opmod};
  int code = sx127x_write_register(REG_OP_MODE, data, 1, device);
  if (code != 0) {
    return code;
  }
  device->opmode = opmod;
  return 0;
}

int sx1276_set_frequency(long frequency, sx127x *device) {
  uint64_t adjusted = ((uint64_t)frequency << 19) / SX1276_OSCILLATOR_FREQUENCY;
  uint8_t data[] = {(uint8_t)(adjusted >> 16), (uint8_t)(adjusted >> 8), (uint8_t)(adjusted >> 0)};
  return sx127x_write_register(REG_FRF_MSB, data, 3, device);
}

int sx1276_reset_fifo(sx127x *device) {
  // TODO check if lora modem selected
  //  reset both RX and TX
  uint8_t data[] = {0, 0};
  return sx127x_write_register(REG_FIFO_TX_BASE_ADDR, data, 2, device);
}

int sx1276_set_lna_gain(uint8_t lna, sx127x *device) {
  uint8_t previous = 0;
  int code = sx127x_read_register(REG_MODEM_CONFIG_3, device, &previous);
  if (code != 0) {
    return code;
  }
  if (lna == SX1276_LNA_GAIN_AUTO) {
    uint8_t data[] = {previous | SX1276_REG_MODEM_CONFIG_3_AGC_ON};
    return sx127x_write_register(REG_MODEM_CONFIG_3, data, 1, device);
  } else {
    uint8_t data[] = {previous | SX1276_REG_MODEM_CONFIG_3_AGC_OFF};
    int code = sx127x_write_register(REG_MODEM_CONFIG_3, data, 1, device);
    if (code != 0) {
      return code;
    }
  }
  code = sx127x_read_register(REG_LNA, device, &previous);
  if (code != 0) {
    return code;
  }
  uint8_t data[] = {previous & lna};
  return sx127x_write_register(REG_LNA, data, 1, device);
}

int sx1276_set_lna_boost_hf(uint8_t value, sx127x *device) {
  uint8_t previous = 0;
  int code = sx127x_read_register(REG_LNA, device, &previous);
  if (code != 0) {
    return code;
  }
  uint8_t data[] = {previous & value};
  return sx127x_write_register(REG_LNA, data, 1, device);
}

int sx1276_set_modem_config_1(uint8_t value, sx127x *device) {
  uint8_t data[] = {value};
  int code = sx127x_write_register(REG_MODEM_CONFIG_1, data, 1, device);
  if (code != 0) {
    return code;
  }
  return sx1276_reload_low_datarate_optimization(device);
}

int sx1276_set_modem_config_2(uint8_t value, sx127x *device) {
  uint8_t detection_optimize;
  uint8_t detection_threshold;
  if (value & SX1276_SF_6 == SX1276_SF_6) {
    detection_optimize = 0xc5;
    detection_threshold = 0x0c;
    // make header implicit
  } else {
    detection_optimize = 0xc3;
    detection_threshold = 0x0a;
  }
  uint8_t data[0] = detection_optimize;
  int code = sx127x_write_register(REG_DETECTION_OPTIMIZE, data, 1, device);
  if (code != 0) {
    return code;
  }
  data[0] = detection_threshold;
  code = sx127x_write_register(REG_DETECTION_THRESHOLD, data, 1, device);
  if (code != 0) {
    return code;
  }
  data[0] = value;
  code = sx127x_write_register(REG_MODEM_CONFIG_2, data, 1, device);
  if (code != 0) {
    return code;
  }
  return sx1276_reload_low_datarate_optimization(device);
}

int sx1276_set_syncword(uint8_t value, sx127x *device) {
  uint8_t data[] = {value};
  return sx127x_write_register(REG_SYNC_WORD, data, 1, device);
}

int sx1276_set_preamble_length(uint16_t value, sx127x *device) {
  uint8_t data[] = {(uint8_t)(value >> 8), (uint8_t)(value >> 0)};
  return sx127x_write_register(REG_PREAMBLE_MSB, data, 2, device);
}

void IRAM_ATTR sx1276_handle_interrupt(void *arg) {
  sx127x *device = (sx127x *)arg;
  device->irq_received = 1;
}

int sx1276_receive(sx127x *device) {
  if (!device->irq_received) {
    return 0;
  }
  uint8_t value;
  int code = sx127x_read_register(REG_IRQ_FLAGS, device, &value);
  if (code != 0) {
    return code;
  }
  // clear the irq
  uint8_t data[] = {value};
  code = sx127x_write_register(REG_IRQ_FLAGS, data, 1, device);
  if (code != 0) {
    return code;
  }
  if (value & SX1276_IRQ_FLAG_RXDONE == 0) {
    // ignore non rx-related interrupts
    return 0;
  }

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