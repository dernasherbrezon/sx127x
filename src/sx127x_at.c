#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sx127x.h"
#include "sx127x_at.h"

// Maximum lengths for input and output buffers
#define MAX_INPUT_LEN 256
#define MAX_PARAM_COUNT 10

#define ERROR_CHECK(x)           \
  do {                           \
    int __err_rc = (x);          \
    if (__err_rc != SX127X_OK) { \
      return __err_rc;           \
    }                            \
  } while (0)

static sx127x_packet_encoding_t parse_packet_encoding(const char *str) {
  if (strcmp(str, "MANCHESTER") == 0) return SX127X_MANCHESTER;
  if (strcmp(str, "NRZ") == 0) return SX127X_NRZ;
  if (strcmp(str, "SCRAMBLED") == 0) return SX127X_SCRAMBLED;
  return -1;
}

static const char *format_packet_encoding(sx127x_packet_encoding_t encoding) {
  switch (encoding) {
    case SX127X_MANCHESTER:
      return "MANCHESTER";
    case SX127X_NRZ:
      return "NRZ";
    case SX127X_SCRAMBLED:
      return "SCRAMBLED";
    default:
      return "UNKNOWN";
  }
}

static sx127x_crc_type_t parse_crc(const char *str) {
  if (strcmp(str, "CCITT") == 0) return SX127X_CRC_CCITT;
  if (strcmp(str, "IBM") == 0) return SX127X_CRC_IBM;
  if (strcmp(str, "NONE") == 0) return SX127X_CRC_NONE;
  return -1;
}

static const char *format_crc(sx127x_crc_type_t crc) {
  switch (crc) {
    case SX127X_CRC_CCITT:
      return "CCITT";
    case SX127X_CRC_IBM:
      return "IBM";
    case SX127X_CRC_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_packet_format_t parse_packet_format(const char *str) {
  if (strcmp(str, "FIXED") == 0) return SX127X_FIXED;
  if (strcmp(str, "VARIABLE") == 0) return SX127X_VARIABLE;
  return -1;
}

static const char *format_packet_format(sx127x_packet_format_t format) {
  switch (format) {
    case SX127X_FIXED:
      return "FIXED";
    case SX127X_VARIABLE:
      return "VARIABLE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_address_filtering_t parse_address(const char *str) {
  if (strcmp(str, "NODE_ADDRESS") == 0) return SX127X_FILTER_NODE_ADDRESS;
  if (strcmp(str, "NODE_AND_BROADCAST") == 0) return SX127X_FILTER_NODE_AND_BROADCAST;
  if (strcmp(str, "NONE") == 0) return SX127X_FILTER_NONE;
  return -1;
}

static const char *format_address(sx127x_address_filtering_t addr) {
  switch (addr) {
    case SX127X_FILTER_NODE_ADDRESS:
      return "NODE_ADDRESS";
    case SX127X_FILTER_NODE_AND_BROADCAST:
      return "NODE_AND_BROADCAST";
    case SX127X_FILTER_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_fsk_data_shaping_t parse_data_shaping(const char *str) {
  if (strcmp(str, "BT_0.3") == 0) return SX127X_BT_0_3;
  if (strcmp(str, "BT_0.5") == 0) return SX127X_BT_0_5;
  if (strcmp(str, "BT_1.0") == 0) return SX127X_BT_1_0;
  if (strcmp(str, "NONE") == 0) return SX127X_FSK_SHAPING_NONE;
  return -1;
}

static const char *format_data_shaping(sx127x_fsk_data_shaping_t shaping) {
  switch (shaping) {
    case SX127X_BT_0_3:
      return "BT_0.3";
    case SX127X_BT_0_5:
      return "BT_0.5";
    case SX127X_BT_1_0:
      return "BT_1.0";
    case SX127X_FSK_SHAPING_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_ook_data_shaping_t parse_ook_data_shaping(const char *str) {
  if (strcmp(str, "1BITRATE") == 0) return SX127X_1_BIT_RATE;
  if (strcmp(str, "2BITRATE") == 0) return SX127X_2_BIT_RATE;
  if (strcmp(str, "NONE") == 0) return SX127X_OOK_SHAPING_NONE;
  return -1;
}

static const char *format_ook_data_shaping(sx127x_ook_data_shaping_t shaping) {
  switch (shaping) {
    case SX127X_1_BIT_RATE:
      return "1BITRATE";
    case SX127X_2_BIT_RATE:
      return "2BITRATE";
    case SX127X_OOK_SHAPING_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_preamble_type_t parse_preamble_type(const char *str) {
  if (strcmp(str, "55") == 0) return SX127X_PREAMBLE_55;
  if (strcmp(str, "AA") == 0) return SX127X_PREAMBLE_AA;
  return -1;
}

static const char *format_preamble_type(sx127x_preamble_type_t preamble) {
  switch (preamble) {
    case SX127X_PREAMBLE_55:
      return "55";
    case SX127X_PREAMBLE_AA:
      return "AA";
    default:
      return "UNKNOWN";
  }
}

sx127x_ook_peak_thresh_step_t parse_peak_step(const char *str) {
  if (strcmp(str, "0.5db") == 0) return SX127X_0_5_DB;
  if (strcmp(str, "1.0db") == 0) return SX127X_1_0_DB;
  if (strcmp(str, "1.5db") == 0) return SX127X_1_5_DB;
  if (strcmp(str, "2.0db") == 0) return SX127X_2_0_DB;
  if (strcmp(str, "3.0db") == 0) return SX127X_3_0_DB;
  if (strcmp(str, "4.0db") == 0) return SX127X_4_0_DB;
  if (strcmp(str, "5.0db") == 0) return SX127X_5_0_DB;
  if (strcmp(str, "6.0db") == 0) return SX127X_6_0_DB;
  return -1;
}

static const char *format_peak_step(sx127x_ook_peak_thresh_step_t step) {
  switch (step) {
    case SX127X_0_5_DB:
      return "0.5db";
    case SX127X_1_0_DB:
      return "1.0db";
    case SX127X_1_5_DB:
      return "1.5db";
    case SX127X_2_0_DB:
      return "2.0db";
    case SX127X_3_0_DB:
      return "3.0db";
    case SX127X_4_0_DB:
      return "4.0db";
    case SX127X_5_0_DB:
      return "5.0db";
    case SX127X_6_0_DB:
      return "6.0db";
    default:
      return "UNKNOWN";
  }
}

static sx127x_ook_avg_offset_t parse_avg_offset(const char *str) {
  if (strcmp(str, "0db") == 0) return SX127X_0_DB;
  if (strcmp(str, "2db") == 0) return SX127X_2_DB;
  if (strcmp(str, "4db") == 0) return SX127X_4_DB;
  if (strcmp(str, "6db") == 0) return SX127X_6_DB;
  return -1;
}

static const char *format_avg_offset(sx127x_ook_avg_offset_t offset) {
  switch (offset) {
    case SX127X_0_DB:
      return "0db";
    case SX127X_2_DB:
      return "2db";
    case SX127X_4_DB:
      return "4db";
    case SX127X_6_DB:
      return "6db";
    default:
      return "UNKNOWN";
  }
}

static sx127x_rssi_smoothing_t parse_rssi_smoothing(const char *str) {
  if (strcmp(str, "2") == 0) return SX127X_2;
  if (strcmp(str, "4") == 0) return SX127X_4;
  if (strcmp(str, "8") == 0) return SX127X_8;
  if (strcmp(str, "16") == 0) return SX127X_16;
  if (strcmp(str, "32") == 0) return SX127X_32;
  if (strcmp(str, "64") == 0) return SX127X_64;
  if (strcmp(str, "128") == 0) return SX127X_128;
  if (strcmp(str, "256") == 0) return SX127X_256;
  return -1;
}

static const char *format_rssi_smoothing(sx127x_rssi_smoothing_t rssi) {
  switch (rssi) {
    case SX127X_2:
      return "2";
    case SX127X_4:
      return "4";
    case SX127X_8:
      return "8";
    case SX127X_16:
      return "16";
    case SX127X_32:
      return "32";
    case SX127X_64:
      return "64";
    case SX127X_128:
      return "128";
    case SX127X_256:
      return "256";
    default:
      return "UNKNOWN";
  }
}

static sx127x_rx_trigger_t parse_rx_trigger(const char *str) {
  if (strcmp(str, "PREAMBLE") == 0) return SX127X_RX_TRIGGER_PREAMBLE;
  if (strcmp(str, "NONE") == 0) return SX127X_RX_TRIGGER_NONE;
  if (strcmp(str, "RSSI") == 0) return SX127X_RX_TRIGGER_RSSI;
  if (strcmp(str, "RSSI_PREAMBLE") == 0) return SX127X_RX_TRIGGER_RSSI_PREAMBLE;
  return -1;
}

static const char *format_rx_trigger(sx127x_rx_trigger_t trigger) {
  switch (trigger) {
    case SX127X_RX_TRIGGER_PREAMBLE:
      return "PREAMBLE";
    case SX127X_RX_TRIGGER_NONE:
      return "NONE";
    case SX127X_RX_TRIGGER_RSSI:
      return "RSSI";
    case SX127X_RX_TRIGGER_RSSI_PREAMBLE:
      return "RSSI_PREAMBLE";
    default:
      return "UNKNOWN";
  }
}

static sx127x_ook_avg_thresh_t parse_avg_thresh(const char *str) {
  if (strcmp(str, "2pi") == 0) return SX127X_2_PI;
  if (strcmp(str, "4pi") == 0) return SX127X_4_PI;
  if (strcmp(str, "8pi") == 0) return SX127X_8_PI;
  if (strcmp(str, "32pi") == 0) return SX127X_32_PI;
  return -1;
}

static const char *format_avg_thresh(sx127x_ook_avg_thresh_t thresh) {
  switch (thresh) {
    case SX127X_2_PI:
      return "2pi";
    case SX127X_4_PI:
      return "4pi";
    case SX127X_8_PI:
      return "8pi";
    case SX127X_32_PI:
      return "32pi";
    default:
      return "UNKNOWN";
  }
}

static sx127x_ook_peak_thresh_dec_t parse_peak_dec(const char *str) {
  if (strcmp(str, "1.1") == 0) return SX127X_1_1_CHIP;
  if (strcmp(str, "1.2") == 0) return SX127X_1_2_CHIP;
  if (strcmp(str, "1.4") == 0) return SX127X_1_4_CHIP;
  if (strcmp(str, "1.8") == 0) return SX127X_1_8_CHIP;
  if (strcmp(str, "2.1") == 0) return SX127X_2_1_CHIP;
  if (strcmp(str, "4.1") == 0) return SX127X_4_1_CHIP;
  if (strcmp(str, "8.1") == 0) return SX127X_8_1_CHIP;
  if (strcmp(str, "16.1") == 0) return SX127X_16_1_CHIP;
  return -1;
}

static const char *format_peak_dec(sx127x_ook_peak_thresh_dec_t dec) {
  switch (dec) {
    case SX127X_1_1_CHIP:
      return "1.1";
    case SX127X_1_2_CHIP:
      return "1.2";
    case SX127X_1_4_CHIP:
      return "1.4";
    case SX127X_1_8_CHIP:
      return "1.8";
    case SX127X_2_1_CHIP:
      return "2.1";
    case SX127X_4_1_CHIP:
      return "4.1";
    case SX127X_8_1_CHIP:
      return "8.1";
    case SX127X_16_1_CHIP:
      return "16.1";
    default:
      return "UNKNOWN";
  }
}

static const char *format_thresh_type(sx127x_ook_thresh_type_t type) {
  switch (type) {
    case SX127X_OOK_FIXED:
      return "FIXED";
    case SX127X_OOK_AVG:
      return "AVG";
    case SX127X_OOK_PEAK:
      return "PEAK";
    default:
      return "UNKNOWN";
  }
}

static sx127x_pa_ramp_t parse_pa_ramp(const char *str) {
  if (strcmp(str, "3.4ms") == 0) return SX127X_PA_RAMP_1;
  if (strcmp(str, "2ms") == 0) return SX127X_PA_RAMP_2;
  if (strcmp(str, "1ms") == 0) return SX127X_PA_RAMP_3;
  if (strcmp(str, "500us") == 0) return SX127X_PA_RAMP_4;
  if (strcmp(str, "250us") == 0) return SX127X_PA_RAMP_5;
  if (strcmp(str, "125us") == 0) return SX127X_PA_RAMP_6;
  if (strcmp(str, "100us") == 0) return SX127X_PA_RAMP_7;
  if (strcmp(str, "62us") == 0) return SX127X_PA_RAMP_8;
  if (strcmp(str, "50us") == 0) return SX127X_PA_RAMP_9;
  if (strcmp(str, "40us") == 0) return SX127X_PA_RAMP_10;
  if (strcmp(str, "31us") == 0) return SX127X_PA_RAMP_11;
  if (strcmp(str, "25us") == 0) return SX127X_PA_RAMP_12;
  if (strcmp(str, "20us") == 0) return SX127X_PA_RAMP_13;
  if (strcmp(str, "15us") == 0) return SX127X_PA_RAMP_14;
  if (strcmp(str, "12us") == 0) return SX127X_PA_RAMP_15;
  if (strcmp(str, "10us") == 0) return SX127X_PA_RAMP_16;
  return -1;
}

static const char *format_pa_ramp(sx127x_pa_ramp_t pa_ramp) {
  switch (pa_ramp) {
    case SX127X_PA_RAMP_1:
      return "3.4ms";
    case SX127X_PA_RAMP_2:
      return "2ms";
    case SX127X_PA_RAMP_3:
      return "1ms";
    case SX127X_PA_RAMP_4:
      return "500us";
    case SX127X_PA_RAMP_5:
      return "250us";
    case SX127X_PA_RAMP_6:
      return "125us";
    case SX127X_PA_RAMP_7:
      return "100us";
    case SX127X_PA_RAMP_8:
      return "62us";
    case SX127X_PA_RAMP_9:
      return "50us";
    case SX127X_PA_RAMP_10:
      return "40us";
    case SX127X_PA_RAMP_11:
      return "31us";
    case SX127X_PA_RAMP_12:
      return "25us";
    case SX127X_PA_RAMP_13:
      return "20us";
    case SX127X_PA_RAMP_14:
      return "15us";
    case SX127X_PA_RAMP_15:
      return "12us";
    case SX127X_PA_RAMP_16:
      return "10us";
    default:
      return "UNKNOWN";
  }
}

static sx127x_mode_t parse_mode(const char *str) {
  if (strcmp(str, "SLEEP") == 0) return SX127x_MODE_SLEEP;
  if (strcmp(str, "STANDBY") == 0) return SX127x_MODE_STANDBY;
  if (strcmp(str, "FSTX") == 0) return SX127x_MODE_FSTX;
  if (strcmp(str, "TX") == 0) return SX127x_MODE_TX;
  if (strcmp(str, "FSRX") == 0) return SX127x_MODE_FSRX;
  if (strcmp(str, "RXCONT") == 0) return SX127x_MODE_RX_CONT;
  if (strcmp(str, "RXSINGLE") == 0) return SX127x_MODE_RX_SINGLE;
  if (strcmp(str, "CAD") == 0) return SX127x_MODE_CAD;
  return -1; // Invalid
}

static const char *format_mode(sx127x_mode_t mode) {
  switch (mode) {
    case SX127x_MODE_SLEEP:
      return "SLEEP";
    case SX127x_MODE_STANDBY:
      return "STANDBY";
    case SX127x_MODE_FSTX:
      return "FSTX";
    case SX127x_MODE_TX:
      return "TX";
    case SX127x_MODE_FSRX:
      return "FSRX";
    case SX127x_MODE_RX_CONT:
      return "RXCONT";
    case SX127x_MODE_RX_SINGLE:
      return "RXSINGLE";
    case SX127x_MODE_CAD:
      return "CAD";
    default:
      return "UNKNOWN";
  }
}

static sx127x_modulation_t parse_modulation(const char *str) {
  if (strcmp(str, "LORA") == 0) return SX127x_MODULATION_LORA;
  if (strcmp(str, "FSK") == 0) return SX127x_MODULATION_FSK;
  if (strcmp(str, "OOK") == 0) return SX127x_MODULATION_OOK;
  return -1; // Invalid
}

static const char *format_modulation(sx127x_modulation_t modulation) {
  switch (modulation) {
    case SX127x_MODULATION_LORA:
      return "LORA";
    case SX127x_MODULATION_FSK:
      return "FSK";
    case SX127x_MODULATION_OOK:
      return "OOK";
    default:
      return "UNKNOWN";
  }
}

static sx127x_cr_t parse_coding_rate(const char *str) {
  if (strcmp(str, "4_5") == 0) return SX127x_CR_4_5;
  if (strcmp(str, "4_6") == 0) return SX127x_CR_4_6;
  if (strcmp(str, "4_7") == 0) return SX127x_CR_4_7;
  if (strcmp(str, "4_8") == 0) return SX127x_CR_4_8;
  return -1; // Invalid
}

static const char *format_coding_rate(sx127x_cr_t cr) {
  switch (cr) {
    case SX127x_CR_4_5:
      return "4_5";
    case SX127x_CR_4_6:
      return "4_6";
    case SX127x_CR_4_7:
      return "4_7";
    case SX127x_CR_4_8:
      return "4_8";
    default:
      return "UNKNOWN";
  }
}

static sx127x_pa_pin_t parse_pa_pin(const char *str) {
  if (strcmp(str, "RFO") == 0) return SX127x_PA_PIN_RFO;
  if (strcmp(str, "BOOST") == 0) return SX127x_PA_PIN_BOOST;
  return -1; // Invalid
}

static const char *format_pa_pin(sx127x_pa_pin_t pin) {
  switch (pin) {
    case SX127x_PA_PIN_RFO:
      return "RFO";
    case SX127x_PA_PIN_BOOST:
      return "BOOST";
    default:
      return "UNKNOWN";
  }
}

static bool parse_bool(const char *str) {
  return (strcmp(str, "1") == 0 || strcmp(str, "TRUE") == 0);
}

static int split_params(char *input, char *params[]) {
  int count = 0;
  char *token = strtok(input, ",");
  while (token && count < MAX_PARAM_COUNT) {
    params[count++] = token;
    token = strtok(NULL, ",");
  }
  return count;
}

const char SYMBOLS[] = "0123456789ABCDEF";

static int at_util_string2hex(const char *str, uint8_t *output, size_t *output_len) {
  size_t len = 0;
  size_t str_len = strlen(str);
  for (size_t i = 0; i < str_len; i++) {
    if (str[i] == ' ' || str[i] == ':') {
      continue;
    }
    len++;
  }
  if (len % 2 != 0) {
    *output_len = 0;
    return SX127X_ERR_INVALID_ARG;
  }
  size_t bytes = len / 2;
  uint8_t curByte = 0;
  for (size_t i = 0, j = 0; i < strlen(str); i++) {
    char curChar = str[i];
    if (curChar == ' ' || str[i] == ':') {
      continue;
    }
    curByte *= 16;
    if (curChar >= '0' && curChar <= '9') {
      curByte += curChar - '0';
    } else if (curChar >= 'A' && curChar <= 'F') {
      curByte += (curChar - 'A') + 10;
    } else if (curChar >= 'a' && curChar <= 'f') {
      curByte += (curChar - 'a') + 10;
    } else {
      return SX127X_ERR_INVALID_ARG;
    }
    j++;
    if (j % 2 == 0) {
      output[j / 2 - 1] = curByte;
      curByte = 0;
    }
  }
  *output_len = bytes;
  return SX127X_OK;
}

static int at_util_hex2string(const uint8_t *input, size_t input_len, char *output) {
  for (size_t i = 0; i < input_len; i++) {
    uint8_t cur = input[i];
    output[2 * i] = SYMBOLS[cur >> 4];
    output[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  output[input_len * 2] = '\0';
  return SX127X_OK;
}

static int sx127x_at_handler_impl(sx127x *device, const char *input, char *output, size_t output_len) {
  if (!device || !input || !output) {
    return SX127X_ERR_INVALID_ARG;
  }

  // Check for AT+ prefix
  if (strncmp(input, "AT+", 3) != 0) {
    return SX127X_ERR_INVALID_ARG;
  }

  // Copy input to avoid modifying the original
  char cmd[MAX_INPUT_LEN];
  strncpy(cmd, input + 3, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Split command and parameters
  char *cmd_name = strtok(cmd, "=");
  char *param_str = strtok(NULL, "=");
  bool is_query = (cmd_name && cmd_name[strlen(cmd_name) - 1] == '?');
  if (is_query) {
    cmd_name[strlen(cmd_name) - 1] = '\0'; // Remove '?'
  }

  char *params[MAX_PARAM_COUNT];
  int param_count = 0;
  if (param_str) {
    param_count = split_params(param_str, params);
  }

  // Handle commands
  if (strcmp(cmd_name, "OPMOD") == 0) {
    if (is_query) {
      sx127x_mode_t mode;
      sx127x_modulation_t modulation;
      ERROR_CHECK(sx127x_get_opmod(device, &mode, &modulation));
      const char *mode_str = format_mode(mode);
      const char *mod_str = format_modulation(modulation);
      snprintf(output, output_len, "%s,%s\r\n", mode_str, mod_str);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_mode_t mode = parse_mode(params[0]);
      sx127x_modulation_t modulation = parse_modulation(params[1]);
      if (mode == -1 || modulation == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_set_opmod(mode, modulation, device);
    }
  }

  if (strcmp(cmd_name, "FREQ") == 0) {
    if (is_query) {
      uint64_t frequency;
      ERROR_CHECK(sx127x_get_frequency(device, &frequency));
      snprintf(output, output_len, "%" PRIu64 "\r\n", frequency);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      uint64_t frequency = atoll(params[0]);
      return sx127x_set_frequency(frequency, device);
    }
  }

  if (strcmp(cmd_name, "LBW") == 0) {
    if (is_query) {
      sx127x_bw_t bandwidth;
      ERROR_CHECK(sx127x_lora_get_bandwidth(device, &bandwidth));
      uint32_t bw_hz = sx127x_bandwidth_to_hz(bandwidth);
      snprintf(output, output_len, "%" PRIu32 "\r\n", bw_hz);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_bw_t bandwidth = sx127x_hz_to_bandwidth(atoi(params[0]));
      if (bandwidth == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_set_bandwidth(bandwidth, device);
    }
  }

  if (strcmp(cmd_name, "SF") == 0) {
    if (is_query) {
      sx127x_sf_t sf;
      ERROR_CHECK(sx127x_lora_get_spreading_factor(device, &sf));
      snprintf(output, output_len, "%d\r\n", (sf >> 4));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_sf_t sf = (atoi(params[0]) << 4);
      return sx127x_lora_set_spreading_factor(sf, device);
    }
  }

  if (strcmp(cmd_name, "LDO") == 0) {
    if (is_query) {
      bool enabled;
      ERROR_CHECK(sx127x_lora_get_low_datarate_optimization(device, &enabled));
      snprintf(output, output_len, "%s\r\n", enabled ? "TRUE" : "FALSE");
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_set_low_datarate_optimization(parse_bool(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "LORASW") == 0) {
    if (is_query) {
      uint8_t value;
      ERROR_CHECK(sx127x_lora_get_syncword(device, &value));
      snprintf(output, output_len, "%d\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_set_syncword(atoi(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "PLEN") == 0) {
    if (is_query) {
      uint16_t value;
      ERROR_CHECK(sx127x_get_preamble_length(device, &value));
      snprintf(output, output_len, "%d\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_set_preamble_length(atoi(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "IMPLHDR") == 0) {
    if (is_query) {
      sx127x_implicit_header_t header;
      bool enabled;
      ERROR_CHECK(sx127x_lora_get_implicit_header(device, &header, &enabled));
      if (!enabled) {
        snprintf(output, output_len, "DISABLED\r\n");
        return SX127X_OK;
      }
      const char *cr_str = format_coding_rate(header.coding_rate);
      snprintf(output, output_len, "%d,%s,%s\r\n", header.length, header.enable_crc ? "TRUE" : "FALSE", cr_str);
      return SX127X_OK;
    } else {
      if (param_count == 0) {
        // Disable implicit header
        return sx127x_lora_set_implicit_header(NULL, device);
      }
      if (param_count != 3) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_implicit_header_t header;
      header.length = atoi(params[0]);
      header.enable_crc = parse_bool(params[1]);
      header.coding_rate = parse_coding_rate(params[2]);
      if (header.coding_rate == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_set_implicit_header(&header, device);
    }
  }

  if (strcmp(cmd_name, "DUMPREG") == 0) {
    if (!is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    uint8_t registers[MAX_NUMBER_OF_REGISTERS];
    ERROR_CHECK(sx127x_dump_registers(registers, device));
    for (int i = 0; i < MAX_NUMBER_OF_REGISTERS; i++) {
      char reg_str[8];
      snprintf(reg_str, sizeof(reg_str), "%02X", registers[i]);
      strncat(output + strlen(output), reg_str, output_len - strlen(output) - 1);
      if (i < MAX_NUMBER_OF_REGISTERS - 1) {
        strncat(output + strlen(output), ",", output_len - strlen(output) - 1);
      }
    }
    strncat(output + strlen(output), "\r\n", output_len - strlen(output) - 1);
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "GAIN") == 0) {
    if (is_query) {
      sx127x_gain_t value;
      ERROR_CHECK(sx127x_rx_get_lna_gain(device, &value));
      snprintf(output, output_len, "%d\r\n", (value >> 5));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_rx_set_lna_gain((atoi(params[0]) << 5), device);
    }
  }

  if (strcmp(cmd_name, "LNABOOST") == 0) {
    if (is_query) {
      bool enabled;
      ERROR_CHECK(sx127x_rx_get_lna_boost_hf(device, &enabled));
      snprintf(output, output_len, "%s\r\n", enabled ? "TRUE" : "FALSE");
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_rx_set_lna_boost_hf(parse_bool(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "PPM") == 0) {
    if (is_query) {
      int32_t value;
      ERROR_CHECK(sx127x_lora_get_ppm_offset(device, &value));
      snprintf(output, output_len, "%" PRId32 "\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_set_ppm_offset(strtol(params[0], NULL, 10), device);
    }
  }

  if (strcmp(cmd_name, "PA") == 0) {
    if (is_query) {
      sx127x_pa_pin_t pin;
      int power;
      ERROR_CHECK (sx127x_tx_get_pa_config(device, &pin, &power));
      snprintf(output, output_len, "%s,%d\r\n", format_pa_pin(pin), power);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_pa_pin_t pin = parse_pa_pin(params[0]);
      int power = atoi(params[1]);
      if (pin == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_tx_set_pa_config(pin, power, device);
    }
  }

  if (strcmp(cmd_name, "OCP") == 0) {
    if (is_query) {
      bool enabled;
      uint8_t milliamps;
      ERROR_CHECK (sx127x_tx_get_ocp(device, &enabled, &milliamps));
      if (!enabled) {
        snprintf(output, output_len, "DISABLED\r\n");
        return SX127X_OK;
      }
      snprintf(output, output_len, "%d\r\n", milliamps);
      return SX127X_OK;
    } else {
      if (param_count == 0) {
        return sx127x_tx_set_ocp(false, 0, device);
      }
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_tx_set_ocp(true, atoi(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "TXHDR") == 0) {
    if (is_query) {
      bool enabled;
      sx127x_tx_header_t header;
      ERROR_CHECK (sx127x_lora_tx_get_explicit_header(device, &enabled, &header));
      if (!enabled) {
        snprintf(output, output_len, "DISABLED\r\n");
        return SX127X_OK;
      }
      const char *cr_str = format_coding_rate(header.coding_rate);
      snprintf(output, output_len, "%s,%s\r\n", header.enable_crc ? "TRUE" : "FALSE", cr_str);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_tx_header_t header;
      header.enable_crc = parse_bool(params[0]);
      header.coding_rate = parse_coding_rate(params[1]);
      if (header.coding_rate == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_lora_tx_set_explicit_header(&header, device);
    }
  }

  if (strcmp(cmd_name, "LORATX") == 0) {
    if (is_query || param_count != 1) {
      return SX127X_ERR_INVALID_ARG;
    }
    uint8_t message_hex[255];
    size_t message_hex_length;
    ERROR_CHECK(at_util_string2hex(params[0], message_hex, &message_hex_length));
    return sx127x_lora_tx_set_for_transmission(message_hex, message_hex_length, device);
  }

  if (strcmp(cmd_name, "FSKTX") == 0) {
    if (is_query || param_count < 1 || param_count > 2) {
      return SX127X_ERR_INVALID_ARG;
    }
    uint8_t message_hex[2048];
    size_t message_hex_length;
    ERROR_CHECK(at_util_string2hex(params[0], message_hex, &message_hex_length));
    if (param_count == 1) {
      return sx127x_fsk_ook_tx_set_for_transmission(message_hex, message_hex_length, device);
    }
    uint8_t address_to = atoi(params[1]);
    return sx127x_fsk_ook_tx_set_for_transmission_with_address(message_hex, message_hex_length, address_to, device);
  }

  if (strcmp(cmd_name, "FSKBCN") == 0) {
    if (is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    if (param_count == 0) {
      return sx127x_fsk_ook_tx_stop_beacon(device);
    }
    uint8_t message_hex[255];
    size_t message_hex_length;
    ERROR_CHECK(at_util_string2hex(params[0], message_hex, &message_hex_length));
    uint32_t interval = strtol(params[1], NULL, 10);
    return sx127x_fsk_ook_tx_start_beacon(message_hex, message_hex_length, interval, device);
  }

  if (strcmp(cmd_name, "BITRATE") == 0) {
    if (is_query) {
      float value;
      ERROR_CHECK(sx127x_fsk_ook_get_bitrate(device, &value));
      snprintf(output, output_len, "%f\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_bitrate(atof(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "FDEV") == 0) {
    if (is_query) {
      float value;
      ERROR_CHECK(sx127x_fsk_get_fdev(device, &value));
      snprintf(output, output_len, "%f\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_set_fdev(atof(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "FSKSW") == 0) {
    if (is_query) {
      uint8_t syncword[8];
      uint8_t syncword_length;
      ERROR_CHECK(sx127x_fsk_ook_get_syncword(device, syncword, &syncword_length));
      char syncword_message[18];
      at_util_hex2string(syncword, syncword_length, syncword_message);
      snprintf(output, output_len, "%s\r\n", syncword_message);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      uint8_t message_hex[255];
      size_t message_hex_length;
      ERROR_CHECK(at_util_string2hex(params[0], message_hex, &message_hex_length));
      return sx127x_fsk_ook_set_syncword(message_hex, message_hex_length, device);
    }
  }

  if (strcmp(cmd_name, "FSKENC") == 0) {
    if (is_query) {
      sx127x_packet_encoding_t value;
      ERROR_CHECK(sx127x_fsk_ook_get_packet_encoding(device, &value));
      snprintf(output, output_len, "%s\r\n", format_packet_encoding(value));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_packet_encoding_t value = parse_packet_encoding(params[0]);
      if (value == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_packet_encoding(value, device);
    }
  }

  if (strcmp(cmd_name, "FSKCRC") == 0) {
    if (is_query) {
      sx127x_crc_type_t value;
      ERROR_CHECK(sx127x_fsk_ook_get_crc(device, &value));
      snprintf(output, output_len, "%s\r\n", format_crc(value));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_packet_encoding_t value = parse_crc(params[0]);
      if (value == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_crc(value, device);
    }
  }

  if (strcmp(cmd_name, "FSKPKT") == 0) {
    if (is_query) {
      sx127x_packet_format_t value;
      uint16_t max_payload_length;
      ERROR_CHECK(sx127x_fsk_ook_get_packet_format(device, &value, &max_payload_length));
      snprintf(output, output_len, "%s,%d\r\n", format_packet_format(value), max_payload_length);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_packet_format_t value = parse_packet_format(params[0]);
      if (value == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_packet_format(value, atoi(params[1]), device);
    }
  }

  if (strcmp(cmd_name, "FSKADDR") == 0) {
    if (is_query) {
      sx127x_address_filtering_t value;
      uint8_t node_address;
      uint8_t broadcast_address;
      ERROR_CHECK(sx127x_fsk_ook_get_address_filtering(device, &value, &node_address, &broadcast_address));
      snprintf(output, output_len, "%s,%d,%d\r\n", format_address(value), node_address, broadcast_address);
      return SX127X_OK;
    } else {
      if (param_count != 3) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_address_filtering_t value = parse_address(params[0]);
      if (value == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_address_filtering(value, atoi(params[1]), atoi(params[2]), device);
    }
  }

  if (strcmp(cmd_name, "FSKSHAPING") == 0) {
    if (is_query) {
      sx127x_fsk_data_shaping_t value;
      sx127x_pa_ramp_t pa_ramp;
      ERROR_CHECK(sx127x_fsk_get_data_shaping(device, &value, &pa_ramp));
      snprintf(output, output_len, "%s,%s\r\n", format_data_shaping(value), format_pa_ramp(pa_ramp));
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_fsk_data_shaping_t value = parse_data_shaping(params[0]);
      sx127x_pa_ramp_t pa_ramp = parse_pa_ramp(params[1]);
      if (value == -1 || pa_ramp == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_set_data_shaping(value, pa_ramp, device);
    }
  }

  if (strcmp(cmd_name, "OOKSHAPING") == 0) {
    if (is_query) {
      sx127x_ook_data_shaping_t value;
      sx127x_pa_ramp_t pa_ramp;
      ERROR_CHECK(sx127x_ook_get_data_shaping(device, &value, &pa_ramp));
      snprintf(output, output_len, "%s,%s\r\n", format_ook_data_shaping(value), format_pa_ramp(pa_ramp));
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_fsk_data_shaping_t value = parse_ook_data_shaping(params[0]);
      sx127x_pa_ramp_t pa_ramp = parse_pa_ramp(params[1]);
      if (value == -1 || pa_ramp == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_ook_set_data_shaping(value, pa_ramp, device);
    }
  }

  if (strcmp(cmd_name, "PTYPE") == 0) {
    if (is_query) {
      sx127x_preamble_type_t value;
      ERROR_CHECK(sx127x_fsk_ook_get_preamble_type(device, &value));
      snprintf(output, output_len, "%s\r\n", format_preamble_type(value));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_preamble_type_t value = parse_preamble_type(params[0]);
      if (value == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_preamble_type(value, device);
    }
  }

  if (strcmp(cmd_name, "OOKTYPE") == 0) {
    if (!is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    sx127x_ook_thresh_type_t value;
    ERROR_CHECK(sx127x_ook_get_ook_thresh_type(device, &value));
    snprintf(output, output_len, "%s\r\n", format_thresh_type(value));
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "PEAKMODE") == 0) {
    if (is_query) {
      sx127x_ook_peak_thresh_step_t value;
      uint8_t floor_threshold;
      sx127x_ook_peak_thresh_dec_t decrement;
      ERROR_CHECK(sx127x_ook_rx_get_peak_mode(device, &value, &floor_threshold, &decrement));
      snprintf(output, output_len, "%s,%d,%s\r\n", format_peak_step(value), floor_threshold, format_peak_dec(decrement));
      return SX127X_OK;
    } else {
      if (param_count != 3) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_ook_peak_thresh_step_t value = parse_peak_step(params[0]);
      uint8_t floor_threshold = atoi(params[1]);
      sx127x_ook_peak_thresh_dec_t decrement = parse_peak_dec(params[2]);
      if (value == -1 || decrement == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_ook_rx_set_peak_mode(value, floor_threshold, decrement, device);
    }
  }

  if (strcmp(cmd_name, "FIXEDMODE") == 0) {
    if (is_query) {
      uint8_t value;
      ERROR_CHECK(sx127x_ook_rx_get_fixed_mode(device, &value));
      snprintf(output, output_len, "%d\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_ook_rx_set_fixed_mode(atoi(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "AVGMODE") == 0) {
    if (is_query) {
      sx127x_ook_avg_offset_t value;
      sx127x_ook_avg_thresh_t thresh;
      ERROR_CHECK(sx127x_ook_rx_get_avg_mode(device, &value, &thresh));
      snprintf(output, output_len, "%s,%s\r\n", format_avg_offset(value), format_avg_thresh(thresh));
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_ook_avg_offset_t value = parse_avg_offset(params[0]);
      sx127x_ook_avg_thresh_t thresh = parse_avg_thresh(params[1]);
      if (value == -1 || thresh == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_ook_rx_set_avg_mode(value, thresh, device);
    }
  }

  if (strcmp(cmd_name, "AFC") == 0) {
    if (is_query) {
      bool value;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_afc_auto(device, &value));
      snprintf(output, output_len, "%s\r\n", value ? "TRUE" : "FALSE");
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_afc_auto(parse_bool(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "AFCBW") == 0) {
    if (is_query) {
      float value;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_afc_bandwidth(device, &value));
      snprintf(output, output_len, "%f\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_afc_bandwidth(atof(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "FSKBW") == 0) {
    if (is_query) {
      float value;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_bandwidth(device, &value));
      snprintf(output, output_len, "%f\r\n", value);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_bandwidth(atof(params[0]), device);
    }
  }

  if (strcmp(cmd_name, "RSSI") == 0) {
    if (is_query) {
      sx127x_rssi_smoothing_t smoothing;
      int8_t offset;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_rssi_config(device, &smoothing, &offset));
      snprintf(output, output_len, "%s,%d\r\n", format_rssi_smoothing(smoothing), offset);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_rssi_smoothing_t smoothing = parse_rssi_smoothing(params[0]);
      int8_t offset = atoi(params[1]);
      return sx127x_fsk_ook_rx_set_rssi_config(smoothing, offset, device);
    }
  }

  if (strcmp(cmd_name, "COLLISION") == 0) {
    if (is_query) {
      bool enable;
      uint8_t threshold;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_collision_restart(device, &enable, &threshold));
      snprintf(output, output_len, "%s,%d\r\n", enable ? "TRUE" : "FALSE", threshold);
      return SX127X_OK;
    } else {
      if (param_count != 2) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_collision_restart(parse_bool(params[0]), atoi(params[1]), device);
    }
  }

  if (strcmp(cmd_name, "TRIGGER") == 0) {
    if (is_query) {
      sx127x_rx_trigger_t trigger;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_trigger(device, &trigger));
      snprintf(output, output_len, "%s\r\n", format_rx_trigger(trigger));
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_rx_trigger_t trigger = parse_rx_trigger(params[0]);
      if (trigger == -1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_trigger(trigger, device);
    }
  }

  if (strcmp(cmd_name, "PCONF") == 0) {
    if (is_query) {
      bool enable;
      uint8_t detector_size;
      uint8_t detector_tolerance;
      ERROR_CHECK(sx127x_fsk_ook_rx_get_preamble_detector(device, &enable, &detector_size, &detector_tolerance));
      snprintf(output, output_len, "%s,%d,%d\r\n", enable ? "TRUE" : "FALSE", detector_size, detector_tolerance);
      return SX127X_OK;
    } else {
      if (param_count != 3) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_rx_set_preamble_detector(parse_bool(params[0]), atoi(params[1]), atoi(params[2]), device);
    }
  }

  if (strcmp(cmd_name, "RXCAL") == 0) {
    if (is_query) {
      return SX127X_ERR_INVALID_ARG;
    }
    if (param_count != 0) {
      return SX127X_ERR_INVALID_ARG;
    }
    return sx127x_fsk_ook_rx_calibrate(device);
  }

  if (strcmp(cmd_name, "TEMP") == 0) {
    if (is_query) {
      int8_t temp;
      ERROR_CHECK(sx127x_fsk_ook_get_raw_temperature(device, &temp));
      snprintf(output, output_len, "%d\r\n", temp);
      return SX127X_OK;
    } else {
      if (param_count != 1) {
        return SX127X_ERR_INVALID_ARG;
      }
      return sx127x_fsk_ook_set_temp_monitor(parse_bool(params[0]), device);
    }
  }

// TODO sx127x_lora_set_frequency_hopping

  return SX127X_CONTINUE;
}

int sx127x_at_handler(sx127x *device, const char *input, char *output, size_t output_len) {
  int result = sx127x_at_handler_impl(device, input, output, output_len);
  if (result == SX127X_ERR_INVALID_ARG) {
    snprintf(output, output_len, "invalid argument\r\nERROR\r\n");
  } else if (result == SX127X_CONTINUE) {
    // do nothing
  } else if (result == SX127X_OK) {
    snprintf(output + strlen(output), output_len - strlen(output), "OK\r\n");
  } else {
    snprintf(output, output_len, "operation failed %d\r\nERROR\r\n", result);
  }
  return result;
}