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
      snprintf(output, output_len, "operation failed %d\r\nERROR\r\n", __err_rc);                           \
      return __err_rc;           \
    }                            \
  } while (0)

#define ERROR_CHECK_SETTER(x)           \
  do {                           \
    int __err_rc = (x);                 \
    snprintf(output, output_len, __err_rc == SX127X_OK ? "OK\r\n" : "%d\r\nERROR\r\n", __err_rc);                                    \
  } while (0)

// Helper function to convert string to sx127x_mode_t
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
      // remaining
    default:
      return "CAD";
  }
}

// Helper function to convert string to sx127x_modulation_t
static sx127x_modulation_t parse_modulation(const char *str) {
  if (strcmp(str, "LORA") == 0) return SX127x_MODULATION_LORA;
  if (strcmp(str, "FSK") == 0) return SX127x_MODULATION_FSK;
  if (strcmp(str, "OOK\r\n") == 0) return SX127x_MODULATION_OOK;
  return -1; // Invalid
}

static const char *format_modulation(sx127x_modulation_t modulation) {
  switch (modulation) {
    case SX127x_MODULATION_LORA:
      return "LORA";
    case SX127x_MODULATION_FSK:
      return "FSK";
    default:
      return "OOK\r\n";
  }
}

// Helper function to convert string to sx127x_bw_t
static sx127x_bw_t parse_bandwidth(const char *str) {
  if (strcmp(str, "7800") == 0) return SX127x_BW_7800;
  if (strcmp(str, "10400") == 0) return SX127x_BW_10400;
  if (strcmp(str, "15600") == 0) return SX127x_BW_15600;
  if (strcmp(str, "20800") == 0) return SX127x_BW_20800;
  if (strcmp(str, "31250") == 0) return SX127x_BW_31250;
  if (strcmp(str, "41700") == 0) return SX127x_BW_41700;
  if (strcmp(str, "62500") == 0) return SX127x_BW_62500;
  if (strcmp(str, "125000") == 0) return SX127x_BW_125000;
  if (strcmp(str, "250000") == 0) return SX127x_BW_250000;
  if (strcmp(str, "500000") == 0) return SX127x_BW_500000;
  return -1; // Invalid
}

// Helper function to convert string to sx127x_cr_t
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
    default:
      return "4_8";
  }
}

// Helper function to convert string to sx127x_pa_pin_t
static sx127x_pa_pin_t parse_pa_pin(const char *str) {
  if (strcmp(str, "RFO") == 0) return SX127x_PA_PIN_RFO;
  if (strcmp(str, "BOOST") == 0) return SX127x_PA_PIN_BOOST;
  return -1; // Invalid
}

// Helper function to parse boolean
static bool parse_bool(const char *str) {
  return (strcmp(str, "1") == 0 || strcmp(str, "TRUE") == 0);
}

// Helper function to split parameters
static int split_params(char *input, char *params[]) {
  int count = 0;
  char *token = strtok(input, ",");
  while (token && count < MAX_PARAM_COUNT) {
    params[count++] = token;
    token = strtok(NULL, ",");
  }
  return count;
}

int sx127x_at_handler(sx127x *device, const char *input, char *output, size_t output_len) {
  if (!device || !input || !output) {
    snprintf(output, output_len, "invalid argument\r\nERROR\r\n");
    return SX127X_ERR_INVALID_ARG;
  }

  // Check for AT+ prefix
  if (strncmp(input, "AT+", 3) != 0) {
    snprintf(output, output_len, "not an at command\r\nERROR\r\n");
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
      snprintf(output, output_len, "%s,%s\r\nOK\r\n", mode_str, mod_str);
    } else {
      if (param_count != 2) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_mode_t mode = parse_mode(params[0]);
      sx127x_modulation_t modulation = parse_modulation(params[1]);
      if (mode == -1 || modulation == -1) {
        snprintf(output, output_len, "invalid param\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_set_opmod(mode, modulation, device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "FREQ") == 0) {
    if (is_query) {
      uint64_t frequency;
      ERROR_CHECK(sx127x_get_frequency(device, &frequency));
      snprintf(output, output_len, "%" PRIu64 "\r\nOK\r\n", frequency);
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      uint64_t frequency = atoll(params[0]);
      ERROR_CHECK_SETTER(sx127x_set_frequency(frequency, device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "LBW") == 0) {
    if (is_query) {
      sx127x_bw_t bandwidth;
      ERROR_CHECK(sx127x_lora_get_bandwidth(device, &bandwidth));
      uint32_t bw_hz = sx127x_bandwidth_to_hz(bandwidth);
      snprintf(output, output_len, "%" PRIu32 "\r\nOK\r\n", bw_hz);
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_bw_t bandwidth = sx127x_hz_to_bandwidth(atoi(params[0]));
      if (bandwidth == -1) {
        snprintf(output, output_len, "invalid bandwidth\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_lora_set_bandwidth(bandwidth, device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "SF") == 0) {
    if (is_query) {
      sx127x_sf_t sf;
      ERROR_CHECK(sx127x_lora_get_spreading_factor(device, &sf));
      snprintf(output, output_len, "%d\r\nOK\r\n", (sf >> 4));
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_sf_t sf = (atoi(params[0]) << 4);
      ERROR_CHECK_SETTER(sx127x_lora_set_spreading_factor(sf, device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "LDO") == 0) {
    if (is_query) {
      bool enabled;
      ERROR_CHECK(sx127x_lora_get_low_datarate_optimization(device, &enabled));
      snprintf(output, output_len, "%s\r\nOK\r\n", enabled ? "TRUE" : "FALSE");
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_lora_set_low_datarate_optimization(parse_bool(params[0]), device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "LSW") == 0) {
    if (is_query) {
      uint8_t value;
      ERROR_CHECK(sx127x_lora_get_syncword(device, &value));
      snprintf(output, output_len, "%d\r\nOK\r\n", value);
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_lora_set_syncword(atoi(params[0]), device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "PLEN") == 0) {
    if (is_query) {
      uint16_t value;
      ERROR_CHECK(sx127x_get_preamble_length(device, &value));
      snprintf(output, output_len, "%d\r\nOK\r\n", value);
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_set_preamble_length(atoi(params[0]), device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "IMPLHDR") == 0) {
    if (is_query) {
      sx127x_implicit_header_t header;
      bool enabled;
      ERROR_CHECK(sx127x_lora_get_implicit_header(device, &header, &enabled));
      if (!enabled) {
        snprintf(output, output_len, "DISABLED\r\nOK\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      const char *cr_str = format_coding_rate(header.coding_rate);
      snprintf(output, output_len, "%d,%d,%s\r\nOK\r\n", header.length, header.enable_crc, cr_str);
    } else {
      if (param_count == 0) {
        // Disable implicit header
        ERROR_CHECK_SETTER(sx127x_lora_set_implicit_header(NULL, device));
        return SX127X_OK;
      }
      if (param_count != 3) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      sx127x_implicit_header_t header;
      header.length = atoi(params[0]);
      header.enable_crc = parse_bool(params[1]);
      header.coding_rate = parse_coding_rate(params[2]);
      if (header.coding_rate == -1) {
        snprintf(output, output_len, "invalid coding rate\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_lora_set_implicit_header(&header, device));
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "DUMPREG") == 0) {
    if (is_query) {
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
      strncat(output + strlen(output), "\r\nOK\r\n", output_len - strlen(output) - 1);
    } else {
      snprintf(output + strlen(output), output_len - strlen(output), "set is not supported\r\nERROR\r\n");
    }
    return SX127X_OK;
  }

  if (strcmp(cmd_name, "GAIN") == 0) {
    if (is_query) {
      sx127x_gain_t value;
      ERROR_CHECK(sx127x_rx_get_lna_gain(device, &value));
      snprintf(output, output_len, "%d\r\nOK\r\n", (value >> 5));
    } else {
      if (param_count != 1) {
        snprintf(output, output_len, "invalid param count\r\nERROR\r\n");
        return SX127X_ERR_INVALID_ARG;
      }
      ERROR_CHECK_SETTER(sx127x_rx_set_lna_gain((atoi(params[0]) << 5), device));
    }
    return SX127X_OK;
  }

//  if (strcmp(cmd_name, "TXPA") == 0) {
//    if (is_query) {
//      sx127x_pa_pin_t pin;
//      int power;
//      if (sx127x_tx_get_pa_config(device, &pin, &power) == SX127X_OK) {
//        const char *pin_str = (pin == SX127x_PA_PIN_RFO) ? "RFO" : "BOOST";
//        snprintf(output + strlen(output), output_len - strlen(output), "%s,%d", pin_str, power);
//      } else {
//        snprintf(output + strlen(output), output_len - strlen(output), "ERROR,GET_TXPA_FAILED");
//      }
//    } else {
//      if (param_count != 2) {
//        snprintf(output + strlen(output), output_len - strlen(output), "ERROR,INVALID_PARAM_COUNT");
//        return SX127X_ERR_INVALID_ARG;
//      }
//      sx127x_pa_pin_t pin = parse_pa_pin(params[0]);
//      int power = atoi(params[1]);
//      if (pin == -1) {
//        snprintf(output + strlen(output), output_len - strlen(output), "ERROR,INVALID_PIN");
//        return SX127X_ERR_INVALID_ARG;
//      }
//      int result = sx127x_tx_set_pa_config(pin, power, device);
//      snprintf(output + strlen(output), output_len - strlen(output), result == SX127X_OK ? "OK\r\n" : "ERROR,%d", result);
//    }
//  } else if (strcmp(cmd_name, "TX") == 0) {
//    if (param_count < 1) {
//      snprintf(output + strlen(output), output_len - strlen(output), "ERROR,INVALID_PARAM_COUNT");
//      return;
//    }
//    uint8_t data[CONFIG_SX127X_MAX_PACKET_SIZE];
//    uint8_t data_length = 0;
//    // Convert hex string to binary
//    for (int i = 0; i < strlen(params[0]) && data_length < CONFIG_SX127X_MAX_PACKET_SIZE; i += 2) {
//      char hex[3] = {params[0][i], params[0][i + 1], '\0'};
//      data[data_length++] = (uint8_t) strtol(hex, NULL, 16);
//    }
//    int result = (device->active_modem == SX127x_MODULATION_LORA) ?
//                 sx127x_lora_tx_set_for_transmission(data, data_length, device) :
//                 sx127x_fsk_ook_tx_set_for_transmission(data, data_length, device);
//    snprintf(output + strlen(output), output_len - strlen(output), result == SX127X_OK ? "OK\r\n" : "ERROR,%d", result);
//  } else {
//    snprintf(output, output_len, "ERROR,UNKNOWN_COMMAND");
//  }

  return SX127X_CONTINUE;
}