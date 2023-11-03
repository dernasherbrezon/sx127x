#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

void print_op_mode(uint8_t value) {
  printf("0x01 RegOpMode: \n");
  if ((value & 0b10000000) == 0b10000000) {
    printf("\tLongRangeMode=LORA\n");
  } else {
    printf("\tLongRangeMode=FSK\n");
  }
  if (((value & 0b01000000) >> 6) == 0) {
    printf("\tAccessSharedReg=Access LoRa registers\n");
  } else {
    printf("\tAccessSharedReg=Access FSK registers\n");
  }
  if (((value & 0b00001000) >> 3) == 0) {
    printf("\tLowFrequencyModeOn=High Frequency Mode\n");
  } else {
    printf("\tLowFrequencyModeOn=Low Frequency Mode\n");
  }
  uint8_t mode = (value & 0b111);
  switch (mode) {
    case 0b000:
      printf("\tMode=SLEEP\n");
      break;
    case 0b001:
      printf("\tMode=STDBY\n");
      break;
    case 0b010:
      printf("\tMode=Frequency synthesis TX\n");
      break;
    case 0b011:
      printf("\tMode=Transmit (TX)\n");
      break;
    case 0b100:
      printf("\tMode=Frequency synthesis RX (FSRX)\n");
      break;
    case 0b101:
      printf("\tMode=Receive continuous\n");
      break;
    case 0b110:
      printf("\tMode=receive single\n");
      break;
    case 0b111:
      printf("\tMode=Channel activity detection\n");
      break;
  }
}

void print_ocp(uint8_t value) {
  printf("0x0b: RegOcp:\n");
  if ((value & 0b100000) != 0) {
    printf("\tOcpOn=OCP enabled\n");
  } else {
    printf("\tOcpOn=OCP disabled\n");
  }
  printf("\tOcpTrim=0x%x\n", (value & 0b11111));
}

void print_lna(uint8_t value) {
  printf("0x0c: RegLna:\n");
  printf("\tLnaGain=%d\n", ((value & 0b11100000) >> 5));
  printf("\tLnaBoostLf=%d\n", ((value & 0b11000) >> 3));
  if (((value & 0b11)) != 0) {
    printf("\tLnaBoostHf=Boost on\n");
  } else {
    printf("\tLnaBoostHf=Default LNA current\n");
  }
}

int dump_fsk_registers(const uint8_t *regs) {
  uint8_t value = regs[0x01];
  print_op_mode(value);
  printf("0x02: RegBitrateMsb:\n");
  double bit_rate = 32000000.0 / (((regs[0x02] << 8) | (regs[0x03])) + regs[0x5D] / 16.0);
  printf("\tBitRate=%f\n", bit_rate);
  printf("0x04: RegFdevMsb\n");
  double freq_deviation = (32000000.0 / (1 << 19)) * (((regs[0x04] & 0b111111) << 8) | regs[0x05]);
  printf("\tFdev=%f\n", freq_deviation);
  uint64_t freq = (((uint64_t) regs[0x06]) << 16) | (((uint64_t) regs[0x07]) << 8) | (regs[0x08]);
  printf("0x06: RegFr:\n");
  printf("\tFrf=%" PRIu64 "\n", ((freq * 32000000) / (1 << 19)));
  printf("0x09: RegPaConfig:\n");
  value = regs[0x07];
  if ((value & 0b10000000) == 0b10000000) {
    printf("\tPaSelect=PA_BOOST pin\n");
  } else {
    printf("\tPaSelect=RFO pin\n");
  }
  printf("\tMaxPower=0x%x\n", ((value & 0b110000) >> 4));
  printf("\tOutputPower=0x%x\n", ((value & 0b1111)));
  printf("0x0a: RegPaRamp:\n");
  printf("\tModulationShaping=");
  value = (regs[0x0a] & 0b01100000) >> 5;
  switch (value) {
    case 0b00:
      printf("No shaping");
      break;
    case 0b01:
      printf("Gaussian filter BT = 1.0");
      break;
    case 0b10:
      printf("Gaussian filter BT = 0.5");
      break;
    case 0b11:
      printf("Gaussian filter BT = 0.3");
      break;
  }
  printf("\n");
  printf("\tPaRamp=0x%x\n", (regs[0x0a] & 0b1111));
  print_ocp(regs[0x0b]);
  print_lna(regs[0x0c]);
  printf("0x0d: RegRxConfig\n");
  if ((regs[0x0d] & 0b10000000) != 0) {
    printf("\tRestartRxOnCollision=Automatic restart On\n");
  } else {
    printf("\tRestartRxOnCollision=No automatic Restart\n");
  }
  if ((regs[0x0d] & 0b1000000) != 0) {
    printf("\tRestartRxWithoutPllLock=Manual Restart of the Receiver\n");
  } else {
    printf("\tRestartRxWithoutPllLock=No restart\n");
  }
  if ((regs[0x0d] & 0b100000) != 0) {
    printf("\tRestartRxWithPllLock=Manual Restart of the Receiver\n");
  } else {
    printf("\tRestartRxWithPllLock=No restart\n");
  }
  if ((regs[0x0d] & 0b10000) != 0) {
    printf("\tAfcAutoOn=AFC is performed at each receiver startup\n");
  } else {
    printf("\tAfcAutoOn=No AFC performed at receiver startup\n");
  }
  if ((regs[0x0d] & 0b1000) != 0) {
    printf("\tAgcAutoOn=LNA gain is controlled by the AGC\n");
  } else {
    printf("\tAgcAutoOn=LNA gain forced by the LnaGain Setting\n");
  }
  uint8_t trigger_event = (regs[0x0d] & 0b111);
  switch (trigger_event) {
    case 0b000:
      printf("\tRxTrigger=None\n");
      break;
    case 0b001:
      printf("\tRxTrigger=RSSI\n");
      break;
    case 0b110:
      printf("\tRxTrigger=PreambleDetect\n");
      break;
    case 0b111:
      printf("\tRxTrigger=RSSI and PreambleDetect\n");
      break;
  }
  printf("0x0e: RegRssiConfig\n");
  printf("\tRssiOffset=%d\n", (regs[0x0e] & 0b11111000) >> 3);
  printf("\tRssiSmoothing=%d\n", (regs[0x0e] & 0b111));
  printf("0x0f: RegRssiCollision\n");
  printf("\tRssiCollisionThreshold=%d\n", (regs[0x0f]));
  printf("0x10: RegRssiThresh\n");
  printf("\tRssiThreshold=%d\n", (regs[0x10] / 2));
  printf("0x11: RegRssiValue\n");
  printf("\tRssiValue=%f\n", (regs[0x11] / 2.0f));
  return EXIT_SUCCESS;
}

int dump_lora_registers(uint8_t *regs) {
  uint8_t value = regs[0x01];
  print_op_mode(value);
  uint64_t freq = (((uint64_t) regs[0x06]) << 16) | (((uint64_t) regs[0x07]) << 8) | (regs[0x08]);
  printf("0x06: RegFr:\n");
  printf("\tFrf=%" PRIu64 "\n", ((freq * 32000000) / (1 << 19)));
  printf("0x09: RegPaConfig:\n");
  value = regs[0x07];
  if ((value & 0b10000000) == 0b10000000) {
    printf("\tPaSelect=PA_BOOST pin\n");
  } else {
    printf("\tPaSelect=RFO pin\n");
  }
  printf("\tMaxPower=0x%x\n", ((value & 0b110000) >> 4));
  printf("\tOutputPower=0x%x\n", ((value & 0b1111)));
  printf("0x0a: RegPaRamp:\n");
  printf("\tPaRamp=0x%x\n", regs[0x0a]);
  print_ocp(regs[0x0b]);
  print_lna(regs[0x0c]);
  printf("0x0d: RegFifoAddrPtr:\n");
  printf("\tFifoAddrPtr=%x\n", regs[0x0d]);
  printf("0x0e: RegFifoTxBaseAddr:\n");
  printf("\tFifoTxBaseAddr=%x\n", regs[0x0e]);
  printf("0x0f: RegFifoRxBaseAddr:\n");
  printf("\tFifoRxBaseAddr=%x\n", regs[0x0f]);
  printf("0x10: RegFifoRxCurrentAddr:\n");
  printf("\tFifoRxCurrentAddr=%x\n", regs[0x10]);
  value = regs[0x1d];
  printf("0x1d: RegModemConfig1:\n");
  switch (((value & 0b11110000) >> 4)) {
    case 0b0000:
      printf("\tBw=7.8 kHz\n");
      break;
    case 0b0001:
      printf("\tBw=10.4 kHz\n");
      break;
    case 0b0010:
      printf("\tBw=15.6 kHz\n");
      break;
    case 0b0011:
      printf("\tBw=20.8kHz\n");
      break;
    case 0b0100:
      printf("\tBw=31.25 kHz\n");
      break;
    case 0b0101:
      printf("\tBw=41.7 kHz\n");
      break;
    case 0b0110:
      printf("\tBw=62.5 kHz\n");
      break;
    case 0b0111:
      printf("\tBw=125 kHz\n");
      break;
    case 0b1000:
      printf("\tBw=250 kHz\n");
      break;
    case 0b1001:
      printf("\tBw=500 kHz\n");
      break;
  }
  switch (((value & 0b1110) >> 1)) {
    case 0b001:
      printf("\tCodingRate=4/5\n");
      break;
    case 0b010:
      printf("\tCodingRate=4/6\n");
      break;
    case 0b011:
      printf("\tCodingRate=4/7\n");
      break;
    case 0b100:
      printf("\tCodingRate=4/8\n");
      break;
  }
  printf("\tImplicitHeaderModeOn=%d\n", (value & 0b1));
  printf("0x1e: RegModemConfig2\n");
  value = regs[0x1e];
  printf("\tSpreadingFactor=%d\n", ((value & 0b11110000) >> 4));
  if (((value & 0b1000) >> 3) != 0) {
    printf("\tTxContinuousMode=continuous mode\n");
  } else {
    printf("\tTxContinuousMode=normal mode\n");
  }
  printf("\tRxPayloadCrcOn=%d\n", ((value & 0b100) >> 2));
  printf("0x1f: RegSymbTimeoutLsb:\n");
  printf("\tSymbTimeout=%d\n", ((regs[0x1e] & 0b11) << 8) | regs[0x1f]);
  printf("0x20: RegPreamble:\n");
  printf("\tPreambleLength=%d\n", (regs[0x20] << 8) | regs[0x21]);
  printf("0x22: RegPayloadLength:\n");
  printf("\tPayloadLength=%d\n", regs[0x22]);
  printf("0x23: RegMaxPayloadLength:\n");
  printf("\tPayloadMaxLength=%d\n", regs[0x23]);
  printf("0x24: RegHopPeriod:\n");
  printf("\tFreqHoppingPeriod=%d\n", regs[0x24]);
  printf("0x25: RegFifoRxByteAddr:\n");
  printf("\tFifoRxByteAddrPtr=%d\n", regs[0x25]);
  printf("0x26: RegModemConfig3:\n");
  value = regs[0x26];
  if ((value & 0b1000) != 0) {
    printf("\tLowDataRateOptimize=Enabled\n");
  } else {
    printf("\tLowDataRateOptimize=Disabled\n");
  }
  printf("\tAgcAutoOn=%d\n", ((value & 0b100) >> 2));
  printf("0x27: PpmCorrection:\n");
  printf("\tPpmCorrection=%d\n", regs[0x27]);
  printf("0x31: RegDetectOptimize:\n");
  printf("\tDetectionOptimize=%d\n", (regs[0x31] & 0b111));
  printf("0x37: RegDetectionThreshold:\n");
  printf("\tDetectionThreshold=%d\n", regs[0x37]);
  printf("0x39: RegSyncWord:\n");
  printf("\tSyncWord=%d\n", regs[0x39]);
  printf("0x40: RegDioMapping1\n");
  value = ((regs[0x40] & 0b11000000) >> 6);
  switch (value) {
    case 0b00:
      printf("\tDIO0=RxDone\n");
      break;
    case 0b01:
      printf("\tDIO0=TxDone\n");
      break;
    case 0b10:
      printf("\tDIO0=CadDone\n");
      break;
    case 0b11:
      printf("\tDIO0=Invalid\n");
      break;
  }
  value = ((regs[0x40] & 0b00110000) >> 4);
  switch (value) {
    case 0b00:
      printf("\tDIO1=RxTimeout\n");
      break;
    case 0b01:
      printf("\tDIO1=FhssChangeChannel\n");
      break;
    case 0b10:
      printf("\tDIO1=CadDetected\n");
      break;
    case 0b11:
      printf("\tDIO1=Invalid\n");
      break;
  }
  value = ((regs[0x40] & 0b00001100) >> 2);
  switch (value) {
    case 0b00:
      printf("\tDIO2=FhssChangeChannel\n");
      break;
    case 0b01:
      printf("\tDIO2=FhssChangeChannel\n");
      break;
    case 0b10:
      printf("\tDIO2=FhssChangeChannel\n");
      break;
    case 0b11:
      printf("\tDIO2=Invalid\n");
      break;
  }
  value = ((regs[0x40] & 0b11));
  switch (value) {
    case 0b00:
      printf("\tDIO3=CadDone\n");
      break;
    case 0b01:
      printf("\tDIO3=ValidHeader\n");
      break;
    case 0b10:
      printf("\tDIO3=PayloadCrcError\n");
      break;
    case 0b11:
      printf("\tDIO3=Invalid\n");
      break;
  }
  printf("0x41: RegDioMapping2\n");
  value = ((regs[0x41] & 0b11000000) >> 6);
  switch (value) {
    case 0b00:
      printf("\tDIO4=CadDetected\n");
      break;
    case 0b01:
      printf("\tDIO4=PllLock\n");
      break;
    case 0b10:
      printf("\tDIO4=PllLock\n");
      break;
    case 0b11:
      printf("\tDIO4=Invalid\n");
      break;
  }
  value = ((regs[0x41] & 0b110000) >> 4);
  switch (value) {
    case 0b00:
      printf("\tDIO5=ModeReady\n");
      break;
    case 0b01:
      printf("\tDIO5=ClkOut\n");
      break;
    case 0b10:
      printf("\tDIO5=ClkOut\n");
      break;
    case 0b11:
      printf("\tDIO5=Invalid\n");
      break;
  }
  return 0;
}

int at_util_string2hex(const char *str, uint8_t **output, size_t *output_length) {
  size_t len = 0;
  size_t str_len = strlen(str);
  for (size_t i = 0; i < str_len; i++) {
    if (str[i] == ' ' || str[i] == ':' || str[i] == ',') {
      continue;
    }
    //expect 0x99 so each "x" is for separate number
    if (str[i] == 'x') {
      len++;
    }
  }
  size_t bytes = len;
  uint8_t *result = malloc(sizeof(uint8_t) * bytes);
  if (result == NULL) {
    return -1;
  }
  uint8_t curByte = 0;
  for (size_t i = 0, j = 0; i < strlen(str);) {
    char curChar = str[i];
    if (curChar == ' ' || str[i] == ':') {
      i++;
      continue;
    }
    if (str[i] == ',') {
      result[j] = curByte;
      curByte = 0;
      i++;
      j++;
      continue;
    }
    if (str[i] == '0' && str[i + 1] == 'x') {
      i += 2;
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
      return -1;
    }
    i++;
  }
  *output = result;
  *output_length = bytes;
  return 0;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "missing argument\n");
    return EXIT_FAILURE;
  }
  uint8_t *output = NULL;
  size_t output_length = 0;
  int code = at_util_string2hex(argv[1], &output, &output_length);
  if (code != 0) {
    return EXIT_FAILURE;
  }
  uint8_t *prepended = malloc(sizeof(uint8_t) * (output_length + 1));
  memset(prepended, 0, (output_length + 1));
  memcpy(prepended + 1, output, output_length);
  if ((prepended[0x01] & 0b10000000) == 0b10000000) {
    return dump_lora_registers(prepended);
  }
  if ((prepended[0x01] & 0b00000000) == 0b00000000) {
    return dump_fsk_registers(prepended);
  }
  if ((prepended[0x01] & 0b00100000) == 0b00100000) {
    printf("ook is not supported yet");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}