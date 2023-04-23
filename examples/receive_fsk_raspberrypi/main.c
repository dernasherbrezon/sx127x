#include <fcntl.h>
#include <inttypes.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sx127x.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <unistd.h>

// Correspond to SPI0 with chip select pin CE0 (GPIO8) on RaspberryPI
#define SPI_DEVICE "/dev/spidev0.0"
#define GPIO_DEVICE "/dev/gpiochip0"
// GPIO 27
#define GPIO_DIO0_PIN 27
#define GPIO_POLL_TIMEOUT -1

#define LINUX_ERROR_CHECK(x)                                                       \
  do {                                                                             \
    int __err_rc = (x);                                                            \
    if (__err_rc != 0) {                                                           \
      fprintf(stderr, "failed at %s:%d code: %d\n", __FILE__, __LINE__, __err_rc); \
      return EXIT_FAILURE;                                                         \
    }                                                                              \
  } while (0)

#define LINUX_NO_CODE_ERROR_CHECK(x)                                               \
  do {                                                                             \
    int __err_rc = (x);                                                            \
    if (__err_rc != 0) {                                                           \
      fprintf(stderr, "failed at %s:%d code: %d\n", __FILE__, __LINE__, __err_rc); \
      return;                                                                      \
    }                                                                              \
  } while (0)

void rx_callback(sx127x *device, uint8_t *data, uint16_t data_length) {
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int16_t rssi;
  LINUX_NO_CODE_ERROR_CHECK(sx127x_rx_get_packet_rssi(device, &rssi));
  int32_t frequency_error;
  LINUX_NO_CODE_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  fprintf(stdout, "received: %d %s rssi: %d freq_error: %" PRId32 "\n", data_length, payload, rssi, frequency_error);
}

int setup_and_wait_for_interrupt(sx127x *device) {
  int fd = open(GPIO_DEVICE, O_RDONLY);
  if (fd < 0) {
    perror("unable to open device");
    return EXIT_FAILURE;
  }
  struct gpioevent_request rq;
  rq.lineoffset = GPIO_DIO0_PIN;
  rq.eventflags = GPIOEVENT_EVENT_RISING_EDGE;
  char label[] = "fsk_raspberry";
  memcpy(rq.consumer_label, label, sizeof(label));
  rq.handleflags = GPIOHANDLE_REQUEST_INPUT;

  int code = ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &rq);
  close(fd);
  if (code < 0) {
    perror("unable to setup gpio interrupt");
    return EXIT_FAILURE;
  }

  struct pollfd pfd;
  pfd.fd = rq.fd;
  pfd.events = POLLIN;
  fprintf(stdout, "waiting for packets...\n");
  while (1) {
    code = poll(&pfd, 1, GPIO_POLL_TIMEOUT);
    if (code < 0) {
      perror("unable to receive gpio interrupt");
      break;
    } else if (pfd.events & POLLIN) {
      sx127x_handle_interrupt(device);
    }
  }
  close(rq.fd);
  return EXIT_SUCCESS;
}

int main() {
  int spi_device_fd = open(SPI_DEVICE, O_RDWR);
  if (spi_device_fd < 0) {
    perror("unable to open device");
    return EXIT_FAILURE;
  }
  int mode = SPI_MODE_0;  // CPOL=0, CPHA=0
  LINUX_ERROR_CHECK(ioctl(spi_device_fd, SPI_IOC_WR_MODE, &mode));
  int bits_per_word = 0;  // means 8 bits
  LINUX_ERROR_CHECK(ioctl(spi_device_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word));
  int lsb_setting = 0;  // MSB
  LINUX_ERROR_CHECK(ioctl(spi_device_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting));
  int max_speed = 500000;
  LINUX_ERROR_CHECK(ioctl(spi_device_fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed));

  sx127x *device = NULL;
  LINUX_ERROR_CHECK(sx127x_create(&spi_device_fd, &device));
  LINUX_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  LINUX_ERROR_CHECK(sx127x_set_frequency(437200012, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, device));
  LINUX_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_auto(true, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_afc_bandwidth(20000.0, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_bandwidth(5000.0, device));
  uint8_t syncWord[] = {0x12, 0xAD};
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  LINUX_ERROR_CHECK(sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_trigger(SX127X_RX_TRIGGER_RSSI_PREAMBLE, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_rssi_config(SX127X_8, 0, device));
  LINUX_ERROR_CHECK(sx127x_fsk_ook_rx_set_preamble_detector(true, 2, 0x0A, device));

  sx127x_rx_set_callback(rx_callback, device);
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_FSK, device));

  return setup_and_wait_for_interrupt(device);
}