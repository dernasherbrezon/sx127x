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
#include <time.h>
#include <errno.h>

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
  float snr;
  LINUX_NO_CODE_ERROR_CHECK(sx127x_lora_rx_get_packet_snr(device, &snr));
  int32_t frequency_error;
  LINUX_NO_CODE_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  fprintf(stdout, "received: %d %s rssi: %d snr: %f freq_error: %" PRId32 "\n", data_length, payload, rssi, snr, frequency_error);
}

void cad_callback(sx127x *device, int cad_detected) {
  if (cad_detected == 0) {
    fprintf(stdout, "cad not detected\n");
    LINUX_NO_CODE_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
    return;
  }
  // put into RX mode first to handle interrupt as soon as possible
  LINUX_NO_CODE_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  fprintf(stdout, "cad detected\n");
}

int msleep(long msec) {
  struct timespec ts;
  int res;

  if (msec < 0) {
    errno = EINVAL;
    return EXIT_FAILURE;
  }

  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;

  do {
    res = nanosleep(&ts, &ts);
  } while (res && errno == EINTR);

  return res;
}

int gpio_write_value(int fd, int value) {
  struct gpiohandle_data data;
  data.values[0] = value;
  int code = ioctl(fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (code < 0) {
    perror("unable to write value");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int reset_sx127x() {
  int fd = open(GPIO_DEVICE, O_RDONLY);
  if (fd < 0) {
    perror("unable to open device");
    return EXIT_FAILURE;
  }
  struct gpiohandle_request rq;
  rq.lineoffsets[0] = 6;
  rq.lines = 1;
  rq.flags = GPIOHANDLE_REQUEST_OUTPUT;
  strcpy(rq.consumer_label, "sx127x_reset");
  int code = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
  if (code < 0) {
    perror("unable to reset chip");
    return EXIT_FAILURE;
  }
  close(fd);

  LINUX_ERROR_CHECK(gpio_write_value(rq.fd, 1));
  LINUX_ERROR_CHECK(gpio_write_value(rq.fd, 0));
  msleep(5);
  LINUX_ERROR_CHECK(gpio_write_value(rq.fd, 1));
  msleep(5);
  close(rq.fd);
  return EXIT_SUCCESS;
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
  char label[] = "lora_raspberry";
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
  char buffer[32] = {0};
  while (1) {
    code = poll(&pfd, 1, GPIO_POLL_TIMEOUT);
    if (code < 0) {
      perror("unable to receive gpio interrupt");
      break;
    }
    if (pfd.events & POLLIN) {
      // discard data
      read(pfd.fd, buffer, 32);
    }
    sx127x_handle_interrupt(device);
  }
  close(rq.fd);
  return EXIT_SUCCESS;
}

int main() {
  LINUX_ERROR_CHECK(reset_sx127x());

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
  int max_speed = 8000000;
  LINUX_ERROR_CHECK(ioctl(spi_device_fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed));

  sx127x *device = NULL;
  LINUX_ERROR_CHECK(sx127x_create(&spi_device_fd, &device));
  LINUX_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
  LINUX_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device));
  LINUX_ERROR_CHECK(sx127x_set_frequency(437200000, device));
  LINUX_ERROR_CHECK(sx127x_lora_set_ppm_offset(4000, device));
  LINUX_ERROR_CHECK(sx127x_lora_reset_fifo(device));
  LINUX_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, device));
  LINUX_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, device));
  LINUX_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, device));
  LINUX_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, device));
  LINUX_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, device));
  LINUX_ERROR_CHECK(sx127x_lora_set_syncword(18, device));
  LINUX_ERROR_CHECK(sx127x_set_preamble_length(8, device));
  sx127x_rx_set_callback(rx_callback, device);
  sx127x_lora_cad_set_callback(cad_callback, device);
  LINUX_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));

  return setup_and_wait_for_interrupt(device);
}