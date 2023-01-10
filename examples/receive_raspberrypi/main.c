#include <errno.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <sx127x.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#define SPI_DEVICE "/dev/spidev1.1"
#define GPIO_DEVICE "/dev/gpiochip0"
#define GPIO_DIO0_PIN 17
#define GPIO_POLL_TIMEOUT -1

void rx_callback(sx127x *device) {
  uint8_t *data = NULL;
  uint8_t data_length = 0;
  int code = sx127x_read_payload(device, &data, &data_length);
  if (code != SX127X_OK) {
    fprintf(stderr, "can't read %d", code);
    return;
  }
  if (data_length == 0) {
    // no message received
    return;
  }
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int16_t rssi;
  code = sx127x_get_packet_rssi(device, &rssi);
  if (code != SX127X_OK) {
    fprintf(stderr, "can't read rssi %d", code);
  }
  float snr;
  code = sx127x_get_packet_snr(device, &snr);
  if (code != SX127X_OK) {
    fprintf(stderr, "can't read snr %d", code);
  }
  int32_t frequency_error;
  code = sx127x_get_frequency_error(device, &frequency_error);
  if (code != SX127X_OK) {
    fprintf(stderr, "can't read frequency error %d", code);
  }

  fprintf(stdout, "received: %d %s rssi: %d snr: %f freq_error: %d", data_length, payload, rssi, snr, frequency_error);
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

  int code = ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &rq);
  close(fd);
  if (code < 0) {
    perror("unable to setup gpio interrupt");
    return EXIT_FAILURE;
  }

  struct pollfd pfd;
  pfd.fd = rq.fd;
  pfd.events = POLLIN;
  code = poll(&pfd, 1, GPIO_POLL_TIMEOUT);
  if (code < 0) {
    perror("unable to receive gpio interrupt");
  } else if (pfd.events & POLLIN) {
    sx127x_handle_interrupt(device);
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
  // FIXME SPI_IOC_WR_MODE
  // FIXME SPI_IOC_RD_MODE
  // FIXME SPI_IOC_WR_BITS_PER_WORD
  // FIXME SPI_IOC_RD_BITS_PER_WORD
  // FIXME SPI_IOC_WR_MAX_SPEED_HZ
  // FIXME SPI_IOC_RD_MAX_SPEED_HZ
  sx127x *device = NULL;
  sx127x_create(&spi_device_fd, &device);
  sx127x_set_opmod(SX127x_MODE_SLEEP, device);
  sx127x_set_frequency(437200012, device);
  sx127x_reset_fifo(device);
  sx127x_set_lna_boost_hf(SX127x_LNA_BOOST_HF_ON, device);
  sx127x_set_opmod(SX127x_MODE_STANDBY, device);
  sx127x_set_lna_gain(SX127x_LNA_GAIN_G4, device);
  sx127x_set_bandwidth(SX127x_BW_125000, device);
  sx127x_set_implicit_header(NULL, device);
  sx127x_set_modem_config_2(SX127x_SF_9, device);
  sx127x_set_syncword(18, device);
  sx127x_set_preamble_length(8, device);
  sx127x_set_rx_callback(rx_callback, device);
  // FIXME better poll on a separate thread and start RX only when started
  sx127x_set_opmod(SX127x_MODE_RX_CONT, device);

  return setup_and_wait_for_interrupt(device);
}