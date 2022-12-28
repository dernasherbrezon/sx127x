#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <sx127x.h>

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

sx127x *device = NULL;

int current_power = 0;
int messages_current_power = 0;

void tx_callback(sx127x *device) {
  Serial.printf("transmitted\n");
  if (current_power > 20) {
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_pa_config(SX127x_PA_PIN_BOOST, current_power, device));

  uint8_t data[] = {0xCA, 0xFE};
  ESP_ERROR_CHECK(sx127x_set_for_transmission(data, sizeof(data), device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, device));
  Serial.printf("transmitting %d %d\n", current_power, messages_current_power);
  messages_current_power++;
  if (messages_current_power > 5) {
    current_power++;
    messages_current_power = 0;
  }
  if (current_power == 18) {
    current_power = 20;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting up");
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  esp_err_t code = spi_bus_initialize(HSPI_HOST, &config, 0);
  if (code != ESP_OK) {
    Serial.println("can't init bus");
    return;
  }
  code = sx127x_create(HSPI_HOST, SS, &device);
  if (code != ESP_OK) {
    Serial.println("can't create device");
    return;
  }

  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, device));
  ESP_ERROR_CHECK(sx127x_set_frequency(437200012, device));
  ESP_ERROR_CHECK(sx127x_reset_fifo(device));
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, device));
  ESP_ERROR_CHECK(sx127x_set_bandwidth(SX127x_BW_125000, device));
  ESP_ERROR_CHECK(sx127x_set_implicit_header(NULL, device));
  ESP_ERROR_CHECK(sx127x_set_modem_config_2(SX127x_SF_9, device));
  ESP_ERROR_CHECK(sx127x_set_syncword(18, device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(8, device));
  sx127x_set_tx_callback(tx_callback, device);

  gpio_pad_select_gpio(DIO0);
  gpio_set_direction((gpio_num_t)DIO0, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)DIO0);
  gpio_pullup_dis((gpio_num_t)DIO0);
  gpio_set_intr_type((gpio_num_t)DIO0, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)DIO0, sx127x_handle_interrupt_fromisr, (void *)device);

  sx127x_tx_header_t header;
  header.crc = SX127x_RX_PAYLOAD_CRC_ON;
  header.coding_rate = SX127x_CR_4_5;
  ESP_ERROR_CHECK(sx127x_set_tx_explcit_header(&header, device));
  current_power = 2;
  tx_callback(device);
}

void loop() {
  delay(100000);
}
