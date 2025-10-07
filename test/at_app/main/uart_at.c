#include "uart_at.h"
#include <errno.h>
#include <driver/uart.h>
#include <string.h>
#include <esp_log.h>
#include <sdkconfig.h>
#include <sx127x_at.h>

#ifndef CONFIG_AT_UART_PORT_NUM
#define CONFIG_AT_UART_PORT_NUM UART_NUM_0
#endif

#ifndef CONFIG_AT_UART_BAUD_RATE
#define CONFIG_AT_UART_BAUD_RATE 115200
#endif

#ifndef CONFIG_AT_UART_BUFFER_LENGTH
#define CONFIG_AT_UART_BUFFER_LENGTH 1024
#endif

#ifndef CONFIG_AT_UART_RX_PIN
#define CONFIG_AT_UART_RX_PIN UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_AT_UART_TX_PIN
#define CONFIG_AT_UART_TX_PIN UART_PIN_NO_CHANGE
#endif

#define ERROR_CHECK_ON_CREATE(x)        \
  do {                        \
    esp_err_t __err_rc = (x); \
    if (__err_rc != 0) {                \
      uart_at_handler_destroy(result);                                  \
      return __err_rc;        \
    }                         \
  } while (0)

esp_err_t uart_at_handler_create(int (*extra_callback)(sx127x *device, const char *input, char *output, size_t output_len), sx127x *device, uart_at_handler_t **handler) {
  uart_at_handler_t *result = malloc(sizeof(uart_at_handler_t));
  if (result == NULL) {
    return ESP_ERR_NO_MEM;
  }
  result->uart_port_num = CONFIG_AT_UART_PORT_NUM;
  result->device = device;
  result->extra_callback = extra_callback;
  result->buffer_length = (MAX_PACKET_SIZE_FSK_FIXED * 2 + 128);
  result->buffer = malloc(sizeof(uint8_t) * (result->buffer_length + 1)); // 1 is for \0
  if (result->buffer == NULL) {
    uart_at_handler_destroy(result);
    return ESP_ERR_NO_MEM;
  }
  memset(result->buffer, 0, (result->buffer_length + 1));
  result->temp_buffer = malloc(sizeof(uint8_t) * (result->buffer_length + 1));
  if (result->temp_buffer == NULL) {
    uart_at_handler_destroy(result);
    return ESP_ERR_NO_MEM;
  }
  memset(result->temp_buffer, 0, (result->buffer_length + 1));
  result->output_length = result->buffer_length;
  result->output = malloc(sizeof(char) * (result->output_length + 1));
  if (result->output == NULL) {
    uart_at_handler_destroy(result);
    return ESP_ERR_NO_MEM;
  }
  memset(result->output, 0, result->output_length + 1);

  uart_config_t uart_config = {
      .baud_rate = CONFIG_AT_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ERROR_CHECK_ON_CREATE(uart_driver_install(result->uart_port_num, CONFIG_AT_UART_BUFFER_LENGTH * 2, CONFIG_AT_UART_BUFFER_LENGTH * 2, 20, &result->uart_queue, 0));
  ERROR_CHECK_ON_CREATE(uart_param_config(result->uart_port_num, &uart_config));
  ERROR_CHECK_ON_CREATE(uart_set_pin(result->uart_port_num, CONFIG_AT_UART_TX_PIN, CONFIG_AT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ERROR_CHECK_ON_CREATE(uart_enable_pattern_det_baud_intr(result->uart_port_num, '\n', 1, 9, 0, 0));
  ERROR_CHECK_ON_CREATE(uart_pattern_queue_reset(result->uart_port_num, 20));
  *handler = result;
  return ESP_OK;
}

void uart_at_handler_send(char *output, size_t output_length, void *ctx) {
  uart_at_handler_t *handler = (uart_at_handler_t *) ctx;
  uart_write_bytes(handler->uart_port_num, output, output_length);
}

void uart_at_handler_process(uart_at_handler_t *handler) {
  uart_event_t event;
  size_t current_index = 0;
  int pattern_length = 0;
  while (1) {
    if (xQueueReceive(handler->uart_queue, (void *) &event, (TickType_t) portMAX_DELAY)) {
      switch (event.type) {
        //Event of UART receiving data
        /*We'd better handler data event fast, there would be much more data events than
        other types of events. If we take too much time on data event, the queue might
        be full.*/
        case UART_DATA:
          uart_read_bytes(handler->uart_port_num, handler->buffer + current_index, event.size, portMAX_DELAY);
          current_index += event.size;
          break;
        case UART_PATTERN_DET:
          pattern_length = uart_pattern_pop_pos(handler->uart_port_num);
          if (pattern_length == -1) {
            // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
            // record the position. We should set a larger queue size.
            // As an example, we directly flush the rx buffer here.
            uart_flush_input(handler->uart_port_num);
            current_index = 0;
          } else {
            // 1 is for pattern  - '\n'
            uart_read_bytes(handler->uart_port_num, handler->buffer + current_index, pattern_length + 1, 100 / portTICK_PERIOD_MS);
            current_index += pattern_length + 1;
          }
          break;
          //Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          // If fifo overflow happened, you should consider adding flow control for your application.
          // The ISR has already reset the rx FIFO,
          // As an example, we directly flush the rx buffer here in order to read more data.
          uart_flush_input(handler->uart_port_num);
          xQueueReset(handler->uart_queue);
          current_index = 0;
          break;
        default:
          break;
      }
      if (current_index == 0) {
        continue;
      }
      bool found = false;
      // support for \r, \r\n or \n terminators
      if (handler->buffer[current_index - 1] == '\n') {
        handler->buffer[current_index - 1] = '\0';
        current_index--;
        found = true;
      }
      if (current_index > 0 && handler->buffer[current_index - 1] == '\r') {
        handler->buffer[current_index - 1] = '\0';
        current_index--;
        found = true;
      }
      if (found && current_index > 0) {
        memset(handler->output, 0, handler->output_length + 1);
        strncpy(handler->temp_buffer, handler->buffer, handler->buffer_length);
        // workaround to properly configure "level" interrupt
        if (strcmp(handler->temp_buffer, "AT+OPMOD=SX127X_MODE_RX_CONT,SX127X_MODULATION_FSK") == 0) {
          handler->extra_callback(handler->device, handler->temp_buffer, handler->output, handler->output_length);
        }
        int code = sx127x_at_handler(handler->device, handler->temp_buffer, handler->output, handler->output_length);
        if (code == SX127X_CONTINUE && handler->extra_callback != NULL) {
          strncpy(handler->temp_buffer, handler->buffer, handler->buffer_length);
          code = handler->extra_callback(handler->device, handler->temp_buffer, handler->output, handler->output_length);
        }
        if (code == SX127X_CONTINUE) {
          snprintf(handler->output, handler->output_length, "unsupported command\r\nERROR\r\n");
        }
        size_t actual_output = strlen(handler->output);
        if (actual_output > 0) {
          uart_at_handler_send(handler->output, actual_output, handler);
        }
        current_index = 0;
      }
    }
  }
}

void uart_at_handler_destroy(uart_at_handler_t *handler) {
  if (handler == NULL) {
    return;
  }
  if (handler->buffer != NULL) {
    free(handler->buffer);
  }
  if (handler->output != NULL) {
    free(handler->output);
  }
  if (handler->temp_buffer != NULL) {
    free(handler->temp_buffer);
  }
  uart_driver_delete(handler->uart_port_num);
  free(handler);
}