#ifndef LORA_AT_UART_AT_H
#define LORA_AT_UART_AT_H

#include <esp_err.h>
#include <sx127x.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct {
  int uart_port_num;
  char *buffer;
  sx127x *device;
  QueueHandle_t uart_queue;

  int (*extra_callback)(sx127x *device, const char *input, char *output, size_t output_len);
} uart_at_handler_t;

esp_err_t uart_at_handler_create(int (*extra_callback)(sx127x *device, const char *input, char *output, size_t output_len), sx127x *device, uart_at_handler_t **result);

void uart_at_handler_process(uart_at_handler_t *handler);

void uart_at_handler_destroy(uart_at_handler_t *handler);

void uart_at_handler_send(char *output, size_t output_length, void *handler);

#endif //LORA_AT_UART_AT_H
