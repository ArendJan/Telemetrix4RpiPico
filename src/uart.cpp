#include "Telemetrix4RpiPico.hpp"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"
/*************************************************
 * Write data to serial interface
 * @param buffer
 * @param num_of_bytes_to_send
 */
void serial_write(const int *buffer, int num_of_bytes_to_send) {

  for (int i = 0; i < num_of_bytes_to_send; i++) {
    put_byte((buffer[i]) & 0x00ff);
  }
  stdio_flush();
}

void serial_write(std::vector<uint8_t> data) {
  data[0] = data.size() - 1;
  // // stdio_out_chars_crlf((char *)data.data(), data.size());
  // // stdio_usb.out_chars((char *)data.data(), data.size());

  for (auto byte : data) {
    putchar_raw(byte);
    // uart_putc_raw(UART_ID, byte);
  }
}

void put_byte(uint8_t byte) { putchar_raw(byte); }

int get_byte() {
  // // If there is no uart loopback, then also check the uart for incoming
  // data. if (uart_enabled) {
  //   if (uart_is_readable(UART_ID)) {
  //     return uart_getc(UART_ID);
  //   }
  // }
  return getchar_timeout_us(10);
}

void init_uart_port() {

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_fifo_enabled(UART_ID, true);
}

volatile bool uart_enabled = true;

void check_uart_loopback() {
  // return;
  sleep_ms(10);
  init_uart_port();
  // If we read back the same message as sent, then there is a loopback
  // and disable the uart for normal Telemetrix communication.
  while (uart_is_readable(UART_ID)) {
    (void)uart_getc(UART_ID);
    // empty the uart.
  }
  uint8_t test_message = 10; // send a null character, this shouldn't interfere
                             // with the tmx-pico-aio implementation.
  uint8_t read_byte = 123;

  uart_putc_raw(UART_ID, test_message);
  sleep_ms(100);

  if (uart_is_readable(UART_ID)) {

    read_byte = uart_getc(UART_ID);
    if (read_byte == test_message) {
      uart_enabled = false;
      // return;
    }
  }
  if (uart_enabled) {
    stdio_uart_init();
    stdio_set_translate_crlf(&stdio_uart, false);
    stdio_set_driver_enabled(&stdio_uart, true);
    stdio_flush();
  }
}