#include "Telemetrix4RpiPico.hpp"
#include "pico/stdio.h"


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
  // stdio_out_chars_crlf((char *)data.data(), data.size());
  // stdio_usb.out_chars((char *)data.data(), data.size());
  for (auto byte : data) {
    putchar_raw(byte);
  }
  if (uart_enabled) {
    for (auto byte : data) {
      uart_putc_raw(UART_ID, byte);
    }
  }
}

void put_byte(uint8_t byte) {
  if (uart_enabled) {
    uart_putc_raw(UART_ID, byte);
  }
  // stdio_usb.out_chars(&byte, 1);
  putchar_raw(byte);
}

int get_byte() {
  // If there is no uart loopback, then also check the uart for incoming data.
  if (uart_enabled) {
    if (uart_is_readable(UART_ID)) {
      return uart_getc(UART_ID);
    }
  }
  return getchar_timeout_us(100);
}