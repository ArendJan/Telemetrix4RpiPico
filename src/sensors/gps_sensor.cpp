#include "sensors/gps_sensor.hpp"

#include "hardware/gpio.h"

GPS_Sensor::GPS_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  auto rx = settings[0];
  // auto tx = settings[1];
  auto uart = settings[2];
  this->uart_id = uart0;
  if (uart) {
    this->uart_id = uart1;
  }

  const auto DATA_BITS = 8;
  const auto STOP_BITS = 1;
  const auto PARITY = UART_PARITY_NONE;

  uart_init(uart_id, 9600);
  // gpio_set_function(tx, GPIO_FUNC_UART); // No need to send anything to the
  // GPS receiver
  gpio_set_function(rx, GPIO_FUNC_UART);
  uart_set_hw_flow(uart_id, false, false);
  // Set our data format
  uart_set_fifo_enabled(uart_id, true);

  uart_set_format(uart_id, DATA_BITS, STOP_BITS, PARITY);
}

void GPS_Sensor::readSensor() {
  // Is this fast enough? fifo is 32 bytes long, unknown speed of gps module
  // baud = 9600 -> max ~960 bytes/s. Sensor interval is 50-100Hz -> 1500-3200
  // bytes/s, is okay.

  std::vector<uint8_t> data;
  if (uart_is_readable(this->uart_id)) {
    while (uart_is_readable(this->uart_id)) {
      data.push_back(uart_getc(this->uart_id));
    }
    this->writeSensorData(data);
  }
}
