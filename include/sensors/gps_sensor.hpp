#pragma once
#include "sensor.hpp"

#include "hardware/uart.h"

class GPS_Sensor : public Sensor {
public:
  GPS_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor(){};

private:
  uart_inst_t *uart_id = uart0;
};