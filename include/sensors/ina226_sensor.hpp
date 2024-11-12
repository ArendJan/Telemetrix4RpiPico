#pragma once
#include "sensor.hpp"

#include "drivers/INA226v2.hpp"

class INA226_Sensor : public Sensor {
public:
  INA226_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  INA226_WE *sensor = nullptr;
};