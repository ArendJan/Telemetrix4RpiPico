#pragma once
#include "sensor.hpp"
#include <stdlib.h>
#include <vector>
#include "Adafruit_BNO055.h"
class BNO055_Sensor : public Sensor {
public:
  BNO055_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  Adafruit_BNO055 *bno;
};