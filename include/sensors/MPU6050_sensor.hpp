#pragma once
#include "drivers/MPU6050.hpp"
#include "sensor.hpp"
#include <stdlib.h>
#include <vector>
class MPU6050_Module : public Sensor {
public:
  MPU6050_Module(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor();

private:
  MPU6050 *mpu;
};