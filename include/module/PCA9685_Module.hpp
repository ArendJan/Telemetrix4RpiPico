#pragma once
#include <stdlib.h>
#include <vector>

#include "module.hpp"

class PCA9685_Module : public Module {
public:
  PCA9685_Module(std::vector<uint8_t> &data);
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  void updateOne(std::vector<uint8_t> &data, size_t i);
  void resetModule();
  enum REGISTERS : uint8_t {
    LEDn_DIFF = 0x04,
    LEDn_ON_L_base = 0x06,

    PRESCALE = 0xFE,
    MODE_1 = 0x00,
    MODE_1_VAL = 0xA0, // auto increment
    MODE_1_VAL_SLEEP = 0x30
  };

private:
  int i2c_port = 0;
  uint8_t addr = 0b0100'0000;
  bool ok = true;
};