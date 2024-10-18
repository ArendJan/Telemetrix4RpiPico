#pragma once
#include <stdlib.h>
#include <vector>

#include "drivers/HiwonderServo.hpp"

#include "module.hpp"

class Hiwonder_Servo : public Module {
public:
  Hiwonder_Servo(std::vector<uint8_t> &data);
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  bool writeSingle(std::vector<uint8_t> &data, size_t i, bool single);
  void resetModule(){};

private:
  HiwonderBus *bus = nullptr;
  std::vector<HiwonderServo *> servos = {};
  int enabled_servos = 0;
};