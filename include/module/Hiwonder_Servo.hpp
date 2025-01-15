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
  void resetModule() {
    for (auto &servo : this->servos) {
      servo->motor_mode(0);
      servo->disable();
    }
  };

  enum MessageType : uint8_t {
    // GET_ANGLE
    GET_ANGLE = 0,
    // normal set angle command with one or multiple servos
    SET_ANGLE = 1,
    // enable msg
    ENABLE = 2,
    // update id
    ID_WRITE = 3,
    // id read
    ID_VERIFY = 4,
    // range write
    RANGE_WRITE = 5,
    // read range of servo stored in servo
    RANGE_READ = 6,
    // Set offset in centideg
    OFFSET_WRITE = 7,
    // Read offset
    OFFSET_READ = 8,
    // write voltage limits
    VOLTAGE_LIMIT_WRITE = 9,
    // motor mode write
    MOTOR_MODE_WRITE = 10,
    // Add a servo
    ADD_SERVO = 11,
  };

private:
  HiwonderBus *bus = nullptr;
  std::vector<HiwonderServo *> servos = {};
  int enabled_servos = 0;
};
