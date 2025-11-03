#pragma once
#include <span>
#include <stdlib.h>
#include <vector>

#include "drivers/HiwonderServo.hpp"

#include "module.hpp"

#include <pico/sync.h>
class HiwonderServoItem {
public:
  HiwonderServo servo;
  volatile bool updated_read = false;
  volatile bool updated_write = false;
  volatile int32_t lastreadPosition = 0;
  volatile int32_t lastwritePosition = 0;
  volatile int32_t write_time = 0;
  bool disabled = false;
  int fault_count = 0;
  mutex_t mutex; // for reading and writing updated and position

  // Used for telemetrix to only publish on change:
  int32_t lastPublishedPosition = 0;
  HiwonderServoItem(HiwonderBus *bus, int id) : servo(bus, id) {}
};

class Hiwonder_Servo : public Module {
public:
  Hiwonder_Servo(std::vector<uint8_t> &data);
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  bool writeSingle(std::vector<uint8_t> &data, size_t i, bool single);
  void core1_update();
  void resetModule() {
    for (auto &servo : this->servos) {
      servo->servo.motor_mode(0);
      servo->servo.disable();
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
  mutex_t bus_mutex;
  HiwonderBus *bus = nullptr;
  std::vector<HiwonderServoItem *> servos = {};
  volatile int enabled_servos = 0;
  decltype(time_us_32()) last_force_write = 0;
};
