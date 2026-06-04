#pragma once
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "module.hpp"
#include "watchdog.hpp"
#include <vector>

class Shutdown_Relay : public Module {
public:
  Shutdown_Relay(std::vector<uint8_t> &data);
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  void resetModule() {};
  void core1_update() {};

private:
  int pin = 0;
  bool enabled = false;
  bool enable_on = true;
  int wait_time = 10;
  decltype(time_us_32()) start_time = 0;
};
