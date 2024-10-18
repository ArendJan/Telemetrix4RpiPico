#include "module/PCA9685_Module.hpp"

#include "hardware/clocks.h"

#include "i2c_helpers.hpp"

PCA9685_Module::PCA9685_Module(std::vector<uint8_t> &data) {
  // init pca
  // write_i2c(this->i2c_port, 00, {06}); // reset
  sleep_us(100);
  this->i2c_port = data[0];
  auto update_rate = 50;

  if (data.size() == 4) {
    this->addr = data[1];
    update_rate = data[2] << 8 | data[3];
  }
  write_i2c(this->i2c_port, this->addr,
            {MODE_1, MODE_1_VAL_SLEEP}); // go to sleep for prescaler

  const auto clock = 25'000'000;
  uint8_t prescale = (int)((clock) / (4096 * update_rate)) - 1; // For servos
  write_i2c(this->i2c_port, this->addr, {PRESCALE, prescale});
  write_i2c(this->i2c_port, this->addr,
            {MODE_1, MODE_1_VAL}); // restart and auto increment
  sleep_ms(100);
}

void PCA9685_Module::readModule() {
  // no data to read
}

// NOTE: THERE IS A RESET/RESTART COMMAND
// LINK:
// https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/73cf3ecc79c7c33a72f8ce1a3d91ca556cd34ab3/Adafruit_PWMServoDriver.cpp#L87-L93
void PCA9685_Module::resetModule() {
  for (auto i = 0; i < 16; i++) {
    std::vector<uint8_t> data = {(uint8_t)i, 0, 0, 0, 0};
    this->updateOne(data, 0);
  }
}

void PCA9685_Module::updateOne(std::vector<uint8_t> &dataList, size_t i) {
  std::array<uint8_t, 5> data;
  auto LEDn = dataList[0 + i * 5];
  if (LEDn > 15) {
    return;
  }
  data[0] = REGISTERS::LEDn_ON_L_base + LEDn_DIFF * LEDn;
  // outputting data is always low bytes, then high bytes
  data[1] = dataList[1 + i * 5];
  data[2] = dataList[2 + i * 5];
  data[3] = dataList[3 + i * 5];
  data[4] = dataList[4 + i * 5];
  this->ok = write_i2c_t(this->i2c_port, this->addr, data);
}

void PCA9685_Module::writeModule(std::vector<uint8_t> &data) {
  // data format:
  // 0: output #
  // 1 + 2 ON
  // 3+4 OFF
  // update real module
  this->ok = true;
  for (uint8_t i = 0; i < data.size() / 5; i++) {
    this->updateOne(data, i);
    if (!this->ok) {
      return;
    }
  }
}
