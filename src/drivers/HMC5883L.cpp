/*

Copied and modified from https://github.com/jarzebski/Arduino-HMC5883L/

HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino
Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "drivers/HMC5883L.hpp"
#include "i2c_helpers.hpp"
bool HMC5883L::begin() {
  // Wire.begin();

  if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48) ||
      (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) ||
      (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33)) {
    return false;
  }

  setRange(HMC5883L_RANGE_1_3GA);
  setMeasurementMode(HMC5883L_CONTINOUS);
  setDataRate(HMC5883L_DATARATE_15HZ);
  setSamples(HMC5883L_SAMPLES_1);

  mgPerDigit = 0.92f;

  return true;
}

Vector HMC5883L::readRaw(void) {
  v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M) - xOffset;
  v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset;
  v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M) - zOffset;

  return v;
}

Vector HMC5883L::readNormalize(void) {
  v.XAxis =
      ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
  v.YAxis =
      ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
  v.ZAxis =
      ((float)readRegister16(HMC5883L_REG_OUT_Z_M) - zOffset) * mgPerDigit;

  return v;
}

void HMC5883L::setOffset(int xo, int yo, int zo) {
  xOffset = xo;
  yOffset = yo;
  zOffset = zo;
}

void HMC5883L::setRange(hmc5883l_range_t range) {
  switch (range) {
  case HMC5883L_RANGE_0_88GA:
    mgPerDigit = 0.073f;
    break;

  case HMC5883L_RANGE_1_3GA:
    mgPerDigit = 0.92f;
    break;

  case HMC5883L_RANGE_1_9GA:
    mgPerDigit = 1.22f;
    break;

  case HMC5883L_RANGE_2_5GA:
    mgPerDigit = 1.52f;
    break;

  case HMC5883L_RANGE_4GA:
    mgPerDigit = 2.27f;
    break;

  case HMC5883L_RANGE_4_7GA:
    mgPerDigit = 2.56f;
    break;

  case HMC5883L_RANGE_5_6GA:
    mgPerDigit = 3.03f;
    break;

  case HMC5883L_RANGE_8_1GA:
    mgPerDigit = 4.35f;
    break;

  default:
    break;
  }

  writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L::getRange(void) {
  return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_MODE);
  value &= 0b11111100;
  value |= mode;

  writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode(void) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_MODE);
  value &= 0b00000011;

  return (hmc5883l_mode_t)value;
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_CONFIG_A);
  value &= 0b11100011;
  value |= (dataRate << 2);

  writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L::getDataRate(void) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_CONFIG_A);
  value &= 0b00011100;
  value >>= 2;

  return (hmc5883l_dataRate_t)value;
}

void HMC5883L::setSamples(hmc5883l_samples_t samples) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_CONFIG_A);
  value &= 0b10011111;
  value |= (samples << 5);

  writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L::getSamples(void) {
  uint8_t value;

  value = readRegister8(HMC5883L_REG_CONFIG_A);
  value &= 0b01100000;
  value >>= 5;

  return (hmc5883l_samples_t)value;
}

Vector HMC5883L::selfTest() {
  Vector value;

  setMeasurementMode(HMC5883L_SINGLE);
  writeRegister8(HMC5883L_REG_CONFIG_A, 0x00);
  writeRegister8(HMC5883L_REG_CONFIG_A, 0x01);
  value = readRaw();
  writeRegister8(HMC5883L_REG_CONFIG_A, 0x00);

  return value;
}

// Write byte to register
void HMC5883L::writeRegister8(uint8_t reg, uint8_t value) {

  auto ok = write_i2c(this->i2c_port, HMC5883L_ADDRESS, {reg, value});

  //     Wire.beginTransmission(HMC5883L_ADDRESS);
  //     #if ARDUINO >= 100
  //         Wire.write(reg);
  //         Wire.write(value);
  //     #else
  //         Wire.send(reg);
  //         Wire.send(value);
  //     #endif
  //     Wire.endTransmission();
}

// Read byte to register
uint8_t HMC5883L::fastRegister8(uint8_t reg) {
  // uint8_t value;
  std::vector<uint8_t> values;
  values.reserve(1);
  auto ok = read_i2c(this->i2c_port, HMC5883L_ADDRESS, {reg}, 1, values);

  return values[0];
}

// Read byte from register
uint8_t HMC5883L::readRegister8(uint8_t reg) {
  std::vector<uint8_t> values;
  values.reserve(1);
  auto ok = read_i2c(this->i2c_port, HMC5883L_ADDRESS, {reg}, 1, values);

  return values[0];
}

// Read word from register
int16_t HMC5883L::readRegister16(uint8_t reg) {
  int16_t value;
  std::vector<uint8_t> values;
  auto ok = read_i2c(this->i2c_port, HMC5883L_ADDRESS, {reg}, 2, values);

  return values[0] << 8 | values[1];
}