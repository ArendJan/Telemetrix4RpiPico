#include "module/Hiwonder_Servo.hpp"

#include "message_types.hpp"
#include "serialization.hpp"

#include "Telemetrix4RpiPico.hpp"

/**
 * When creating a servo chain:
 * Byte1: 0: uart0, 1: uart1
 * Byte2: rx pin
 * Byte3: tx pin
 * Byte4: wanted amount of servos
 * Byte5-N: ids of servos
 */
Hiwonder_Servo::Hiwonder_Servo(std::vector<uint8_t> &data) {

  auto uart = data[0] == 0 ? uart0 : uart1;
  auto rxPin = data[1];
  auto txPin = data[2];
  auto servos = data[3];
  if (data.size() != (uint8_t)(servos + 4)) {
    return;
  }
  this->bus = new HiwonderBus();

  this->bus->begin(uart, rxPin, txPin);
  this->servos.reserve(servos);
  for (auto i = 0; i < servos; i++) {
    auto id = data[4 + i];
    auto servo = new HiwonderServo(this->bus, id);
    servo->initialize();
    // auto offset = servo->read_angle_offset();
    this->servos.push_back(servo);
    this->enabled_servos++;
  }
  this->bus->enableAll();
}

bool Hiwonder_Servo::writeSingle(std::vector<uint8_t> &data, size_t i,
                                 bool single) {
  const auto offset = 2; // 1 for msg_type, 1 for count
  const int numBytes = 5;
  auto data_span =
      std::span(data).subspan(offset + numBytes * i).first<numBytes>();
  auto servoI = data_span[0];
  // TODO: What happens here?
  // TODO: Maybe this needs to decode a i16
  auto angle = (int32_t)decode_u16(data_span.subspan<1, sizeof(uint16_t)>());
  // ((int32_t)data[offset + 1 + numBytes * i] << 8) |
  //        data[offset + 2 + numBytes * i];
  auto time = decode_u16(data_span.subspan<3, sizeof(uint16_t)>());

  if (servoI >= this->servos.size()) {
    return false;
  }
  if (single) {
    this->servos[servoI]->move_time(angle, time);
  } else {
    this->servos[servoI]->move_time_and_wait_for_sync(angle, time);
  }

  // repair servo if it was disabled
  if (this->servos[servoI]->isCommandOk() && this->servos[servoI]->disabled) {
    this->servos[servoI]->fault_count = 0;
    this->servos[servoI]->disabled = false;
    this->enabled_servos++;
  }
  return true;
}

void Hiwonder_Servo::writeModule(std::vector<uint8_t> &data) {
  auto data_span = std::span(data);
  auto msg_type = data[0];
  if (!timeout_safe()) {
    return;
  }
  if (msg_type ==
      SET_ANGLE) { // normal set angle command with one or multiple servos
    auto count = data[1];
    // If just one, directly move, otherwise wait for the other commands to
    // finish before moving
    if (count == 1) {
      this->writeSingle(data, 0, true);
    } else {
      if (data.size() != (uint8_t)(count * 5 + 2)) {
        return;
      }
      for (auto i = 0; i < count; i++) {
        this->writeSingle(data, i, false);
      }
      this->bus->move_sync_start();
    }
    // TODO: Add Ok Send?
  } else if (msg_type == ENABLE) { // enable msg
    auto count = data[1];
    auto enabled = data[2];
    if (count == 0) { // all servos
      if (enabled) {
        this->bus->enableAll();
      } else {
        this->bus->disableAll();
      }
      return;
    }
    for (auto i = 0; i < count; i++) {
      auto servoI = data[3 + i];
      if (servoI >= this->servos.size()) {
        return;
      }
      if (enabled == 1) {
        this->servos[servoI]->enable();
      } else {
        this->servos[servoI]->disable();
      }
    }
  } else if (msg_type == ID_WRITE) { // update id
    auto new_id = data[1];
    this->bus->id_write(new_id);
  } else if (msg_type == ID_VERIFY) {
    // id read
    auto check_id = data[1];
    HiwonderServo tempServo(this->bus, check_id);
    bool ok = tempServo.id_verify() == check_id;
    std::vector<uint8_t> data = {
        ID_VERIFY,   // id check type
        check_id,    // id
        (uint8_t)ok, // ok
    };
    this->publishData(data);
  } else if (msg_type == RANGE_WRITE) {
    // range write
    auto id = data[1];
    int16_t min = decode_u16(
        data_span.subspan<2, sizeof(uint16_t)>()); //((int16_t)data[2]
                                                   //<< 8) | data[3];
    int16_t max = decode_u16(
        data_span.subspan<4, sizeof(uint16_t)>()); //((int16_t)data[4]
                                                   //<< 8) | data[5];
    this->servos[id]->setLimitsTicks(min / 24,
                                     max / 24); // 24 centidegrees per tick
  } else if (msg_type == RANGE_READ) {
    // read range of servo stored in servo
    auto id = data[1];
    this->servos[id]->readLimits();
    auto min = this->servos[id]->minCentDegrees;
    auto max = this->servos[id]->maxCentDegrees;
    std::vector<uint8_t> data = {RANGE_READ, // id check type
                                 id};        //, // id
    // data.reserve(data.size() + 2 * sizeof(uint16_t));

    append_range(data, encode_u16((uint16_t)min));
    append_range(data, encode_u16((uint16_t)max));

    this->publishData(data);
  } else if (msg_type == OFFSET_WRITE) { // Set offset in centideg
    auto id = data[1];
    // TODO: Maybe i16 decode instead
    int16_t offset = (int16_t)decode_u16(
        data_span.subspan<2, sizeof(uint16_t)>()); //((int16_t)data[2] << 8) |
                                                   // data[3];
    offset /= 24;
    this->servos[id]->angle_offset_adjust(offset);
    this->servos[id]->angle_offset_save();
  } else if (msg_type == OFFSET_READ) {
    auto id = data[1];
    auto offset = this->servos[id]->read_angle_offset() * 24;
    std::vector<uint8_t> data = {OFFSET_READ, // offset type
                                 id};         // id
    append_range(data, encode_u16(offset));
    this->publishData(data);
  } else if (msg_type == VOLTAGE_LIMIT_WRITE) { // write voltage limits
    auto id = data[1];
    // TODO: Shouldn't this use more bytes?
    uint32_t vMin = decode_u16(data_span.subspan<2, sizeof(uint16_t)>());
    uint32_t vMax = decode_u16(data_span.subspan<4, sizeof(uint16_t)>());
    this->servos[id]->setVoltageLimits(vMin, vMax);
  }
}

void Hiwonder_Servo::readModule() {
  if (!timeout_safe()) {
    return;
  }
  if (this->enabled_servos == 0) {
    return;
  }
  // read angle, temp?
  std::vector<uint8_t> data;
  data.reserve(this->servos.size() * 3 + 1);
  data.push_back(GET_ANGLE); // message type servo angles
  // only update position when changed
  for (auto i = 0; auto servo : this->servos) {
    if (servo->disabled) { // skip disabled servos, they are very slow
      i++;
      continue;
    }
    auto pos = servo->pos_read();
    if (servo->isCommandOk()) {
      servo->fault_count = 0;
    } else {
      servo->fault_count++;
      if (servo->fault_count > 2) {
        servo->disabled = true;
        this->enabled_servos--;
      }
    }
    if (pos != servo->lastPublishedPosition) {
      data.push_back(i);
      // Pos is 0...24000 -> 15 bits
      append_range(data, encode_u16(pos));
    }
    servo->lastPublishedPosition = pos;
    i++;
  }
  if (data.size() > 1) {
    this->publishData(data);
  }
}
