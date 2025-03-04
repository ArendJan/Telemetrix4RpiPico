#include "serialization.hpp"
#include <bit>

std::vector<uint8_t> encode_u16(const uint16_t &value) {
  return {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
}

std::vector<uint8_t> encode_u32(const uint32_t &value) {
  return {(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8),
          (uint8_t)(value & 0xFF)};
}

std::vector<uint8_t> encode_u64(const uint64_t &value) {
  return {(uint8_t)(value >> 56), (uint8_t)(value >> 48),
          (uint8_t)(value >> 40), (uint8_t)(value >> 32),
          (uint8_t)(value >> 24), (uint8_t)(value >> 16),
          (uint8_t)(value >> 8),  (uint8_t)(value & 0xFF)};
}

std::vector<uint8_t> encode_i16(const int16_t &value) {
  auto converted = *reinterpret_cast<uint16_t *>(value);
  return encode_u16(converted);
}

std::vector<uint8_t> encode_i32(const int32_t &value) {
  auto converted = *reinterpret_cast<uint32_t *>(value);
  return encode_u32(converted);
}

std::vector<uint8_t> encode_i64(const int64_t &value) {
  auto converted = *reinterpret_cast<uint64_t *>(value);
  return encode_u64(converted);
}

// std::vector<uint8_t> encode_float(const float & value) {
//   return encode_u32(std::bit_cast<uint32_t>(value));
// }

uint16_t decode_u16(const std::span<const uint8_t, 2> &data) {
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

uint32_t decode_u32(const std::span<const uint8_t, 4> &data) {
  return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

uint64_t decode_u64(const std::span<const uint8_t, 8> &data) {
  return ((uint64_t)data[0] << 56) | ((uint64_t)data[1] << 48) |
         ((uint64_t)data[2] << 40) | ((uint64_t)data[3] << 32) |
         ((uint64_t)data[4] << 24) | ((uint64_t)data[5] << 16) |
         ((uint64_t)data[6] << 8) | (uint64_t)data[7];
}

int16_t decode_i16(const std::span<const uint8_t, 2> &data) {
  auto value = decode_u16(data);
  return *reinterpret_cast<int16_t *>(&value);
}

int32_t decode_i32(const std::span<const uint8_t, 4> &data) {
  auto value = decode_u32(data);
  return *reinterpret_cast<int32_t *>(&value);
}

int64_t decode_i64(const std::span<const uint8_t, 8> &data) {
  auto value = decode_u64(data);
  return *reinterpret_cast<int64_t *>(&value);
}

// float decode_float(const std::span<const uint8_t, 4> & data) {
//   return std::bit_cast<float>(decode_u32(data));
// }