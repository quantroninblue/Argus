#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Core>

namespace argus::grid {

// Little-endian byte packing for map payloads. Readers throw std::invalid_argument when the
// payload runs out, so callers can decode untrusted data directly.
class Writer {
 public:
  void u8(uint8_t v) { out_.push_back(static_cast<char>(v)); }
  void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
  void u16(uint16_t v) { put(v, 2); }
  void u32(uint32_t v) { put(v, 4); }
  void i32(int32_t v) { put(static_cast<uint32_t>(v), 4); }
  void u64(uint64_t v) { put(v, 8); }
  void bytes(const void* data, size_t n) { out_.append(static_cast<const char*>(data), n); }
  std::string take() { return std::move(out_); }

 private:
  void put(uint64_t v, int n) {
    for (int k = 0; k < n; ++k) {
      out_.push_back(static_cast<char>((v >> (8 * k)) & 0xff));
    }
  }
  std::string out_;
};

class Reader {
 public:
  explicit Reader(const std::string& in) : in_(in) {}
  uint8_t u8() { return static_cast<uint8_t>(get(1)); }
  int8_t i8() { return static_cast<int8_t>(u8()); }
  uint16_t u16() { return static_cast<uint16_t>(get(2)); }
  uint32_t u32() { return static_cast<uint32_t>(get(4)); }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  uint64_t u64() { return get(8); }
  // Three i32 in order (function arguments have no defined evaluation order).
  Eigen::Vector3i index3() {
    const int32_t x = i32();
    const int32_t y = i32();
    const int32_t z = i32();
    return Eigen::Vector3i(x, y, z);
  }
  void bytes(void* data, size_t n) {
    need(n);
    std::memcpy(data, in_.data() + pos_, n);
    pos_ += n;
  }
  bool done() const { return pos_ == in_.size(); }

 private:
  void need(size_t n) const {
    if (in_.size() - pos_ < n) {
      throw std::invalid_argument("payload truncated");
    }
  }
  uint64_t get(int n) {
    need(static_cast<size_t>(n));
    uint64_t v = 0;
    for (int k = 0; k < n; ++k) {
      v |= static_cast<uint64_t>(static_cast<uint8_t>(in_[pos_ + k])) << (8 * k);
    }
    pos_ += static_cast<size_t>(n);
    return v;
  }
  const std::string& in_;
  size_t pos_ = 0;
};

}  // namespace argus::grid
