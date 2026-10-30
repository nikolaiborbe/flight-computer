#pragma once

#include <cstdint>
#include <cstring>

namespace proto::pack {

inline void wr_u16(uint8_t* d, const int offset, const uint16_t v) {
  d[offset + 0] = static_cast<uint8_t>(v & 0xFFu);
  d[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline uint16_t rd_u16(const uint8_t* d, const int offset) {
  return static_cast<uint16_t>(d[offset + 0]) |
       (static_cast<uint16_t>(d[offset + 1]) << 8);
}

inline void wr_i16(uint8_t* d, const int offset, const int16_t v) {
  wr_u16(d, offset, static_cast<uint16_t>(v));
}

inline int16_t rd_i16(const uint8_t* d, const int offset) {
  return static_cast<int16_t>(rd_u16(d, offset));
}

inline void wr_u32(uint8_t* d, const int offset, const uint32_t v) {
    d[offset + 0] = static_cast<uint8_t>(v & 0xFFu);
    d[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    d[offset + 2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    d[offset + 3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

inline uint32_t rd_u32(const uint8_t* d, const int offset) {
    return static_cast<uint32_t>(d[offset + 0]) |
           (static_cast<uint32_t>(d[offset + 1]) << 8) |
           (static_cast<uint32_t>(d[offset + 2]) << 16) |
           (static_cast<uint32_t>(d[offset + 3]) << 24);
}

inline void wr_f32(uint8_t* d, const int offset, const float v) {
  uint32_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  d[offset + 0] = static_cast<uint8_t>( u        & 0xFFu);
  d[offset + 1] = static_cast<uint8_t>((u >> 8)  & 0xFFu);
  d[offset + 2] = static_cast<uint8_t>((u >> 16) & 0xFFu);
  d[offset + 3] = static_cast<uint8_t>((u >> 24) & 0xFFu);
}

inline float rd_f32(const uint8_t* d, const int offset) {
  uint32_t u = 0;
  u |= static_cast<uint32_t>(d[offset + 0]);
  u |= static_cast<uint32_t>(d[offset + 1]) << 8;
  u |= static_cast<uint32_t>(d[offset + 2]) << 16;
  u |= static_cast<uint32_t>(d[offset + 3]) << 24;
  float v = 0.0f;
  std::memcpy(&v, &u, sizeof(v));
  return v;
}

inline void wr_u64(uint8_t* d, const int offset, const uint64_t v) {
  d[offset + 0] = static_cast<uint8_t>( v        & 0xFFu);
  d[offset + 1] = static_cast<uint8_t>((v >> 8)  & 0xFFu);
  d[offset + 2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
  d[offset + 3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
  d[offset + 4] = static_cast<uint8_t>((v >> 32) & 0xFFu);
  d[offset + 5] = static_cast<uint8_t>((v >> 40) & 0xFFu);
  d[offset + 6] = static_cast<uint8_t>((v >> 48) & 0xFFu);
  d[offset + 7] = static_cast<uint8_t>((v >> 56) & 0xFFu);
}

inline uint64_t rd_u64(const uint8_t* d, const int offset) {
  return static_cast<uint64_t>(d[offset + 0]) |
         (static_cast<uint64_t>(d[offset + 1]) << 8) |
         (static_cast<uint64_t>(d[offset + 2]) << 16) |
         (static_cast<uint64_t>(d[offset + 3]) << 24) |
         (static_cast<uint64_t>(d[offset + 4]) << 32) |
         (static_cast<uint64_t>(d[offset + 5]) << 40) |
         (static_cast<uint64_t>(d[offset + 6]) << 48) |
         (static_cast<uint64_t>(d[offset + 7]) << 56);
}

}
