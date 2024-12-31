// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float q[4]; // x y z w
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendAimPacket
{
  uint8_t header = 0xA5;
  uint8_t tracking : 1;
  uint8_t iffire : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t reserved : 3;
  
  float pitch;
  float yaw;
  
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendNavPacket
{
  uint8_t header = 0xA6;  // Packet header, fixed value 0xA5

  // Linear velocities
  float linear_x;
  float linear_y;
  float linear_z;

  // Angular velocities
  float angular_x;
  float angular_y;
  float angular_z;

  uint16_t checksum = 0;  // Checksum for error detection
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(T), packet.begin());
  return packet;
}


}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
