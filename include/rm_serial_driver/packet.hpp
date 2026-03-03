// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
#pragma pack(push, 1)

struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  //uint8_t set_mode : 4; // 0-outpost 6-guard 7-base 8-rune 9-auto
  uint8_t game_start : 1;
  uint8_t reserved : 6;
  uint16_t sentryHP;
  uint16_t our_baseHP;
  uint16_t enemy_baseHP;
  uint16_t our_outpostHP;
  uint16_t enemy_outpostHP;
  uint16_t remain_ammo;
  float yaw_imu_q[4];  // x y z w for odom_omni IMU
  float aim_imu_q[4];  // x y z w for odom_aim IMU
  float motor_yaw;     // small yaw motor feedback (rad)
  float motor_pitch;   // pitch motor feedback (rad)
  uint16_t checksum = 0;
};

struct SendAimPacket
{
    uint8_t header = 0xA5;
    uint8_t tracking : 1;
    uint8_t iffire : 1;
    uint8_t id : 4;  // 0-outpost 6-guard 7-base  8-rune
    uint8_t reserved : 2;

    float pitch;
    float big_yaw;     // outer (heavy) yaw
    float small_yaw;   // inner (light) yaw

    uint16_t checksum = 0;
};

struct SendNavPacket
{
  uint8_t header = 0xA6;  // Packet header, fixed value 0xA6

  // Linear velocities
  float linear_x;
  float linear_y;
  float linear_z;

  // Angular velocities
  float angular_x;
  float angular_y;
  float angular_z;

  uint16_t checksum = 0;  // Checksum for error detection
};

struct SendNavPacketV2
{
  uint8_t header = 0xA6;  // Packet header, fixed value 0xA6
  uint8_t follow_mark = 1;  // Follow-mark status from /chassis/follow_mark
  // Linear velocities
  float linear_x;
  float linear_y;
  float linear_z;

  // Angular velocities
  float angular_x;
  float angular_y;
  float angular_z;

  uint16_t checksum = 0;    // Checksum for error detection
};

constexpr std::size_t kSendNavPacketV2ExpectedSize = 28U;
static_assert(
  sizeof(SendNavPacketV2) == kSendNavPacketV2ExpectedSize,
  "SendNavPacketV2 layout mismatch");


struct SendDecisionPacket
{
  uint8_t header = 0xA7;  // Packet header, fixed value 0xA5

  // Decision values
  uint8_t ifreload;

  uint16_t checksum = 0;  // Checksum for error detection
};

#pragma pack(pop)

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
