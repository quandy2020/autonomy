/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file rrc_protocol.hpp
 * @brief Hiwonder RRC host serial framing (non-ROS) for JetAuto STM32 board.
 *
 * Frame: 0xAA 0x55 | func | len | data[len] | checksum
 * Checksum: low byte of bitwise complement of (func + len + sum(data)).
 * Motor multi-set: PACKET_FUNC_MOTOR=3, subcommand 0x01 (5N+2 payload).
 *
 * Spec: ROS Robot Control Board §3.14 host protocol (Hiwonder wiki).
 */

#ifndef AUTODRIVER_CHASSIS_JETAUTO_RRC_PROTOCOL_HPP_
#define AUTODRIVER_CHASSIS_JETAUTO_RRC_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autodriver {
namespace chassis {
namespace jetauto {

/** @brief RRC packet function: encoder motor control. */
inline constexpr std::uint8_t kPacketFuncMotor = 3;
/** @brief Frame header byte 0. */
inline constexpr std::uint8_t kStartByte1 = 0xAA;
/** @brief Frame header byte 1. */
inline constexpr std::uint8_t kStartByte2 = 0x55;
/** @brief Multi-motor set-speed subcommand. */
inline constexpr std::uint8_t kMotorSubSetMulti = 0x01;

/**
 * @brief RRC checksum: ~ (func + len + Σ data) & 0xFF.
 */
std::uint8_t ChecksumCrc8(std::uint8_t function, std::uint8_t length,
                          const std::uint8_t* data);

/**
 * @brief Build PACKET_FUNC_MOTOR multi-set frame for up to 4 motors.
 * @param[in] motor_ids Motor indices (1–4); length @p count.
 * @param[in] rps Target wheel speeds in revolutions per second.
 * @param[in] count Number of motors (1–4).
 * @return Complete wire bytes including header and checksum.
 */
std::vector<std::uint8_t> BuildSetMotorsPacket(const std::uint8_t* motor_ids,
                                               const float* rps,
                                               std::size_t count);

/**
 * @brief Convenience: four motors with ids 1..4.
 */
inline std::vector<std::uint8_t> BuildSetFourMotorsPacket(
    const float rps[4]) {
  const std::uint8_t ids[4] = {1, 2, 3, 4};
  return BuildSetMotorsPacket(ids, rps, 4);
}

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_JETAUTO_RRC_PROTOCOL_HPP_
