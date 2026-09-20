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
 * @file rrc_protocol.cpp
 * @brief Hiwonder RRC host serial framing (implementation).
 */

#include "chassis/jetauto/rrc_protocol.hpp"

#include <cstring>

namespace autodriver {
namespace chassis {
namespace jetauto {
namespace {

void AppendFloatLe(std::vector<std::uint8_t>* out, float value) {
  static_assert(sizeof(float) == 4, "IEEE-754 float required");
  std::uint8_t bytes[4];
  std::memcpy(bytes, &value, sizeof(bytes));
  out->insert(out->end(), bytes, bytes + 4);
}

}  // namespace

std::uint8_t ChecksumCrc8(std::uint8_t function, std::uint8_t length,
                          const std::uint8_t* data) {
  unsigned sum = static_cast<unsigned>(function) + static_cast<unsigned>(length);
  if (data != nullptr) {
    for (std::uint8_t i = 0; i < length; ++i) {
      sum += data[i];
    }
  }
  return static_cast<std::uint8_t>(~sum);
}

std::vector<std::uint8_t> BuildSetMotorsPacket(const std::uint8_t* motor_ids,
                                               const float* rps,
                                               std::size_t count) {
  std::vector<std::uint8_t> packet;
  if (motor_ids == nullptr || rps == nullptr || count == 0 || count > 4) {
    return packet;
  }

  // Payload length = 5N + 2 (subcommand + quantity + N*(id + float)).
  const std::uint8_t length =
      static_cast<std::uint8_t>(5 * count + 2);
  std::vector<std::uint8_t> payload;
  payload.reserve(length);
  payload.push_back(kMotorSubSetMulti);
  payload.push_back(static_cast<std::uint8_t>(count));
  for (std::size_t i = 0; i < count; ++i) {
    payload.push_back(motor_ids[i]);
    AppendFloatLe(&payload, rps[i]);
  }

  packet.reserve(static_cast<std::size_t>(length) + 5);
  packet.push_back(kStartByte1);
  packet.push_back(kStartByte2);
  packet.push_back(kPacketFuncMotor);
  packet.push_back(length);
  packet.insert(packet.end(), payload.begin(), payload.end());
  packet.push_back(ChecksumCrc8(kPacketFuncMotor, length, payload.data()));
  return packet;
}

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver
