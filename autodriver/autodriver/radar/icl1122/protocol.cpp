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
 * @file protocol.cpp
 * @brief EVB1122 / ICL1122 UART frame parsers (implementation).
 */

#include "autodriver/radar/icl1122/protocol.hpp"

namespace autodriver {
namespace protocol {
namespace {

constexpr std::uint8_t kDemoHead0 = 0xAA;
constexpr std::uint8_t kDemoHead1 = 0xFF;
constexpr std::uint8_t kDemoTail0 = 0x55;
constexpr std::uint8_t kDemoTail1 = 0xCC;
constexpr std::size_t kBytesPerTarget = 8;

constexpr std::uint8_t kMcuH0 = 0x49;  // 'I'
constexpr std::uint8_t kMcuH1 = 0x43;  // 'C'
constexpr std::uint8_t kMcuH2 = 0x4C;  // 'L'
constexpr std::uint8_t kMcuH3 = 0x48;  // 'H'
constexpr std::uint8_t kMcuT0 = 0x49;  // 'I'
constexpr std::uint8_t kMcuT1 = 0x43;  // 'C'
constexpr std::uint8_t kMcuT2 = 0x4C;  // 'L'
constexpr std::uint8_t kMcuT3 = 0x54;  // 'T'

bool SlotNonZero(const std::uint8_t* p) {
  for (std::size_t i = 0; i < kBytesPerTarget; ++i) {
    if (p[i] != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

double Icl1122TargetParser::DecodeSignMagnitude(std::uint8_t lo, std::uint8_t hi,
                                                double scale) {
  const std::uint16_t raw =
      static_cast<std::uint16_t>(lo) |
      (static_cast<std::uint16_t>(hi) << 8);
  const std::uint16_t mag = static_cast<std::uint16_t>(raw & 0x7FFFu);
  const bool positive = (raw & 0x8000u) != 0;
  const double signed_mag =
      (positive ? 1.0 : -1.0) * static_cast<double>(mag);
  return signed_mag * scale;
}

void Icl1122TargetParser::Reset() {
  state_ = State::kSync0;
  target_count_ = 0;
  payload_need_ = 0;
  payload_.clear();
}

std::optional<Icl1122TargetFrame> Icl1122TargetParser::Feed(std::uint8_t byte) {
  switch (state_) {
    case State::kSync0:
      if (byte == kDemoHead0) {
        state_ = State::kSync1;
      }
      break;
    case State::kSync1:
      if (byte == kDemoHead1) {
        state_ = State::kCount;
      } else if (byte == kDemoHead0) {
        state_ = State::kSync1;
      } else {
        state_ = State::kSync0;
      }
      break;
    case State::kCount:
      target_count_ = byte;
      if (target_count_ == 0 || target_count_ > kMaxTargets) {
        Reset();
        break;
      }
      state_ = State::kReserved;
      break;
    case State::kReserved:
      // Manual: AA FF N 00
      if (byte != 0x00) {
        Reset();
        if (byte == kDemoHead0) {
          state_ = State::kSync1;
        }
        break;
      }
      payload_need_ = target_count_ * kBytesPerTarget;
      payload_.clear();
      payload_.reserve(payload_need_);
      state_ = State::kPayload;
      break;
    case State::kPayload:
      payload_.push_back(byte);
      if (payload_.size() >= payload_need_) {
        state_ = State::kTail0;
      }
      break;
    case State::kTail0:
      if (byte != kDemoTail0) {
        Reset();
        if (byte == kDemoHead0) {
          state_ = State::kSync1;
        }
        break;
      }
      state_ = State::kTail1;
      break;
    case State::kTail1:
      if (byte != kDemoTail1) {
        Reset();
        if (byte == kDemoHead0) {
          state_ = State::kSync1;
        }
        break;
      }
      {
        Icl1122TargetFrame frame;
        frame.targets.resize(target_count_);
        for (std::size_t i = 0; i < target_count_; ++i) {
          const std::uint8_t* p = payload_.data() + i * kBytesPerTarget;
          Icl1122Target& t = frame.targets[i];
          t.x_m = DecodeSignMagnitude(p[0], p[1], 0.001);
          t.y_m = DecodeSignMagnitude(p[2], p[3], 0.001);
          t.velocity_mps = DecodeSignMagnitude(p[4], p[5], 0.01);
          const std::uint16_t rr =
              static_cast<std::uint16_t>(p[6]) |
              (static_cast<std::uint16_t>(p[7]) << 8);
          t.range_resolution_m = static_cast<double>(rr) * 0.001;
          t.valid = SlotNonZero(p);
        }
        Reset();
        return frame;
      }
  }
  return std::nullopt;
}

void Icl1122McuParser::Reset() {
  state_ = State::kH0;
  length_ = 0;
  type_ = 0;
  channel_ = 0;
  payload_got_ = 0;
  payload_.clear();
}

std::optional<Icl1122McuFrame> Icl1122McuParser::Feed(std::uint8_t byte) {
  switch (state_) {
    case State::kH0:
      state_ = (byte == kMcuH0) ? State::kH1 : State::kH0;
      break;
    case State::kH1:
      state_ = (byte == kMcuH1) ? State::kH2
                                : (byte == kMcuH0 ? State::kH1 : State::kH0);
      break;
    case State::kH2:
      state_ = (byte == kMcuH2) ? State::kH3
                                : (byte == kMcuH0 ? State::kH1 : State::kH0);
      break;
    case State::kH3:
      state_ = (byte == kMcuH3) ? State::kLen0
                                : (byte == kMcuH0 ? State::kH1 : State::kH0);
      break;
    case State::kLen0:
      length_ = byte;
      state_ = State::kLen1;
      break;
    case State::kLen1:
      length_ = static_cast<std::uint16_t>(
          length_ | (static_cast<std::uint16_t>(byte) << 8));
      // JoySpace / 24GMmwave SerialStreamProcessor: length is payload bytes
      // only; type + channel follow as separate fields (not counted in length).
      if (length_ > kMaxPayload) {
        Reset();
        break;
      }
      state_ = State::kType;
      break;
    case State::kType:
      type_ = byte;
      state_ = State::kChannel;
      break;
    case State::kChannel:
      channel_ = byte;
      payload_got_ = 0;
      payload_.assign(static_cast<std::size_t>(length_), 0);
      if (payload_.empty()) {
        state_ = State::kT0;
      } else {
        state_ = State::kPayload;
      }
      break;
    case State::kPayload:
      payload_[payload_got_++] = byte;
      if (payload_got_ >= payload_.size()) {
        state_ = State::kT0;
      }
      break;
    case State::kT0:
      if (byte != kMcuT0) {
        Reset();
        if (byte == kMcuH0) {
          state_ = State::kH1;
        }
        break;
      }
      state_ = State::kT1;
      break;
    case State::kT1:
      if (byte != kMcuT1) {
        Reset();
        break;
      }
      state_ = State::kT2;
      break;
    case State::kT2:
      if (byte != kMcuT2) {
        Reset();
        break;
      }
      state_ = State::kT3;
      break;
    case State::kT3:
      if (byte != kMcuT3) {
        Reset();
        break;
      }
      {
        Icl1122McuFrame frame;
        frame.type = type_;
        frame.channel = channel_;
        frame.payload = std::move(payload_);
        Reset();
        return frame;
      }
  }
  return std::nullopt;
}

}  // namespace protocol
}  // namespace autodriver
