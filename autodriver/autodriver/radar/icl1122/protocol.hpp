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
 * @file protocol.hpp
 * @brief EVB1122 / ICL1122 UART demo target-list + MCU ICLH frame parsers.
 *
 * Demo object list (UM10035 §6.1.5 / indoor multi-human firmware):
 *   AA FF <N> 00 | N × {x,y,vel,range_res} (LE uint16) | 55 CC
 * Sign-magnitude int16: bit15=1 → +, bit15=0 → −; magnitude in bits[14:0].
 * Units: x/y mm, velocity cm/s, range_res mm.
 *
 * USBHS / JoySpace MCU wrap (UM10034P §5.3 + 24GMmwave SerialStreamProcessor):
 *   ICLH (49 43 4C 48) | len_u16 LE | type_u8 | channel_u8 | payload[len] | ICLT
 *   `len` is payload size only (type/channel not included).
 *
 * Assembled RangeFFT bin (host tool output, JoySpace doc):
 *   0x5A5A5A5A | frameIndex | rangeFFT_number | complex bins… | 0xA5A5A5A5
 */

#ifndef AUTODRIVER_RADAR_ICL1122_PROTOCOL_HPP_
#define AUTODRIVER_RADAR_ICL1122_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "autolink/common/macros.hpp"

namespace autodriver {
namespace protocol {

/** @brief One tracked target from the EVB1122 demo UART frame. */
struct Icl1122Target {
  // Lateral position in meters (device X; + = bit15 set in raw).
  double x_m{0.0};
  // Range along board normal in meters (device Y).
  double y_m{0.0};
  // Radial velocity in m/s (+ leaving board, − approaching).
  double velocity_mps{0.0};
  // Reported range-bin size in meters (same for all targets in a frame).
  double range_resolution_m{0.0};
  // True when this slot carries a non-zero detection.
  bool valid{false};
};

/** @brief Parsed demo target-list frame. */
struct Icl1122TargetFrame {
  // Decoded targets (size == N from header, empty slots have valid=false).
  std::vector<Icl1122Target> targets;
};

/** @brief Parsed MCU wrap around DSP RAW / FFT payload (passthrough firmware). */
struct Icl1122McuFrame {
  // Frame type byte (datasheet Table 5-4; demo uses 0x00).
  std::uint8_t type{0};
  // RX / logical channel index from the MCU header.
  std::uint8_t channel{0};
  // Inner DSP dword stream (DS RAW / Range FFT / Doppler FFT).
  std::vector<std::uint8_t> payload;
};

/**
 * @class autodriver::protocol::Icl1122TargetParser
 * @brief Incremental parser for AA FF N 00 … 55 CC target frames.
 */
class Icl1122TargetParser {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(Icl1122TargetParser)
  DISALLOW_COPY_AND_ASSIGN(Icl1122TargetParser)

  Icl1122TargetParser() = default;

  /**
   * @brief Feed one UART byte; returns a frame when a complete list is decoded.
   * @param[in] byte Next byte from the serial stream.
   * @return Frame when trailer matched; otherwise nullopt.
   */
  std::optional<Icl1122TargetFrame> Feed(std::uint8_t byte);

  /** @brief Drop partial sync state. */
  void Reset();

  /**
   * @brief Decode sign-magnitude LE uint16 to signed physical value.
   * @param[in] lo Low byte.
   * @param[in] hi High byte (bit7 = sign flag for magnitude in bits[14:0]).
   * @param[in] scale Multiplier (e.g. 0.001 for mm→m, 0.01 for cm/s→m/s).
   */
  static double DecodeSignMagnitude(std::uint8_t lo, std::uint8_t hi,
                                    double scale);

  /** @brief Max targets accepted from header N (outdoor demo ≤ 20). */
  static constexpr std::size_t kMaxTargets = 20;

 private:
  enum class State : std::uint8_t {
    kSync0,
    kSync1,
    kCount,
    kReserved,
    kPayload,
    kTail0,
    kTail1,
  };

  State state_{State::kSync0};
  std::size_t target_count_{0};
  std::size_t payload_need_{0};
  std::vector<std::uint8_t> payload_;
};

/**
 * @class autodriver::protocol::Icl1122McuParser
 * @brief Incremental parser for ICLH … ICLT MCU encapsulation frames.
 */
class Icl1122McuParser {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(Icl1122McuParser)
  DISALLOW_COPY_AND_ASSIGN(Icl1122McuParser)

  Icl1122McuParser() = default;

  /**
   * @brief Feed one byte; returns a frame when ICLT trailer is complete.
   * @param[in] byte Next UART / USB CDC byte.
   * @return Frame when length and trailer match; otherwise nullopt.
   */
  std::optional<Icl1122McuFrame> Feed(std::uint8_t byte);

  /** @brief Drop partial sync state. */
  void Reset();

  /** @brief Soft upper bound on inner payload (DSP chirp dumps). */
  static constexpr std::size_t kMaxPayload = 1u << 20;

 private:
  enum class State : std::uint8_t {
    kH0,
    kH1,
    kH2,
    kH3,
    kLen0,
    kLen1,
    kType,
    kChannel,
    kPayload,
    kT0,
    kT1,
    kT2,
    kT3,
  };

  State state_{State::kH0};
  std::uint16_t length_{0};
  std::uint8_t type_{0};
  std::uint8_t channel_{0};
  std::size_t payload_got_{0};
  std::vector<std::uint8_t> payload_;
};

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_ICL1122_PROTOCOL_HPP_
