/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ws_frame.hpp
 * @brief Binary framing shared by the Autoviz WebSocket server.
 *
 * Wire layout (little endian):
 * @code
 *   opcode(1) + channel_id(u32) + timestamp_ns(u64) + payload
 * @endcode
 *
 * Kept header-only so both @ref WsServer and the unit tests use one encoder /
 * decoder instead of duplicating the byte layout.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace autoviz {
namespace bridge {

/** Binary frame opcode for channel payloads. */
inline constexpr std::uint8_t kMessageOpcode = 1;

/** Header bytes before the payload: opcode + channel id + timestamp. */
inline constexpr std::size_t kMessageHeaderSize = 1 + 4 + 8;

/** Writes @p value as a little-endian 32-bit integer. */
inline void WriteUint32LE(std::uint8_t* buf, std::uint32_t value) {
  buf[0] = static_cast<std::uint8_t>(value & 0xff);
  buf[1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  buf[2] = static_cast<std::uint8_t>((value >> 16) & 0xff);
  buf[3] = static_cast<std::uint8_t>((value >> 24) & 0xff);
}

/** Writes @p value as a little-endian 64-bit integer. */
inline void WriteUint64LE(std::uint8_t* buf, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    buf[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xff);
  }
}

/** Reads a little-endian 32-bit integer. */
inline std::uint32_t ReadUint32LE(const std::uint8_t* buf) {
  return static_cast<std::uint32_t>(buf[0]) |
         (static_cast<std::uint32_t>(buf[1]) << 8) |
         (static_cast<std::uint32_t>(buf[2]) << 16) |
         (static_cast<std::uint32_t>(buf[3]) << 24);
}

/** Reads a little-endian 64-bit integer. */
inline std::uint64_t ReadUint64LE(const std::uint8_t* buf) {
  std::uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | buf[i];
  }
  return value;
}

/**
 * @brief Encodes one binary message frame.
 *
 * @param channel_id Numeric channel id negotiated over the text protocol.
 * @param timestamp_ns Message stamp in nanoseconds.
 * @param payload Serialized message bytes.
 * @return Frame bytes ready for @c mg_websocket_write.
 */
inline std::string EncodeMessageFrame(std::uint32_t channel_id,
                                      std::uint64_t timestamp_ns,
                                      std::string_view payload) {
  std::string frame(kMessageHeaderSize, '\0');
  auto* bytes = reinterpret_cast<std::uint8_t*>(frame.data());
  bytes[0] = kMessageOpcode;
  WriteUint32LE(bytes + 1, channel_id);
  WriteUint64LE(bytes + 5, timestamp_ns);
  frame.append(payload.data(), payload.size());
  return frame;
}

/** Decoded view of one binary message frame. */
struct DecodedMessageFrame {
  std::uint8_t opcode = 0;          /**< Frame opcode (1 = channel payload). */
  std::uint32_t channel_id = 0;     /**< Numeric channel id. */
  std::uint64_t timestamp_ns = 0;   /**< Message stamp in nanoseconds. */
  std::string_view payload;         /**< Payload bytes (view into the frame). */
};

/**
 * @brief Decodes one binary message frame.
 *
 * @param frame Bytes received from the WebSocket.
 * @param[out] out Decoded fields (unchanged on failure).
 * @return @c false when @p frame is shorter than @ref kMessageHeaderSize.
 */
inline bool DecodeMessageFrame(std::string_view frame,
                               DecodedMessageFrame* out) {
  if (out == nullptr || frame.size() < kMessageHeaderSize) {
    return false;
  }
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(frame.data());
  out->opcode = bytes[0];
  out->channel_id = ReadUint32LE(bytes + 1);
  out->timestamp_ns = ReadUint64LE(bytes + 5);
  out->payload = frame.substr(kMessageHeaderSize);
  return true;
}

}  // namespace bridge
}  // namespace autoviz

