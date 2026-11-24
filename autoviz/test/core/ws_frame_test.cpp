/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ws_frame_test.cpp
 * @brief Byte-layout regression for the WebSocket binary frame helper.
 */

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

#include "autoviz/bridge/ws_frame.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

void TestHeaderSize() {
  Check(autoviz::bridge::kMessageHeaderSize == 13,
        "header is opcode + u32 + u64");
}

void TestRoundTrip() {
  using autoviz::bridge::DecodeMessageFrame;
  using autoviz::bridge::DecodedMessageFrame;
  using autoviz::bridge::EncodeMessageFrame;

  const std::uint32_t channel = 0x01020304u;
  const std::uint64_t stamp = 0x1122334455667788ull;
  const std::string frame = EncodeMessageFrame(channel, stamp, "abc");

  Check(frame.size() == autoviz::bridge::kMessageHeaderSize + 3,
        "frame length is header + payload");
  Check(static_cast<std::uint8_t>(frame[0]) ==
            autoviz::bridge::kMessageOpcode,
        "opcode is the message opcode");

  const auto* bytes = reinterpret_cast<const std::uint8_t*>(frame.data());
  Check(autoviz::bridge::ReadUint32LE(bytes + 1) == channel,
        "channel id is little-endian");
  Check(autoviz::bridge::ReadUint64LE(bytes + 5) == stamp,
        "timestamp is little-endian");

  DecodedMessageFrame decoded;
  Check(DecodeMessageFrame(frame, &decoded), "frame decodes");
  Check(decoded.opcode == autoviz::bridge::kMessageOpcode,
        "decoded opcode matches");
  Check(decoded.channel_id == channel, "decoded channel matches");
  Check(decoded.timestamp_ns == stamp, "decoded timestamp matches");
  Check(decoded.payload == std::string_view("abc"),
        "decoded payload matches");
}

void TestEmptyPayload() {
  using autoviz::bridge::DecodeMessageFrame;
  using autoviz::bridge::DecodedMessageFrame;
  using autoviz::bridge::EncodeMessageFrame;

  const std::string frame = EncodeMessageFrame(7, 42, std::string_view());
  Check(frame.size() == autoviz::bridge::kMessageHeaderSize,
        "empty payload keeps only the header");
  DecodedMessageFrame decoded;
  Check(DecodeMessageFrame(frame, &decoded), "header-only frame decodes");
  Check(decoded.payload.empty(), "header-only frame has empty payload");
}

void TestShortFrame() {
  using autoviz::bridge::DecodeMessageFrame;
  using autoviz::bridge::DecodedMessageFrame;

  DecodedMessageFrame decoded;
  Check(!DecodeMessageFrame(std::string_view("123456789012"), &decoded),
        "frame shorter than the header is rejected");
  Check(!DecodeMessageFrame(std::string_view(), &decoded),
        "empty frame is rejected");
}

void TestOpcodeIsReportedNotFiltered() {
  using autoviz::bridge::DecodeMessageFrame;
  using autoviz::bridge::DecodedMessageFrame;

  std::string frame = autoviz::bridge::EncodeMessageFrame(1, 2, "x");
  frame[0] = 2;
  DecodedMessageFrame decoded;
  Check(DecodeMessageFrame(frame, &decoded), "unknown opcode still decodes");
  Check(decoded.opcode == 2, "decoder reports the raw opcode");
}

}  // namespace

int main() {
  TestHeaderSize();
  TestRoundTrip();
  TestEmptyPayload();
  TestShortFrame();
  TestOpcodeIsReportedNotFiltered();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d ws frame test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("ws frame tests passed\n");
  return 0;
}

