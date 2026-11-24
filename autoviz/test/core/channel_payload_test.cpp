/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_payload_test.cpp
 * @brief Framing regression for ChannelPayloadView / DecodeChannelPayload.
 *
 * Autolink readers deliver either raw protobuf bytes or a MessageHeader-framed
 * payload. Displays parse through the zero-copy view, so both shapes and the
 * malformed cases are covered here.
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "autolink/message/message_header.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

std::string Frame(const std::string& body, bool valid_magic = true) {
  autolink::message::MessageHeader header;
  if (!valid_magic) {
    header.reset_magic_num();
    // Corrupt the first byte so is_magic_num_match() fails.
    std::string bytes(sizeof(header), '\0');
    std::memcpy(bytes.data(), &header, sizeof(header));
    bytes[0] = 'X';
    return bytes + body;
  }
  header.set_content_size(static_cast<std::uint32_t>(body.size()));
  std::string bytes(sizeof(header), '\0');
  std::memcpy(bytes.data(), &header, sizeof(header));
  return bytes + body;
}

void TestUnframed() {
  const std::string raw = "plain-protobuf-bytes";
  const std::string_view view = autoviz::integration::ChannelPayloadView(raw);
  Check(view == std::string_view(raw), "unframed payload is returned as-is");
  Check(autoviz::integration::DecodeChannelPayload(raw) == raw,
        "unframed payload decode matches");
}

void TestFramed() {
  const std::string body = "framed-body";
  const std::string raw = Frame(body) + "trailing-garbage";
  const std::string_view view = autoviz::integration::ChannelPayloadView(raw);
  Check(view == std::string_view(body),
        "framed payload view exposes only the content bytes");
  Check(autoviz::integration::DecodeChannelPayload(raw) == body,
        "framed payload decode matches the content bytes");
}

void TestTruncated() {
  const std::string body = "short";
  std::string raw = Frame(body);
  // Claim more content than is present.
  autolink::message::MessageHeader header;
  header.set_content_size(static_cast<std::uint32_t>(body.size()) + 64);
  std::memcpy(raw.data(), &header, sizeof(header));
  Check(autoviz::integration::ChannelPayloadView(raw) == std::string_view(raw),
        "truncated frame falls back to the raw bytes");
}

void TestBadMagic() {
  const std::string body = "body";
  const std::string raw = Frame(body, /*valid_magic=*/false);
  Check(autoviz::integration::ChannelPayloadView(raw) == std::string_view(raw),
        "foreign header falls back to the raw bytes");
}

void TestShortHeader() {
  const std::string raw = "abc";
  Check(autoviz::integration::ChannelPayloadView(raw) == std::string_view(raw),
        "payload shorter than a header is returned as-is");
}

}  // namespace

int main() {
  TestUnframed();
  TestFramed();
  TestTruncated();
  TestBadMagic();
  TestShortHeader();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d channel payload test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("channel payload tests passed\n");
  return 0;
}

