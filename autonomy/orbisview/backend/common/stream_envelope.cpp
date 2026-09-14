/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/stream_envelope.hpp"

#include <cstdio>
#include <sstream>

namespace autonomy {
namespace orbisview {
namespace core {
namespace {

constexpr char kB64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

bool LooksLikeUtf8Json(const std::vector<uint8_t>& data) {
  if (data.empty()) {
    return false;
  }
  const char c = static_cast<char>(data[0]);
  return c == '{' || c == '[';
}

std::string Base64EncodeImpl(const std::vector<uint8_t>& data) {
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);
  size_t i = 0;
  while (i + 2 < data.size()) {
    const uint32_t n =
        (static_cast<uint32_t>(data[i]) << 16) |
        (static_cast<uint32_t>(data[i + 1]) << 8) |
        static_cast<uint32_t>(data[i + 2]);
    out.push_back(kB64[(n >> 18) & 63]);
    out.push_back(kB64[(n >> 12) & 63]);
    out.push_back(kB64[(n >> 6) & 63]);
    out.push_back(kB64[n & 63]);
    i += 3;
  }
  if (i < data.size()) {
    uint32_t n = static_cast<uint32_t>(data[i]) << 16;
    out.push_back(kB64[(n >> 18) & 63]);
    if (i + 1 < data.size()) {
      n |= static_cast<uint32_t>(data[i + 1]) << 8;
      out.push_back(kB64[(n >> 12) & 63]);
      out.push_back(kB64[(n >> 6) & 63]);
      out.push_back('=');
    } else {
      out.push_back(kB64[(n >> 12) & 63]);
      out.push_back('=');
      out.push_back('=');
    }
  }
  return out;
}

}  // namespace

std::string Base64Encode(const std::vector<uint8_t>& data) {
  return Base64EncodeImpl(data);
}

std::string JsonEscape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  out.push_back('"');
  for (unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out.push_back('"');
  return out;
}

std::string StreamEnvelopeToJson(const StreamEnvelope& env) {
  std::ostringstream oss;
  oss << "{\"op\":\"envelope\""
      << ",\"channel\":" << JsonEscape(env.channel)
      << ",\"schema\":" << JsonEscape(env.schema)
      << ",\"timestamp\":" << env.timestamp_ns
      << ",\"frame_id\":" << JsonEscape(env.frame_id)
      << ",\"sequence\":" << env.sequence
      << ",\"encoding\":" << JsonEscape(env.encoding)
      << ",\"unsupported\":" << (env.unsupported ? "true" : "false")
      << ",\"stale\":" << (env.stale ? "true" : "false");
  if (env.encoding == "json" && LooksLikeUtf8Json(env.payload)) {
    oss << ",\"payload\":";
    oss.write(reinterpret_cast<const char*>(env.payload.data()),
              static_cast<std::streamsize>(env.payload.size()));
  } else {
    oss << ",\"payload_b64\":" << JsonEscape(Base64Encode(env.payload));
  }
  oss << '}';
  return oss.str();
}

std::string ChannelListToJson(const std::vector<ChannelInfo>& channels) {
  std::ostringstream oss;
  oss << "{\"op\":\"channels\",\"channels\":[";
  for (size_t i = 0; i < channels.size(); ++i) {
    const auto& c = channels[i];
    if (i > 0) {
      oss << ',';
    }
    oss << "{\"name\":" << JsonEscape(c.name)
        << ",\"schema\":" << JsonEscape(c.schema)
        << ",\"msg_type\":" << JsonEscape(c.msg_type)
        << ",\"has_writer\":" << (c.has_writer ? "true" : "false")
        << ",\"mock\":" << (c.mock ? "true" : "false") << '}';
  }
  oss << "]}";
  return oss.str();
}

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
