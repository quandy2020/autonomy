/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstring>
#include <string>

#include "autolink/message/message_header.hpp"

namespace autoviz {
namespace integration {

inline std::string DecodeChannelPayload(const std::string& raw) {
  using autolink::message::MessageHeader;
  if (raw.size() < sizeof(MessageHeader)) {
    return raw;
  }
  MessageHeader header;
  std::memcpy(&header, raw.data(), sizeof(MessageHeader));
  if (!header.is_magic_num_match("BDACBDAC", 8)) {
    return raw;
  }
  const std::size_t header_size = sizeof(MessageHeader);
  const std::size_t content_size = header.content_size();
  if (raw.size() < header_size + content_size) {
    return raw;
  }
  return raw.substr(header_size, content_size);
}

}  // namespace integration
}  // namespace autoviz
