/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_payload.hpp
 * @brief Decodes Autolink framed channel payloads (MessageHeader + content).
 *
 * Autolink wire format may prepend a @c MessageHeader with magic @c BDACBDAC.
 * Displays and TF ingest call @ref DecodeChannelPayload before protobuf parse.
 *
 * @see ChannelReaderRegistry
 * @see transform::Listener::applyPayload
 */

#pragma once

#include <cstring>
#include <string>

#include "autolink/message/message_header.hpp"

namespace autoviz {
namespace integration {

/**
 * @brief Strips an Autolink @c MessageHeader when present; otherwise returns @p raw.
 *
 * If @p raw is too short, magic mismatches, or content size is inconsistent,
 * the original @p raw string is returned unchanged (passthrough for already-
 * decoded or non-framed payloads).
 *
 * @param raw Bytes received from an Autolink reader callback.
 * @return Protobuf (or other) content bytes without the framing header.
 */
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
