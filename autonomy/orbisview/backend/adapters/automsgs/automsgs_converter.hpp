/*
 * Copyright 2026 The Openbot Authors
 *
 * Convert selected automsgs protobuf payloads into StreamEnvelope with
 * orbisview.render.* JSON schemas. Unknown types fall back to unsupported.
 */

#pragma once

#include <cstdint>
#include <string>

#include "autonomy/orbisview/backend/common/stream_envelope.hpp"

namespace autonomy {
namespace orbisview {
namespace adapters {

/** Suggested render schema for a topology msg_type (empty if unknown). */
std::string SuggestedRenderSchema(const std::string& msg_type);

/**
 * Parse raw protobuf bytes into a render StreamEnvelope.
 * Returns false on parse failure or unsupported type (caller may emit raw).
 */
bool ConvertAutomsgsRaw(const std::string& channel, const std::string& msg_type,
                        const std::string& bytes, int64_t timestamp_ns,
                        core::StreamEnvelope* out);

}  // namespace adapters
}  // namespace orbisview
}  // namespace autonomy
