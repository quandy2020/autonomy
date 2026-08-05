/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autoviz {
namespace audio_panel {

struct ParsedRawAudio {
  std::int64_t timestamp_ns = 0;
  std::vector<std::int16_t> pcm_samples;
  int sample_rate = 0;
  int number_of_channels = 0;
  std::string format;
};

bool IsRawAudioMessageType(const std::string& message_type);

/** Parses foxglove.RawAudio JSON payloads (pcm-s16 little-endian). */
bool ParseRawAudioPayload(const std::string& message_type,
                          const std::string& payload, ParsedRawAudio* audio,
                          std::string* error_message = nullptr);

}  // namespace audio_panel
}  // namespace autoviz
