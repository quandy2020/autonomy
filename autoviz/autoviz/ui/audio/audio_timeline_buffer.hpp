/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

namespace autoviz {
namespace audio_panel {

struct AudioEnvelopeColumn {
  float min_amplitude = 0.0f;
  float max_amplitude = 0.0f;
};

/** Time-indexed PCM buffer for waveform rendering and scrubbing. */
class AudioTimelineBuffer {
 public:
  void clear();

  void append(std::int64_t timestamp_ns, int sample_rate, int channels,
              std::vector<std::int16_t> pcm_samples);

  void trimToWindow(std::int64_t window_ns);

  std::int64_t startTimestampNs() const;
  std::int64_t endTimestampNs() const;
  int sampleRate() const;
  int channelCount() const;
  bool empty() const;

  void buildEnvelope(std::int64_t view_start_ns, std::int64_t view_end_ns,
                     int column_count, std::vector<AudioEnvelopeColumn>* columns) const;

  bool readInterleavedFrames(std::int64_t start_ns, std::int64_t end_ns,
                             std::vector<std::int16_t>* frames) const;

 private:
  struct Chunk {
    std::int64_t timestamp_ns = 0;
    int sample_rate = 0;
    int channels = 0;
    std::vector<std::int16_t> pcm_samples;
  };

  mutable std::mutex mutex_;
  std::vector<Chunk> chunks_;
  int sample_rate_ = 0;
  int channels_ = 0;
};

}  // namespace audio_panel
}  // namespace autoviz
