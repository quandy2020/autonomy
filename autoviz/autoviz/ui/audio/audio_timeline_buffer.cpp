/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_timeline_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace audio_panel {
namespace {

std::int64_t ChunkEndTimestampNs(std::int64_t timestamp_ns, int sample_rate,
                                 int channels, std::size_t pcm_sample_count) {
  if (sample_rate <= 0 || channels <= 0 || pcm_sample_count == 0) {
    return timestamp_ns;
  }
  const std::int64_t frame_count =
      static_cast<std::int64_t>(pcm_sample_count) / channels;
  const double duration_sec =
      static_cast<double>(frame_count) / static_cast<double>(sample_rate);
  return timestamp_ns + static_cast<std::int64_t>(duration_sec * 1e9);
}

float SampleToAmplitude(std::int16_t sample) {
  return static_cast<float>(sample) / 32768.0f;
}

}  // namespace

void AudioTimelineBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  chunks_.clear();
  sample_rate_ = 0;
  channels_ = 0;
}

void AudioTimelineBuffer::append(std::int64_t timestamp_ns, int sample_rate,
                                 int channels,
                                 std::vector<std::int16_t> pcm_samples) {
  if (sample_rate <= 0 || channels <= 0 || pcm_samples.empty()) {
    return;
  }

  Chunk chunk;
  chunk.timestamp_ns = timestamp_ns;
  chunk.sample_rate = sample_rate;
  chunk.channels = channels;
  chunk.pcm_samples = std::move(pcm_samples);

  std::lock_guard<std::mutex> lock(mutex_);
  if (sample_rate_ == 0) {
    sample_rate_ = sample_rate;
    channels_ = channels;
  }
  chunks_.push_back(std::move(chunk));
}

void AudioTimelineBuffer::trimToWindow(std::int64_t window_ns) {
  if (window_ns <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (chunks_.empty()) {
    return;
  }
  const std::int64_t end_ns =
      ChunkEndTimestampNs(chunks_.back().timestamp_ns, chunks_.back().sample_rate,
                          chunks_.back().channels, chunks_.back().pcm_samples.size());
  const std::int64_t min_ns = end_ns - window_ns;
  chunks_.erase(
      std::remove_if(chunks_.begin(), chunks_.end(),
                     [min_ns](const Chunk& chunk) {
                       return ChunkEndTimestampNs(
                                  chunk.timestamp_ns, chunk.sample_rate,
                                  chunk.channels, chunk.pcm_samples.size()) < min_ns;
                     }),
      chunks_.end());
}

std::int64_t AudioTimelineBuffer::startTimestampNs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (chunks_.empty()) {
    return 0;
  }
  return chunks_.front().timestamp_ns;
}

std::int64_t AudioTimelineBuffer::endTimestampNs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (chunks_.empty()) {
    return 0;
  }
  return ChunkEndTimestampNs(chunks_.back().timestamp_ns, chunks_.back().sample_rate,
                           chunks_.back().channels, chunks_.back().pcm_samples.size());
}

int AudioTimelineBuffer::sampleRate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sample_rate_;
}

int AudioTimelineBuffer::channelCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return channels_;
}

bool AudioTimelineBuffer::empty() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return chunks_.empty();
}

void AudioTimelineBuffer::buildEnvelope(
    std::int64_t view_start_ns, std::int64_t view_end_ns, int column_count,
    std::vector<AudioEnvelopeColumn>* columns) const {
  if (columns == nullptr || column_count <= 0 || view_end_ns <= view_start_ns) {
    return;
  }
  columns->assign(static_cast<std::size_t>(column_count), AudioEnvelopeColumn{});

  std::lock_guard<std::mutex> lock(mutex_);
  if (chunks_.empty()) {
    return;
  }

  const double span_ns = static_cast<double>(view_end_ns - view_start_ns);
  for (const Chunk& chunk : chunks_) {
    const std::int64_t chunk_end_ns = ChunkEndTimestampNs(
        chunk.timestamp_ns, chunk.sample_rate, chunk.channels,
        chunk.pcm_samples.size());
    if (chunk_end_ns <= view_start_ns || chunk.timestamp_ns >= view_end_ns) {
      continue;
    }
    const std::int64_t frame_count =
        static_cast<std::int64_t>(chunk.pcm_samples.size()) / chunk.channels;
    if (frame_count <= 0) {
      continue;
    }
    const double frame_duration_ns =
        1e9 / static_cast<double>(chunk.sample_rate);
    for (std::int64_t frame = 0; frame < frame_count; ++frame) {
      const std::int64_t frame_time_ns =
          chunk.timestamp_ns +
          static_cast<std::int64_t>(frame * frame_duration_ns);
      if (frame_time_ns < view_start_ns || frame_time_ns >= view_end_ns) {
        continue;
      }
      const double normalized =
          static_cast<double>(frame_time_ns - view_start_ns) / span_ns;
      const int column = std::clamp(
          static_cast<int>(normalized * column_count), 0, column_count - 1);
      float peak = 0.0f;
      for (int channel = 0; channel < chunk.channels; ++channel) {
        const std::size_t index =
            static_cast<std::size_t>(frame * chunk.channels + channel);
        peak = std::max(peak, std::abs(SampleToAmplitude(chunk.pcm_samples[index])));
      }
      AudioEnvelopeColumn& entry = (*columns)[static_cast<std::size_t>(column)];
      entry.min_amplitude = std::min(entry.min_amplitude, -peak);
      entry.max_amplitude = std::max(entry.max_amplitude, peak);
    }
  }
}

bool AudioTimelineBuffer::readInterleavedFrames(
    std::int64_t start_ns, std::int64_t end_ns,
    std::vector<std::int16_t>* frames) const {
  if (frames == nullptr || end_ns <= start_ns) {
    return false;
  }
  frames->clear();

  std::lock_guard<std::mutex> lock(mutex_);
  if (chunks_.empty() || channels_ <= 0) {
    return false;
  }

  for (const Chunk& chunk : chunks_) {
    const std::int64_t chunk_end_ns = ChunkEndTimestampNs(
        chunk.timestamp_ns, chunk.sample_rate, chunk.channels,
        chunk.pcm_samples.size());
    if (chunk_end_ns <= start_ns || chunk.timestamp_ns >= end_ns) {
      continue;
    }
    const std::int64_t frame_count =
        static_cast<std::int64_t>(chunk.pcm_samples.size()) / chunk.channels;
    const double frame_duration_ns =
        1e9 / static_cast<double>(chunk.sample_rate);
    for (std::int64_t frame = 0; frame < frame_count; ++frame) {
      const std::int64_t frame_time_ns =
          chunk.timestamp_ns +
          static_cast<std::int64_t>(frame * frame_duration_ns);
      if (frame_time_ns < start_ns || frame_time_ns >= end_ns) {
        continue;
      }
      const std::size_t base =
          static_cast<std::size_t>(frame * chunk.channels);
      for (int channel = 0; channel < chunk.channels; ++channel) {
        frames->push_back(chunk.pcm_samples[base + static_cast<std::size_t>(channel)]);
      }
    }
  }
  return !frames->empty();
}

}  // namespace audio_panel
}  // namespace autoviz
