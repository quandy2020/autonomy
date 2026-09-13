/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView bag format (JSONL):
 *   line 0: {"op":"orbisview_bag_header","version":1,...}
 *   line N: StreamEnvelope JSON
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "autonomy/orbisview/backend/common/stream_envelope.h"

namespace autonomy {
namespace orbisview {
namespace backend {

struct RecordOptions {
  std::string path;
  /** Empty = record all channels. */
  std::unordered_set<std::string> channels;
};

struct PlaybackOptions {
  std::string path;
  double speed{1.0};
  bool loop{false};
  /** Start at this envelope index (0 = first data line after header). */
  uint64_t start_index{0};
  /** Optional start timestamp filter (ns); 0 = ignore. */
  int64_t start_timestamp_ns{0};
};

struct BagIndexEntry {
  uint64_t index{0};
  int64_t timestamp_ns{0};
  std::streampos file_pos{0};
  std::string channel;
};

class EnvelopeRecorder {
 public:
  using EmitFn = std::function<void(core::StreamEnvelope)>;

  bool StartRecording(const RecordOptions& options);
  void StopRecording();
  bool Recording() const { return recording_.load(); }
  void SetChannelFilter(std::unordered_set<std::string> channels);

  void Append(const core::StreamEnvelope& env);

  bool StartPlayback(const PlaybackOptions& options, EmitFn emit);
  void StopPlayback();
  void PausePlayback(bool paused);
  bool Playing() const { return playing_.load(); }
  bool Paused() const { return paused_.load(); }

  /** Seek to envelope index during / before playback (rebuilds index if needed). */
  bool SeekIndex(uint64_t index);
  bool SeekTimestamp(int64_t timestamp_ns);

  /** Build or return cached index for a bag path. */
  bool EnsureIndex(const std::string& path);
  std::string IndexJson(const std::string& path) const;

  std::string StatusJson() const;

 private:
  void PlaybackLoop(PlaybackOptions options, EmitFn emit);
  bool OpenBagForAppendLocked(const std::string& path);
  static bool ParseEnvelopeLine(const std::string& line, core::StreamEnvelope* out);
  static bool IsHeaderLine(const std::string& line);

  mutable std::mutex mutex_;
  std::ofstream out_;
  std::atomic<bool> recording_{false};
  std::atomic<bool> playing_{false};
  std::atomic<bool> paused_{false};
  std::atomic<bool> seek_requested_{false};
  std::atomic<uint64_t> seek_index_{0};
  std::condition_variable pause_cv_;
  std::thread play_thread_;

  std::string record_path_;
  std::string play_path_;
  std::unordered_set<std::string> channel_filter_;
  uint64_t recorded_{0};
  uint64_t play_index_{0};
  uint64_t play_total_{0};
  double play_speed_{1.0};
  bool play_loop_{false};

  mutable std::mutex index_mutex_;
  std::string indexed_path_;
  std::vector<BagIndexEntry> index_;
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
