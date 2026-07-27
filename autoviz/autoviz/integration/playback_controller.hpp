/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/tools/autolink_recorder/player/play_param.hpp"
#include "autolink/tools/autolink_recorder/player/player.hpp"

namespace autoviz {
namespace integration {

class PlaybackController {
 public:
  PlaybackController() = default;
  ~PlaybackController();

  void setNode(const std::shared_ptr<::autolink::Node>& node);

  bool openFile(const std::string& path);
  bool play(double rate = 1.0, bool loop = false);
  bool seekTo(double time_s);
  void pause();
  void resume();
  void stop();

  bool isPlaying() const { return playing_; }
  bool isPaused() const { return paused_; }
  bool loop() const { return loop_; }
  const std::string& currentFile() const { return current_file_; }
  double playRate() const { return play_rate_; }
  double totalTimeSec() const { return total_time_sec_; }
  double currentTimeSec() const { return current_time_sec_.load(); }
  double progress() const { return progress_.load(); }
  int channelCount() const { return channel_count_; }
  const std::vector<std::string>& channelNames() const { return channel_names_; }

 private:
  bool startPlayerLocked(double start_time_s);
  bool previewAtLocked(double time_s);
  void stopInfoReader();

  std::shared_ptr<::autolink::Node> node_;
  std::unique_ptr<::autolink::record::Player> player_;
  ::autolink::record::PlayParam play_param_;
  std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>>
      info_reader_;
  std::string current_file_;
  double play_rate_ = 1.0;
  double total_time_sec_ = 0.0;
  uint64_t record_begin_time_ns_ = 0;
  bool loop_ = false;
  std::atomic<double> current_time_sec_{0.0};
  std::atomic<double> progress_{0.0};
  std::atomic<bool> playing_{false};
  std::atomic<bool> paused_{false};
  int channel_count_ = 0;
  std::vector<std::string> channel_names_;
  std::mutex mutex_;
};

}  // namespace integration
}  // namespace autoviz
