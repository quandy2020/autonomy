/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file playback_controller.hpp
 * @brief Autolink record player control for Autoviz Time / Playback UI.
 *
 * Wraps @c autolink::record::Player with open / play / pause / seek / stop and
 * exposes progress, channel list, and timing for the playback chrome.
 *
 * @see AutolinkContext
 * @see MessageQueue::setAcceptIncoming
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/tools/recorder/player/play_param.hpp"
#include "autolink/tools/recorder/player/player.hpp"

namespace autoviz {
namespace integration {

/**
 * @class PlaybackController
 * @brief UI-facing controller for replaying Autolink record files.
 *
 * ## Typical flow
 *
 * @code
 *   setNode(ctx.node());
 *   openFile(path);
 *   play(rate, loop);
 *   // UI polls currentTimeSec() / progress()
 *   pause(); resume(); seekTo(t); stop();
 * @endcode
 *
 * @note @c playing_ / @c paused_ / time / progress are atomics so the UI
 *       thread may read them without taking @c mutex_; mutating APIs lock.
 */
class PlaybackController {
 public:
  /** @brief Default-constructs an idle controller (no open file). */
  PlaybackController() = default;

  /**
   * @brief Stops playback and releases the player / info reader.
   */
  ~PlaybackController();

  /**
   * @brief Attaches the Autolink node used by the record player.
   *
   * @param node Shared node from @ref AutolinkContext (required before open/play).
   */
  void setNode(const std::shared_ptr<::autolink::Node>& node);

  /**
   * @brief Opens a record file and loads channel / duration metadata.
   *
   * @param path Filesystem path to an Autolink record.
   * @return @c true if the file was opened and metadata loaded.
   */
  bool openFile(const std::string& path);

  /**
   * @brief Starts or restarts playback at @p rate.
   *
   * @param rate Playback speed multiplier (1.0 = realtime).
   * @param loop When @c true, restart from the beginning at EOF.
   * @return @c true if the player started successfully.
   */
  bool play(double rate = 1.0, bool loop = false);

  /**
   * @brief Seeks to @p time_s seconds from the start of the record.
   *
   * May preview / restart the player depending on current state.
   *
   * @param time_s Absolute time within the record (seconds).
   * @return @c true if seek succeeded.
   */
  bool seekTo(double time_s);

  /** @brief Pauses an active player without closing the file. */
  void pause();

  /** @brief Resumes after @ref pause(). */
  void resume();

  /** @brief Stops playback and resets playing/paused flags. */
  void stop();

  /**
   * @brief Whether the player is currently playing (not stopped).
   * @return Value of @c playing_.
   */
  bool isPlaying() const { return playing_; }

  /**
   * @brief Whether playback is paused.
   * @return Value of @c paused_.
   */
  bool isPaused() const { return paused_; }

  /**
   * @brief Whether loop mode is enabled for the current session.
   * @return Value of @c loop_.
   */
  bool loop() const { return loop_; }

  /**
   * @brief Path of the currently opened record file.
   * @return Const reference to @c current_file_ (empty if none).
   */
  const std::string& currentFile() const { return current_file_; }

  /**
   * @brief Current playback rate multiplier.
   * @return Value of @c play_rate_.
   */
  double playRate() const { return play_rate_; }

  /**
   * @brief Total duration of the opened record in seconds.
   * @return Value of @c total_time_sec_.
   */
  double totalTimeSec() const { return total_time_sec_; }

  /**
   * @brief Current playback position in seconds (UI-readable atomic).
   * @return Loaded value of @c current_time_sec_.
   */
  double currentTimeSec() const { return current_time_sec_.load(); }

  /**
   * @brief Normalized progress in \[0, 1\] (UI-readable atomic).
   * @return Loaded value of @c progress_.
   */
  double progress() const { return progress_.load(); }

  /**
   * @brief Number of channels in the opened record.
   * @return Value of @c channel_count_.
   */
  int channelCount() const { return channel_count_; }

  /**
   * @brief Channel names discovered in the opened record.
   * @return Const reference to @c channel_names_.
   */
  const std::vector<std::string>& channelNames() const { return channel_names_; }

 private:
  /**
   * @brief Starts the Autolink player from @p start_time_s (mutex held).
   *
   * @param start_time_s Start offset in seconds.
   * @return @c true on success.
   */
  bool startPlayerLocked(double start_time_s);

  /**
   * @brief Seeks / previews content at @p time_s without full play (mutex held).
   *
   * @param time_s Target time in seconds.
   * @return @c true on success.
   */
  bool previewAtLocked(double time_s);

  /** Tears down @c info_reader_ used for progress / metadata updates. */
  void stopInfoReader();

  /** Autolink node for the player (shared with the app context). */
  std::shared_ptr<::autolink::Node> node_;

  /** Owned Autolink record player instance. */
  std::unique_ptr<::autolink::record::Player> player_;

  /** Play parameters passed to the player. */
  ::autolink::record::PlayParam play_param_;

  /** Optional reader used to observe playback info / clock. */
  std::shared_ptr<::autolink::Reader<::autolink::message::RawMessage>>
      info_reader_;

  /** Opened record path. */
  std::string current_file_;

  /** Playback speed multiplier. */
  double play_rate_ = 1.0;

  /** Record duration in seconds. */
  double total_time_sec_ = 0.0;

  /** Record begin time in nanoseconds (Autolink clock basis). */
  uint64_t record_begin_time_ns_ = 0;

  /** Whether to loop at EOF. */
  bool loop_ = false;

  /** Current position in seconds (updated from player / info callbacks). */
  std::atomic<double> current_time_sec_{0.0};

  /** Normalized progress \[0, 1\]. */
  std::atomic<double> progress_{0.0};

  /** @c true while a play session is active (including paused). */
  std::atomic<bool> playing_{false};

  /** @c true while paused within an active session. */
  std::atomic<bool> paused_{false};

  /** Number of channels in the record. */
  int channel_count_ = 0;

  /** Channel names from the record header / index. */
  std::vector<std::string> channel_names_;

  /** Guards player mutations and metadata writes. */
  std::mutex mutex_;
};

}  // namespace integration
}  // namespace autoviz
