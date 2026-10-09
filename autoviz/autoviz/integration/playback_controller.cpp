/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/playback_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/writer.hpp"
#include "autolink/proto/record.pb.h"
#include "autolink/record/record_reader.hpp"
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autoviz/integration/channel_reader_registry.hpp"

namespace autoviz {
namespace integration {
namespace {

constexpr char kRecordInfoChannel[] = "/autolink/record_info";
constexpr int32_t kMarkerDelete = 2;
constexpr int32_t kMarkerDeleteAll = 3;
constexpr int kDensityBinCount = 64;
constexpr size_t kMaxDensityScanMessages = 250000;

bool IsTfChannel(const ::autolink::record::RecordReader& reader,
                 const std::string& channel) {
  if (channel == "/tf" || channel == "/tf_static") {
    return true;
  }
  const std::string message_type = reader.GetMessageType(channel);
  return message_type.find("tf2_msgs.TFMessage") != std::string::npos ||
         message_type.find("automsgs.msgs.tf2_msgs.TFMessage") !=
             std::string::npos;
}

bool IsMarkerChannel(const ::autolink::record::RecordReader& reader,
                     const std::string& channel) {
  return reader.GetMessageType(channel).find("visualization_msgs.Marker") !=
         std::string::npos;
}

struct MarkerPreviewKey {
  std::string channel;
  std::string ns;
  int32_t id = 0;

  bool operator==(const MarkerPreviewKey& other) const {
    return channel == other.channel && ns == other.ns && id == other.id;
  }
};

struct MarkerPreviewKeyHash {
  std::size_t operator()(const MarkerPreviewKey& key) const {
    const std::hash<std::string> hasher;
    return hasher(key.channel) ^ (hasher(key.ns) << 1) ^
           (static_cast<std::size_t>(key.id) << 2);
  }
};

}  // namespace

PlaybackController::~PlaybackController() {
  stop();
}

void PlaybackController::setNode(
    const std::shared_ptr<::autolink::Node>& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
}

void PlaybackController::stopInfoReader() {
  if (info_subscription_id_ != 0) {
    ChannelReaderRegistry::instance().unsubscribe(info_subscription_id_);
    info_subscription_id_ = 0;
  }
}

double PlaybackController::clampToRangeLocked(double time_s) const {
  const double lo = range_start_sec_;
  double hi = range_end_sec_;
  if (hi < lo) {
    hi = lo;
  }
  if (total_time_sec_ > 0.0) {
    hi = std::min(hi, total_time_sec_);
  }
  return std::clamp(time_s, lo, hi);
}

void PlaybackController::rebuildDensityLocked() {
  density_bins_.assign(kDensityBinCount, 0.0f);
  if (current_file_.empty() || total_time_sec_ <= 0.0 ||
      record_begin_time_ns_ == 0) {
    return;
  }
  ::autolink::record::RecordReader reader(current_file_);
  if (!reader.IsValid()) {
    return;
  }
  std::vector<uint32_t> counts(kDensityBinCount, 0);
  ::autolink::record::RecordMessage message;
  size_t scanned = 0;
  reader.Reset();
  while (reader.ReadMessage(&message) && scanned < kMaxDensityScanMessages) {
    if (message.time < record_begin_time_ns_) {
      ++scanned;
      continue;
    }
    const double t =
        static_cast<double>(message.time - record_begin_time_ns_) / 1e9;
    int bin = static_cast<int>(std::floor((t / total_time_sec_) * kDensityBinCount));
    bin = std::clamp(bin, 0, kDensityBinCount - 1);
    ++counts[static_cast<size_t>(bin)];
    ++scanned;
  }
  uint32_t peak = 0;
  for (uint32_t c : counts) {
    peak = std::max(peak, c);
  }
  if (peak == 0) {
    return;
  }
  for (int i = 0; i < kDensityBinCount; ++i) {
    density_bins_[static_cast<size_t>(i)] =
        static_cast<float>(counts[static_cast<size_t>(i)]) /
        static_cast<float>(peak);
  }
}

bool PlaybackController::startPlayerLocked(double start_time_s) {
  if (current_file_.empty() || node_ == nullptr) {
    return false;
  }
  if (player_ != nullptr) {
    player_->Stop();
    player_.reset();
  }
  stopInfoReader();

  const double start = clampToRangeLocked(start_time_s);
  play_param_ = ::autolink::record::PlayParam{};
  play_param_.play_rate = play_rate_;
  play_param_.is_loop_playback = loop_;
  play_param_.is_play_all_channels = true;
  play_param_.black_channels = excluded_channels_;
  play_param_.start_time_s = start;
  play_param_.files_to_play.insert(current_file_);
  play_param_.record_id = current_file_;
  if (hasPlaybackRange() && record_begin_time_ns_ > 0) {
    play_param_.end_time_ns =
        record_begin_time_ns_ +
        static_cast<uint64_t>(std::max(0.0, range_end_sec_) * 1e9);
  }

  player_ = std::make_unique<::autolink::record::Player>(play_param_, node_,
                                                         true);
  if (!player_->Init() || !player_->PreloadPlayRecord(start, paused_)) {
    player_.reset();
    return false;
  }
  current_time_sec_ = start;
  progress_ = total_time_sec_ > 0.0 ? start / total_time_sec_ : 0.0;

  // Share /autolink/record_info via the registry so Channels probe / other
  // subscribers cannot steal a second Node reader (croutine name collision).
  info_subscription_id_ = ChannelReaderRegistry::instance().subscribe(
      kRecordInfoChannel, [this](const std::string& payload) {
        ::autolink::proto::RecordInfo info;
        if (!info.ParseFromString(payload)) {
          return;
        }
        current_time_sec_.store(info.curr_time_s());
        if (info.total_time_s() > 0.0) {
          total_time_sec_ = info.total_time_s();
        }
        progress_.store(info.progress());
      });

  player_->NohupPlayRecord();
  playing_ = true;
  return true;
}

bool PlaybackController::previewAtLocked(double time_s) {
  if (current_file_.empty() || node_ == nullptr) {
    return false;
  }

  ::autolink::record::RecordReader reader(current_file_);
  if (!reader.IsValid()) {
    return false;
  }

  const auto& header = reader.GetHeader();
  if (record_begin_time_ns_ == 0) {
    record_begin_time_ns_ = header.begin_time();
  }
  const uint64_t begin_ns = record_begin_time_ns_;
  const uint64_t end_ns =
      begin_ns + static_cast<uint64_t>(std::max(0.0, time_s) * 1e9);

  std::unordered_map<
      std::string,
      std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>>
      writers;
  std::unordered_map<std::string, std::string> latest_messages;
  std::unordered_map<MarkerPreviewKey, std::string, MarkerPreviewKeyHash>
      latest_markers;

  auto publish = [&](const std::string& channel, const std::string& content) {
    auto writer_it = writers.find(channel);
    if (writer_it == writers.end()) {
      ::autolink::proto::RoleAttributes attr;
      attr.set_channel_name(channel);
      attr.set_message_type(reader.GetMessageType(channel));
      const std::string& proto_desc = reader.GetProtoDesc(channel);
      if (!proto_desc.empty()) {
        attr.set_proto_desc(proto_desc);
      }
      auto writer = node_->CreateWriter<::autolink::message::RawMessage>(attr);
      if (writer == nullptr) {
        return;
      }
      writers.emplace(channel, writer);
      writer_it = writers.find(channel);
    }
    if (writer_it != writers.end() && writer_it->second != nullptr) {
      writer_it->second->Write(
          std::make_shared<::autolink::message::RawMessage>(content));
    }
  };

  reader.Reset();
  ::autolink::record::RecordMessage message;
  while (reader.ReadMessage(&message, begin_ns, end_ns)) {
    if (IsTfChannel(reader, message.channel_name)) {
      publish(message.channel_name, message.content);
    } else if (IsMarkerChannel(reader, message.channel_name)) {
      automsgs::msgs::visualization_msgs::Marker marker;
      if (!marker.ParseFromString(message.content)) {
        continue;
      }
      const MarkerPreviewKey key{message.channel_name, marker.ns(),
                                 marker.id()};
      if (marker.action() == kMarkerDelete) {
        latest_markers.erase(key);
        continue;
      }
      if (marker.action() == kMarkerDeleteAll) {
        for (auto it = latest_markers.begin(); it != latest_markers.end();) {
          if (it->first.channel != message.channel_name) {
            ++it;
            continue;
          }
          if (!marker.ns().empty() && it->first.ns != marker.ns()) {
            ++it;
            continue;
          }
          it = latest_markers.erase(it);
        }
        continue;
      }
      latest_markers[key] = message.content;
    } else {
      latest_messages[message.channel_name] = message.content;
    }
  }
  for (const auto& entry : latest_messages) {
    publish(entry.first, entry.second);
  }
  for (const auto& entry : latest_markers) {
    publish(entry.first.channel, entry.second);
  }

  current_time_sec_ = time_s;
  progress_ = total_time_sec_ > 0.0 ? time_s / total_time_sec_ : 0.0;
  return true;
}

bool PlaybackController::openFile(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (player_ != nullptr) {
    player_->Stop();
    player_.reset();
  }
  stopInfoReader();
  playing_ = false;
  paused_ = false;
  current_time_sec_ = 0.0;
  progress_ = 0.0;
  total_time_sec_ = 0.0;
  record_begin_time_ns_ = 0;
  range_start_sec_ = 0.0;
  range_end_sec_ = 0.0;
  channel_count_ = 0;
  channel_names_.clear();
  channel_types_.clear();
  channel_message_counts_.clear();
  density_bins_.clear();
  ::autolink::record::RecordReader reader(path);
  if (!reader.IsValid()) {
    current_file_.clear();
    return false;
  }

  current_file_ = path;
  const std::set<std::string> channels = reader.GetChannelList();
  channel_names_.assign(channels.begin(), channels.end());
  channel_count_ = static_cast<int>(channel_names_.size());
  for (const std::string& channel : channel_names_) {
    channel_types_[channel] = reader.GetMessageType(channel);
    channel_message_counts_[channel] = reader.GetMessageNumber(channel);
  }
  const auto& header = reader.GetHeader();
  record_begin_time_ns_ = header.begin_time();
  if (header.end_time() > header.begin_time()) {
    total_time_sec_ =
        static_cast<double>(header.end_time() - header.begin_time()) / 1e9;
  }
  range_start_sec_ = 0.0;
  range_end_sec_ = total_time_sec_;
  rebuildDensityLocked();
  // Do not preview here: callers should bind Displays / subscribe first so
  // writers have readers (avoids "write message failed" storms + races).
  return true;
}

std::string PlaybackController::channelMessageType(
    const std::string& channel) const {
  const auto it = channel_types_.find(channel);
  if (it == channel_types_.end()) {
    return {};
  }
  return it->second;
}

bool PlaybackController::play(double rate, bool loop) {
  std::lock_guard<std::mutex> lock(mutex_);
  play_rate_ = rate;
  loop_ = loop;
  paused_ = false;
  const double start = clampToRangeLocked(current_time_sec_.load());
  progress_ = total_time_sec_ > 0.0 ? start / total_time_sec_ : 0.0;
  return startPlayerLocked(start);
}

bool PlaybackController::seekTo(double time_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double clamped = clampToRangeLocked(
      total_time_sec_ > 0.0 ? std::max(0.0, std::min(time_s, total_time_sec_))
                            : std::max(0.0, time_s));
  current_time_sec_ = clamped;
  progress_ = total_time_sec_ > 0.0 ? clamped / total_time_sec_ : 0.0;

  if (!playing_) {
    return previewAtLocked(clamped);
  }
  return startPlayerLocked(clamped);
}

bool PlaybackController::scrubTo(double time_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double clamped = clampToRangeLocked(
      total_time_sec_ > 0.0 ? std::max(0.0, std::min(time_s, total_time_sec_))
                            : std::max(0.0, time_s));
  current_time_sec_ = clamped;
  progress_ = total_time_sec_ > 0.0 ? clamped / total_time_sec_ : 0.0;
  return previewAtLocked(clamped);
}

void PlaybackController::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (player_ != nullptr && playing_ && !paused_) {
    player_->HandleNohupThreadStatus();
    paused_ = true;
  }
}

void PlaybackController::resume() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (player_ != nullptr && playing_ && paused_) {
    player_->HandleNohupThreadStatus();
    paused_ = false;
  }
}

void PlaybackController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (player_ != nullptr) {
    player_->Stop();
    player_.reset();
  }
  stopInfoReader();
  playing_ = false;
  paused_ = false;
  current_time_sec_ = range_start_sec_;
  progress_ =
      total_time_sec_ > 0.0 ? range_start_sec_ / total_time_sec_ : 0.0;
  // Keep current_file_ / channel metadata so Stop in the Record panel can
  // return to range start without forcing the user to reopen the file.
}

void PlaybackController::setLoop(bool loop) {
  std::lock_guard<std::mutex> lock(mutex_);
  loop_ = loop;
  if (playing_) {
    const double start = current_time_sec_.load();
    startPlayerLocked(start);
  }
}

void PlaybackController::setPlayRate(double rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  play_rate_ = std::max(0.05, std::min(rate, 16.0));
  if (playing_ && !paused_) {
    const double start = current_time_sec_.load();
    startPlayerLocked(start);
  }
}

void PlaybackController::setExcludedChannels(std::set<std::string> channels) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (channels == excluded_channels_) {
    return;
  }
  excluded_channels_ = std::move(channels);
  if (playing_) {
    const double start = current_time_sec_.load();
    startPlayerLocked(start);
  }
}

void PlaybackController::setPlaybackRange(double start_s, double end_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (total_time_sec_ <= 0.0) {
    range_start_sec_ = 0.0;
    range_end_sec_ = 0.0;
    return;
  }
  double lo = std::clamp(std::min(start_s, end_s), 0.0, total_time_sec_);
  double hi = std::clamp(std::max(start_s, end_s), 0.0, total_time_sec_);
  if (hi - lo < 0.05) {
    hi = std::min(total_time_sec_, lo + 0.05);
  }
  range_start_sec_ = lo;
  range_end_sec_ = hi;
  current_time_sec_ = clampToRangeLocked(current_time_sec_.load());
  progress_ = total_time_sec_ > 0.0 ? current_time_sec_.load() / total_time_sec_
                                    : 0.0;
  if (playing_) {
    startPlayerLocked(current_time_sec_.load());
  }
}

void PlaybackController::clearPlaybackRange() {
  std::lock_guard<std::mutex> lock(mutex_);
  range_start_sec_ = 0.0;
  range_end_sec_ = total_time_sec_;
  if (playing_) {
    startPlayerLocked(clampToRangeLocked(current_time_sec_.load()));
  }
}

bool PlaybackController::hasPlaybackRange() const {
  if (total_time_sec_ <= 0.0) {
    return false;
  }
  constexpr double kEps = 1e-3;
  return range_start_sec_ > kEps ||
         range_end_sec_ < total_time_sec_ - kEps;
}

uint64_t PlaybackController::channelMessageCount(
    const std::string& channel) const {
  const auto it = channel_message_counts_.find(channel);
  if (it == channel_message_counts_.end()) {
    return 0;
  }
  return it->second;
}

bool PlaybackController::stepMessage(bool forward,
                                     const std::string& channel) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_file_.empty() || record_begin_time_ns_ == 0 ||
      total_time_sec_ <= 0.0) {
    return false;
  }
  ::autolink::record::RecordReader reader(current_file_);
  if (!reader.IsValid()) {
    return false;
  }

  const uint64_t begin_ns =
      record_begin_time_ns_ +
      static_cast<uint64_t>(std::max(0.0, range_start_sec_) * 1e9);
  const uint64_t end_ns =
      record_begin_time_ns_ +
      static_cast<uint64_t>(std::max(0.0, range_end_sec_) * 1e9);
  const uint64_t now_ns =
      record_begin_time_ns_ +
      static_cast<uint64_t>(
          std::max(0.0, current_time_sec_.load()) * 1e9);

  auto accept = [&](const ::autolink::record::RecordMessage& message) {
    if (!channel.empty() && message.channel_name != channel) {
      return false;
    }
    if (channel.empty() &&
        excluded_channels_.find(message.channel_name) !=
            excluded_channels_.end()) {
      return false;
    }
    if (message.channel_name.rfind("/autolink/", 0) == 0) {
      return false;
    }
    return message.time >= begin_ns && message.time <= end_ns;
  };

  uint64_t target_ns = 0;
  bool found = false;
  ::autolink::record::RecordMessage message;
  reader.Reset();
  if (forward) {
    const uint64_t from =
        now_ns < std::numeric_limits<uint64_t>::max() ? now_ns + 1 : now_ns;
    while (reader.ReadMessage(&message, from, end_ns)) {
      if (!accept(message)) {
        continue;
      }
      target_ns = message.time;
      found = true;
      break;
    }
  } else {
    const uint64_t to = now_ns > begin_ns ? now_ns - 1 : begin_ns;
    while (reader.ReadMessage(&message, begin_ns, to)) {
      if (!accept(message)) {
        continue;
      }
      target_ns = message.time;
      found = true;
    }
  }
  if (!found || target_ns < record_begin_time_ns_) {
    return false;
  }
  const double t =
      static_cast<double>(target_ns - record_begin_time_ns_) / 1e9;
  const double clamped = clampToRangeLocked(t);
  current_time_sec_ = clamped;
  progress_ = total_time_sec_ > 0.0 ? clamped / total_time_sec_ : 0.0;
  if (!playing_) {
    return previewAtLocked(clamped);
  }
  return startPlayerLocked(clamped);
}

}  // namespace integration
}  // namespace autoviz
