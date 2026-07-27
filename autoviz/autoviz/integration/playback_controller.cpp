/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/playback_controller.hpp"

#include <unordered_map>
#include <set>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/writer.hpp"
#include "autolink/proto/record.pb.h"
#include "autolink/record/record_reader.hpp"
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autoviz {
namespace integration {
namespace {

constexpr char kRecordInfoChannel[] = "/autolink/record_info";
constexpr int32_t kMarkerDelete = 2;
constexpr int32_t kMarkerDeleteAll = 3;

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
  info_reader_.reset();
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

  play_param_ = ::autolink::record::PlayParam{};
  play_param_.play_rate = play_rate_;
  play_param_.is_loop_playback = loop_;
  play_param_.is_play_all_channels = true;
  play_param_.start_time_s = start_time_s;
  play_param_.files_to_play.insert(current_file_);
  play_param_.record_id = current_file_;

  player_ = std::make_unique<::autolink::record::Player>(play_param_, node_,
                                                         true);
  if (!player_->Init() || !player_->PreloadPlayRecord(start_time_s, paused_)) {
    player_.reset();
    return false;
  }

  info_reader_ =
      node_->CreateReader<::autolink::message::RawMessage>(
          kRecordInfoChannel,
          [this](const std::shared_ptr<::autolink::message::RawMessage>& msg) {
            if (msg == nullptr) {
              return;
            }
            ::autolink::proto::RecordInfo info;
            if (!info.ParseFromString(msg->message)) {
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
  channel_count_ = 0;
  channel_names_.clear();
  ::autolink::record::RecordReader reader(path);
  if (!reader.IsValid()) {
    current_file_.clear();
    return false;
  }

  current_file_ = path;
  const std::set<std::string> channels = reader.GetChannelList();
  channel_names_.assign(channels.begin(), channels.end());
  channel_count_ = static_cast<int>(channel_names_.size());
  const auto& header = reader.GetHeader();
  record_begin_time_ns_ = header.begin_time();
  if (header.end_time() > header.begin_time()) {
    total_time_sec_ =
        static_cast<double>(header.end_time() - header.begin_time()) / 1e9;
  }
  previewAtLocked(0.0);
  return true;
}

bool PlaybackController::play(double rate, bool loop) {
  std::lock_guard<std::mutex> lock(mutex_);
  play_rate_ = rate;
  loop_ = loop;
  paused_ = false;
  const double start = current_time_sec_.load();
  progress_ = total_time_sec_ > 0.0 ? start / total_time_sec_ : 0.0;
  return startPlayerLocked(start);
}

bool PlaybackController::seekTo(double time_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double clamped =
      total_time_sec_ > 0.0 ? std::max(0.0, std::min(time_s, total_time_sec_))
                            : std::max(0.0, time_s);
  current_time_sec_ = clamped;
  progress_ = total_time_sec_ > 0.0 ? clamped / total_time_sec_ : 0.0;

  if (!playing_) {
    return previewAtLocked(clamped);
  }
  return startPlayerLocked(clamped);
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
  current_time_sec_ = 0.0;
  progress_ = 0.0;
  channel_count_ = 0;
  channel_names_.clear();
}

}  // namespace integration
}  // namespace autoviz
