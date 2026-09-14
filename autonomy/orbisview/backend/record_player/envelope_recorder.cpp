/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/record_player/envelope_recorder.hpp"

#include <chrono>
#include <sstream>

#include <glog/logging.h>

namespace autonomy {
namespace orbisview {
namespace backend {
namespace {

std::string ExtractJsonString(const std::string& text, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return {};
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return {};
  ++pos;
  while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) ++pos;
  if (pos >= text.size() || text[pos] != '"') return {};
  ++pos;
  std::string out;
  while (pos < text.size() && text[pos] != '"') {
    if (text[pos] == '\\' && pos + 1 < text.size()) {
      out.push_back(text[pos + 1]);
      pos += 2;
      continue;
    }
    out.push_back(text[pos++]);
  }
  return out;
}

int64_t ExtractJsonNumber(const std::string& text, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return 0;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return 0;
  try {
    return std::stoll(text.substr(pos + 1));
  } catch (...) {
    return 0;
  }
}

}  // namespace

bool EnvelopeRecorder::IsHeaderLine(const std::string& line) {
  return line.find("\"orbisview_bag_header\"") != std::string::npos ||
         line.find("\"op\":\"orbisview_bag_header\"") != std::string::npos;
}

bool EnvelopeRecorder::ParseEnvelopeLine(const std::string& line,
                                         core::StreamEnvelope* out) {
  if (IsHeaderLine(line)) return false;
  out->channel = ExtractJsonString(line, "channel");
  out->schema = ExtractJsonString(line, "schema");
  out->frame_id = ExtractJsonString(line, "frame_id");
  out->encoding = ExtractJsonString(line, "encoding");
  if (out->encoding.empty()) out->encoding = "json";
  out->timestamp_ns = ExtractJsonNumber(line, "timestamp");
  out->sequence = static_cast<uint64_t>(ExtractJsonNumber(line, "sequence"));
  auto p = line.find("\"payload\":");
  if (p != std::string::npos) {
    auto start = line.find_first_of("{[", p);
    if (start != std::string::npos) {
      const char open = line[start];
      const char close = open == '{' ? '}' : ']';
      int depth = 0;
      size_t i = start;
      for (; i < line.size(); ++i) {
        if (line[i] == open) ++depth;
        else if (line[i] == close) {
          --depth;
          if (depth == 0) {
            ++i;
            break;
          }
        }
      }
      const std::string payload = line.substr(start, i - start);
      out->payload.assign(payload.begin(), payload.end());
    }
  }
  return !out->channel.empty();
}

bool EnvelopeRecorder::OpenBagForAppendLocked(const std::string& path) {
  out_.open(path, std::ios::out | std::ios::trunc);
  return static_cast<bool>(out_);
}

bool EnvelopeRecorder::StartRecording(const RecordOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (recording_.load()) return false;
  if (!OpenBagForAppendLocked(options.path)) {
    LOG(ERROR) << "OrbisView recorder open failed: " << options.path;
    return false;
  }
  record_path_ = options.path;
  channel_filter_ = options.channels;
  recorded_ = 0;

  std::ostringstream header;
  header << "{\"op\":\"orbisview_bag_header\",\"version\":1"
         << ",\"created_ns\":"
         << std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count()
         << ",\"channels\":[";
  bool first = true;
  for (const auto& ch : channel_filter_) {
    if (!first) header << ',';
    first = false;
    header << core::JsonEscape(ch);
  }
  header << "]}";
  out_ << header.str() << '\n';
  recording_ = true;
  LOG(INFO) << "OrbisView recording -> " << options.path
            << " filter_size=" << channel_filter_.size();
  return true;
}

void EnvelopeRecorder::StopRecording() {
  std::lock_guard<std::mutex> lock(mutex_);
  recording_ = false;
  if (out_.is_open()) out_.close();
}

void EnvelopeRecorder::SetChannelFilter(std::unordered_set<std::string> channels) {
  std::lock_guard<std::mutex> lock(mutex_);
  channel_filter_ = std::move(channels);
}

void EnvelopeRecorder::Append(const core::StreamEnvelope& env) {
  if (!recording_.load()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out_.is_open()) return;
  if (!channel_filter_.empty() && !channel_filter_.count(env.channel)) {
    return;
  }
  out_ << core::StreamEnvelopeToJson(env) << '\n';
  ++recorded_;
}

bool EnvelopeRecorder::EnsureIndex(const std::string& path) {
  std::lock_guard<std::mutex> lock(index_mutex_);
  if (indexed_path_ == path && !index_.empty()) return true;
  std::ifstream in(path);
  if (!in) return false;
  index_.clear();
  indexed_path_ = path;
  uint64_t idx = 0;
  std::string line;
  while (true) {
    const std::streampos pos = in.tellg();
    if (!std::getline(in, line)) break;
    if (line.empty() || IsHeaderLine(line)) continue;
    core::StreamEnvelope env;
    if (!ParseEnvelopeLine(line, &env)) continue;
    index_.push_back(BagIndexEntry{idx, env.timestamp_ns, pos, env.channel});
    ++idx;
  }
  return true;
}

std::string EnvelopeRecorder::IndexJson(const std::string& path) const {
  std::lock_guard<std::mutex> lock(index_mutex_);
  std::ostringstream oss;
  oss << "{\"op\":\"bag_index\",\"path\":" << core::JsonEscape(path)
      << ",\"count\":" << index_.size() << ",\"entries\":[";
  const size_t max_show = std::min<size_t>(index_.size(), 64);
  for (size_t i = 0; i < max_show; ++i) {
    if (i) oss << ',';
    const auto& e = index_[i];
    oss << "{\"index\":" << e.index << ",\"timestamp\":" << e.timestamp_ns
        << ",\"channel\":" << core::JsonEscape(e.channel) << '}';
  }
  if (index_.size() > max_show) {
    oss << ",{\"index\":" << index_.back().index
        << ",\"timestamp\":" << index_.back().timestamp_ns
        << ",\"channel\":" << core::JsonEscape(index_.back().channel)
        << ",\"truncated\":true}";
  }
  oss << "]}";
  return oss.str();
}

bool EnvelopeRecorder::StartPlayback(const PlaybackOptions& options, EmitFn emit) {
  if (playing_.exchange(true)) return false;
  paused_ = false;
  seek_requested_ = false;
  play_path_ = options.path;
  play_speed_ = options.speed;
  play_loop_ = options.loop;
  play_index_ = 0;
  EnsureIndex(options.path);
  {
    std::lock_guard<std::mutex> lock(index_mutex_);
    play_total_ = index_.size();
  }
  play_thread_ = std::thread([this, options, emit] {
    PlaybackLoop(options, emit);
  });
  return true;
}

void EnvelopeRecorder::StopPlayback() {
  playing_ = false;
  paused_ = false;
  pause_cv_.notify_all();
  if (play_thread_.joinable()) play_thread_.join();
}

void EnvelopeRecorder::PausePlayback(bool paused) {
  paused_ = paused;
  if (!paused) pause_cv_.notify_all();
}

bool EnvelopeRecorder::SeekIndex(uint64_t index) {
  if (!EnsureIndex(play_path_.empty() ? indexed_path_ : play_path_)) {
    return false;
  }
  seek_index_ = index;
  seek_requested_ = true;
  paused_ = false;
  pause_cv_.notify_all();
  return true;
}

bool EnvelopeRecorder::SeekTimestamp(int64_t timestamp_ns) {
  if (!EnsureIndex(play_path_.empty() ? indexed_path_ : play_path_)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(index_mutex_);
  uint64_t best = 0;
  for (const auto& e : index_) {
    if (e.timestamp_ns >= timestamp_ns) {
      best = e.index;
      break;
    }
    best = e.index;
  }
  seek_index_ = best;
  seek_requested_ = true;
  paused_ = false;
  pause_cv_.notify_all();
  return true;
}

void EnvelopeRecorder::PlaybackLoop(PlaybackOptions options, EmitFn emit) {
  do {
    if (!EnsureIndex(options.path)) {
      LOG(ERROR) << "OrbisView playback index failed: " << options.path;
      break;
    }
    std::ifstream in(options.path);
    if (!in) {
      LOG(ERROR) << "OrbisView playback open failed: " << options.path;
      break;
    }

    uint64_t start = options.start_index;
    if (options.start_timestamp_ns > 0) {
      std::lock_guard<std::mutex> lock(index_mutex_);
      for (const auto& e : index_) {
        if (e.timestamp_ns >= options.start_timestamp_ns) {
          start = e.index;
          break;
        }
      }
    }

    auto seek_to = [&](uint64_t index) -> bool {
      std::lock_guard<std::mutex> lock(index_mutex_);
      if (index >= index_.size()) return false;
      in.clear();
      in.seekg(index_[index].file_pos);
      play_index_ = index;
      return true;
    };

    if (!seek_to(start)) {
      // Fall back to sequential from beginning.
      in.clear();
      in.seekg(0);
      play_index_ = 0;
    }

    LOG(INFO) << "OrbisView playback <- " << options.path
              << " start_index=" << play_index_ << " speed=" << options.speed
              << " loop=" << options.loop;

    int64_t prev_ts = 0;
    std::string line;
    while (playing_.load()) {
      if (seek_requested_.exchange(false)) {
        if (!seek_to(seek_index_.load())) {
          break;
        }
        prev_ts = 0;
        continue;
      }
      if (paused_.load()) {
        std::unique_lock<std::mutex> lock(mutex_);
        pause_cv_.wait(lock, [this] {
          return !paused_.load() || !playing_.load() || seek_requested_.load();
        });
        continue;
      }
      if (!std::getline(in, line)) break;
      if (line.empty() || IsHeaderLine(line)) continue;
      core::StreamEnvelope env;
      if (!ParseEnvelopeLine(line, &env)) continue;
      if (prev_ts > 0 && env.timestamp_ns > prev_ts && options.speed > 0.0) {
        const double dt_ms =
            static_cast<double>(env.timestamp_ns - prev_ts) / 1.0e6 /
            options.speed;
        if (dt_ms > 0 && dt_ms < 5000) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>(dt_ms)));
        }
      }
      prev_ts = env.timestamp_ns;
      ++play_index_;
      if (emit) emit(std::move(env));
    }
  } while (playing_.load() && options.loop);

  playing_ = false;
  paused_ = false;
}

std::string EnvelopeRecorder::StatusJson() const {
  std::ostringstream oss;
  oss << "{\"op\":\"recorder_status\",\"recording\":"
      << (recording_.load() ? "true" : "false") << ",\"playing\":"
      << (playing_.load() ? "true" : "false") << ",\"paused\":"
      << (paused_.load() ? "true" : "false")
      << ",\"recorded\":" << recorded_
      << ",\"play_index\":" << play_index_
      << ",\"play_total\":" << play_total_
      << ",\"speed\":" << play_speed_
      << ",\"loop\":" << (play_loop_ ? "true" : "false")
      << ",\"record_path\":" << core::JsonEscape(record_path_)
      << ",\"play_path\":" << core::JsonEscape(play_path_) << '}';
  return oss.str();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
