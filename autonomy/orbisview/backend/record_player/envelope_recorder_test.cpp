/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/record_player/envelope_recorder.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

#include <gtest/gtest.h>

using autonomy::orbisview::backend::EnvelopeRecorder;
using autonomy::orbisview::backend::PlaybackOptions;
using autonomy::orbisview::backend::RecordOptions;
using autonomy::orbisview::core::StreamEnvelope;

namespace {

std::string TempBag() {
  return "/tmp/orbisview_recorder_test_" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()) +
         ".jsonl";
}

StreamEnvelope MakeEnv(const std::string& channel, int64_t ts, uint64_t seq) {
  StreamEnvelope env;
  env.channel = channel;
  env.schema = "orbisview.render.Pose2D";
  env.timestamp_ns = ts;
  env.frame_id = "map";
  env.sequence = seq;
  env.encoding = "json";
  const std::string payload = "{\"x\":1,\"y\":2,\"yaw\":0}";
  env.payload.assign(payload.begin(), payload.end());
  return env;
}

}  // namespace

TEST(EnvelopeRecorderTest, RecordIndexSeek) {
  const std::string path = TempBag();
  EnvelopeRecorder rec;

  RecordOptions ro;
  ro.path = path;
  ro.channels = {"/a"};
  ASSERT_TRUE(rec.StartRecording(ro));
  rec.Append(MakeEnv("/a", 1000, 1));
  rec.Append(MakeEnv("/b", 2000, 2));  // filtered out
  rec.Append(MakeEnv("/a", 3000, 3));
  rec.StopRecording();

  ASSERT_TRUE(rec.EnsureIndex(path));
  const std::string index_json = rec.IndexJson(path);
  EXPECT_NE(index_json.find("\"count\":2"), std::string::npos);

  std::atomic<int> played{0};
  PlaybackOptions po;
  po.path = path;
  po.speed = 100.0;
  po.start_index = 1;
  ASSERT_TRUE(rec.StartPlayback(po, [&](StreamEnvelope) { ++played; }));
  for (int i = 0; i < 50 && rec.Playing(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  rec.StopPlayback();
  EXPECT_GE(played.load(), 1);
  std::remove(path.c_str());
}
