/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/stream_envelope.h"

#include <gtest/gtest.h>

using autonomy::orbisview::core::ChannelInfo;
using autonomy::orbisview::core::ChannelListToJson;
using autonomy::orbisview::core::JsonEscape;
using autonomy::orbisview::core::StreamEnvelope;
using autonomy::orbisview::core::StreamEnvelopeToJson;

TEST(StreamEnvelopeTest, JsonEscapeQuotes) {
  EXPECT_EQ(JsonEscape("a\"b"), "\"a\\\"b\"");
}

TEST(StreamEnvelopeTest, JsonRoundTripFields) {
  StreamEnvelope env;
  env.channel = "/orbisview/mock/pose";
  env.schema = "orbisview.render.Pose2D";
  env.timestamp_ns = 123456789;
  env.frame_id = "map";
  env.sequence = 7;
  env.encoding = "json";
  const std::string payload = "{\"x\":1.0,\"y\":2.0,\"yaw\":0.5}";
  env.payload.assign(payload.begin(), payload.end());
  env.stale = true;

  const std::string json = StreamEnvelopeToJson(env);
  EXPECT_NE(json.find("\"op\":\"envelope\""), std::string::npos);
  EXPECT_NE(json.find("\"channel\":\"/orbisview/mock/pose\""), std::string::npos);
  EXPECT_NE(json.find("\"schema\":\"orbisview.render.Pose2D\""), std::string::npos);
  EXPECT_NE(json.find("\"timestamp\":123456789"), std::string::npos);
  EXPECT_NE(json.find("\"sequence\":7"), std::string::npos);
  EXPECT_NE(json.find("\"stale\":true"), std::string::npos);
  EXPECT_NE(json.find("\"payload\":{\"x\":1.0"), std::string::npos);
}

TEST(StreamEnvelopeTest, ChannelListJson) {
  std::vector<ChannelInfo> channels = {
      {"/a", "s", "t", true, true},
  };
  const std::string json = ChannelListToJson(channels);
  EXPECT_NE(json.find("\"op\":\"channels\""), std::string::npos);
  EXPECT_NE(json.find("\"name\":\"/a\""), std::string::npos);
  EXPECT_NE(json.find("\"mock\":true"), std::string::npos);
}
