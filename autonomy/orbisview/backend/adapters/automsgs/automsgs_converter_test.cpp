/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/adapters/automsgs/automsgs_converter.hpp"

#include <algorithm>
#include <cstring>
#include <gtest/gtest.h>

#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>

#include "autonomy/orbisview/backend/common/render_schemas.hpp"

using autonomy::orbisview::adapters::ConvertAutomsgsRaw;
using autonomy::orbisview::adapters::SuggestedRenderSchema;
using autonomy::orbisview::core::StreamEnvelope;

TEST(AutomsgsConverterTest, SuggestedSchema) {
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.geometry_msgs.Pose2D"),
            autonomy::orbisview::rendering::kSchemaPose);
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.sensor_msgs.PointCloud2"),
            autonomy::orbisview::rendering::kSchemaPointCloud2);
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.geometry_msgs.Polygon"),
            autonomy::orbisview::rendering::kSchemaFootprint);
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.geometry_msgs.PolygonStamped"),
            autonomy::orbisview::rendering::kSchemaFootprint);
  EXPECT_TRUE(SuggestedRenderSchema("something.unknown").empty());
}

TEST(AutomsgsConverterTest, Pose2DToJson) {
  automsgs::msgs::geometry_msgs::Pose2D pose;
  pose.set_x(1.5);
  pose.set_y(-2.0);
  pose.set_theta(0.25);
  std::string bytes;
  ASSERT_TRUE(pose.SerializeToString(&bytes));

  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw("/pose", "automsgs.msgs.geometry_msgs.Pose2D",
                                 bytes, 42, &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaPose);
  EXPECT_EQ(env.encoding, "json");
  EXPECT_FALSE(env.unsupported);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"x\":1.5"), std::string::npos);
  EXPECT_NE(json.find("\"y\":-2"), std::string::npos);
}

TEST(AutomsgsConverterTest, LaserScanToJson) {
  automsgs::msgs::sensor_msgs::LaserScan scan;
  scan.set_angle_min(-1.0f);
  scan.set_angle_increment(0.5f);
  scan.set_range_max(10.f);
  scan.add_ranges(1.0f);
  scan.add_ranges(2.0f);
  scan.mutable_header()->set_frame_id("laser");
  std::string bytes;
  ASSERT_TRUE(scan.SerializeToString(&bytes));

  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/scan", "automsgs.msgs.sensor_msgs.LaserScan", bytes, 1, &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaLaserScan);
  EXPECT_EQ(env.frame_id, "laser");
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"ranges\":[1,2]"), std::string::npos);
}

TEST(AutomsgsConverterTest, PointCloud2Downsample) {
  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
  cloud.set_height(1);
  cloud.set_width(4);
  cloud.set_point_step(12);
  cloud.set_row_step(48);
  cloud.set_is_dense(true);
  auto* fx = cloud.add_fields();
  fx->set_name("x");
  fx->set_offset(0);
  fx->set_datatype(7);
  fx->set_count(1);
  auto* fy = cloud.add_fields();
  fy->set_name("y");
  fy->set_offset(4);
  fy->set_datatype(7);
  fy->set_count(1);
  auto* fz = cloud.add_fields();
  fz->set_name("z");
  fz->set_offset(8);
  fz->set_datatype(7);
  fz->set_count(1);

  std::string data(48, '\0');
  for (int i = 0; i < 4; ++i) {
    float x = static_cast<float>(i);
    float y = 1.f;
    float z = 2.f;
    std::memcpy(&data[i * 12 + 0], &x, 4);
    std::memcpy(&data[i * 12 + 4], &y, 4);
    std::memcpy(&data[i * 12 + 8], &z, 4);
  }
  cloud.set_data(data);
  std::string bytes;
  ASSERT_TRUE(cloud.SerializeToString(&bytes));

  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/cloud", "automsgs.msgs.sensor_msgs.PointCloud2", bytes, 1, &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaPointCloud2);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"points\":["), std::string::npos);
}

TEST(AutomsgsConverterTest, UnknownFallsThrough) {
  StreamEnvelope env;
  EXPECT_FALSE(ConvertAutomsgsRaw("/x", "no.such.Type", "abc", 0, &env));
}

TEST(AutomsgsConverterTest, PolygonToFootprintJson) {
  automsgs::msgs::geometry_msgs::Polygon poly;
  auto* p0 = poly.add_points();
  p0->set_x(0.45f);
  p0->set_y(0.28f);
  auto* p1 = poly.add_points();
  p1->set_x(0.45f);
  p1->set_y(-0.28f);
  auto* p2 = poly.add_points();
  p2->set_x(-0.45f);
  p2->set_y(-0.28f);
  std::string bytes;
  ASSERT_TRUE(poly.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw("/footprint",
                                 "automsgs.msgs.geometry_msgs.Polygon", bytes,
                                 7, &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaFootprint);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"shape\":\"POLYGON\""), std::string::npos);
  EXPECT_NE(json.find("\"x\":0.45"), std::string::npos);
}

TEST(AutomsgsConverterTest, PolygonStampedToFootprintJson) {
  automsgs::msgs::geometry_msgs::PolygonStamped stamped;
  stamped.mutable_header()->set_frame_id("base_link");
  auto* poly = stamped.mutable_polygon();
  for (int i = 0; i < 3; ++i) {
    auto* p = poly->add_points();
    p->set_x(static_cast<float>(i));
    p->set_y(0.1f);
  }
  std::string bytes;
  ASSERT_TRUE(stamped.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/footprint", "automsgs.msgs.geometry_msgs.PolygonStamped", bytes, 1,
      &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaFootprint);
  EXPECT_EQ(env.frame_id, "base_link");
}

TEST(AutomsgsConverterTest, PolygonTooFewPointsFails) {
  automsgs::msgs::geometry_msgs::Polygon poly;
  poly.add_points()->set_x(0.f);
  std::string bytes;
  ASSERT_TRUE(poly.SerializeToString(&bytes));
  StreamEnvelope env;
  EXPECT_FALSE(ConvertAutomsgsRaw(
      "/footprint", "automsgs.msgs.geometry_msgs.Polygon", bytes, 1, &env));
}

TEST(AutomsgsConverterTest, Depth32FC1DecodesFloatMetres) {
  automsgs::msgs::sensor_msgs::Image image;
  image.set_width(2);
  image.set_height(1);
  image.set_encoding("32FC1");
  image.set_step(8);
  image.set_is_bigendian(false);
  const float depths[2] = {0.25f, 6.0f};
  image.mutable_data()->assign(reinterpret_cast<const char*>(depths),
                               reinterpret_cast<const char*>(depths) + 8);

  std::string bytes;
  ASSERT_TRUE(image.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/camera/depth/image_raw", "automsgs.msgs.sensor_msgs.Image", bytes, 1,
      &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaDepthImage);
  const std::string json(env.payload.begin(), env.payload.end());
  // Near / far map to ~0 and ~255 (not raw float bytes).
  EXPECT_NE(json.find("\"data_b64\""), std::string::npos) << json;
  // 0 and 255 as raw mono bytes → base64 "AP8="
  EXPECT_NE(json.find("AP8="), std::string::npos) << json;
}

TEST(AutomsgsConverterTest, Rgb8KeepsColorChannels) {
  automsgs::msgs::sensor_msgs::Image image;
  image.set_width(1);
  image.set_height(1);
  image.set_encoding("rgb8");
  image.set_step(3);
  image.mutable_data()->assign("\x10\x20\x30", 3);

  std::string bytes;
  ASSERT_TRUE(image.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/camera/rgb/image_raw", "automsgs.msgs.sensor_msgs.Image", bytes, 1,
      &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaImage);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"encoding\":\"rgb8\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"data_b64\""), std::string::npos) << json;
  // 0x10,0x20,0x30 → base64 "ECAw"
  EXPECT_NE(json.find("ECAw"), std::string::npos) << json;
}

TEST(AutomsgsConverterTest, Rgb8LabeledButRgbaStepStillSamplesRgb) {
  // Mis-labeled Habitat-style frame: encoding=rgb8, step=width*4, RGBA bytes.
  automsgs::msgs::sensor_msgs::Image image;
  image.set_width(2);
  image.set_height(1);
  image.set_encoding("rgb8");
  image.set_step(8);
  // px0=RGB(1,2,3)A255, px1=RGB(4,5,6)A255
  image.mutable_data()->assign("\x01\x02\x03\xff\x04\x05\x06\xff", 8);

  std::string bytes;
  ASSERT_TRUE(image.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw(
      "/camera/rgb/image_raw", "automsgs.msgs.sensor_msgs.Image", bytes, 1,
      &env));
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"encoding\":\"rgb8\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"data_b64\""), std::string::npos) << json;
  // RGB bytes 1,2,3,4,5,6 → "AQIDBAUG"
  EXPECT_NE(json.find("AQIDBAUG"), std::string::npos) << json;
}

TEST(AutomsgsConverterTest, ImuToJsonFields) {
  automsgs::msgs::sensor_msgs::Imu imu;
  imu.mutable_linear_acceleration()->set_x(0.1);
  imu.mutable_linear_acceleration()->set_y(0.2);
  imu.mutable_linear_acceleration()->set_z(9.8);
  imu.mutable_angular_velocity()->set_z(-0.05);
  std::string bytes;
  ASSERT_TRUE(imu.SerializeToString(&bytes));

  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw("/imu", "automsgs.msgs.sensor_msgs.Imu", bytes,
                                 7, &env));
  EXPECT_EQ(env.encoding, "json");
  EXPECT_FALSE(env.unsupported);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("linear_acceleration"), std::string::npos) << json;
  EXPECT_NE(json.find("\"x\":0.1"), std::string::npos) << json;
  EXPECT_NE(json.find("angular_velocity"), std::string::npos) << json;
}

TEST(AutomsgsConverterTest, ListImuNumericPaths) {
  using autonomy::orbisview::adapters::ListNumericProtoPaths;
  const auto paths = ListNumericProtoPaths("automsgs.msgs.sensor_msgs.Imu");
  EXPECT_FALSE(paths.empty());
  auto has = [&](const char* p) {
    return std::find(paths.begin(), paths.end(), p) != paths.end();
  };
  EXPECT_TRUE(has("linear_acceleration.x"));
  EXPECT_TRUE(has("angular_velocity.z"));
  EXPECT_TRUE(has("orientation.w"));
  EXPECT_TRUE(has("angular_velocity_covariance[0]"));
}
