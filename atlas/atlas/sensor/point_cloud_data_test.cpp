#include "sensor/point_cloud_data.hpp"

#include <gtest/gtest.h>

#include <cstring>

#include "sensor/point_cloud.hpp"
#include "sensor/point_types.hpp"
#include "sensor/timed_point_cloud_data.hpp"

namespace atlas {
namespace sensor {

TEST(PointCloudDataTest, UsesSecondsRelativeToLastReturn) {
  PointCloudXYZI cloud;
  PointXYZINormal first_point;
  first_point.x = 1.f;
  first_point.curvature = 0.010f;
  first_point.intensity = 2.f;
  PointXYZINormal last_point;
  last_point.x = 3.f;
  last_point.curvature = 0.025f;
  last_point.intensity = 4.f;
  cloud.points = {first_point, last_point};

  const Eigen::Vector3f origin(0.1f, 0.2f, 0.3f);
  const TimedPointCloudData result = ToTimedPointCloudData(cloud, 12.5, origin);

  ASSERT_EQ(result.ranges.size(), 2u);
  ASSERT_EQ(result.intensities.size(), 2u);
  EXPECT_DOUBLE_EQ(result.time, 12.5);
  EXPECT_FLOAT_EQ(result.origin.x(), origin.x());
  EXPECT_FLOAT_EQ(result.ranges[0].time, -0.015f);
  EXPECT_FLOAT_EQ(result.ranges[1].time, 0.f);
  EXPECT_FLOAT_EQ(result.intensities[0], 2.f);
  EXPECT_FLOAT_EQ(result.intensities[1], 4.f);
}

TEST(PointCloudDataTest, ConvertsExplicitMillisecondFieldToSeconds) {
  ::automsgs::msgs::sensor_msgs::PointCloud2 message;
  message.mutable_header()->mutable_stamp()->set_sec(10);
  message.set_height(1);
  message.set_width(1);
  message.set_point_step(20);
  message.set_row_step(20);
  message.set_is_dense(true);
  const char* field_names[] = {"x", "y", "z", "intensity", "time_ms"};
  for (std::uint32_t index = 0; index < 5; ++index) {
    ::automsgs::msgs::sensor_msgs::PointField* field = message.add_fields();
    field->set_name(field_names[index]);
    field->set_offset(index * sizeof(float));
    field->set_datatype(::automsgs::msgs::sensor_msgs::PointField::FLOAT32);
    field->set_count(1);
  }
  const float point_values[] = {1.0f, 2.0f, 3.0f, 4.0f, 25.0f};
  message.set_data(reinterpret_cast<const char*>(point_values),
                   sizeof(point_values));

  PointCloudXYZI cloud;
  FromProto(message, &cloud);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud.points.front().curvature, 0.025f);
}

TEST(PointCloudDataTest, PreservesRelativeSecondMessageTime) {
  ::automsgs::msgs::sensor_msgs::PointCloud2 message;
  message.mutable_header()->mutable_stamp()->set_sec(10);
  message.set_height(1);
  message.set_width(1);
  message.set_point_step(20);
  message.set_row_step(20);
  message.set_is_dense(true);
  const char* field_names[] = {"x", "y", "z", "intensity", "time"};
  for (std::uint32_t index = 0; index < 5; ++index) {
    auto* field = message.add_fields();
    field->set_name(field_names[index]);
    field->set_offset(index * sizeof(float));
    field->set_datatype(::automsgs::msgs::sensor_msgs::PointField::FLOAT32);
    field->set_count(1);
  }
  const float point_values[] = {1.0f, 2.0f, 3.0f, 4.0f, 0.025f};
  message.set_data(reinterpret_cast<const char*>(point_values),
                   sizeof(point_values));

  PointCloudXYZI cloud;
  FromProto(message, &cloud);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud.points.front().curvature, 0.025f);
}

}  // namespace sensor
}  // namespace atlas
