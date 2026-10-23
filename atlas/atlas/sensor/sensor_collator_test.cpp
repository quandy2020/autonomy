#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "sensor/compressed_point_cloud.hpp"
#include "sensor/internal/collator.hpp"
#include "sensor/internal/dispatchable.hpp"
#include "sensor/map_by_time.hpp"

namespace atlas {
namespace sensor {
namespace {

struct Stamp {
  double time = 0.0;
};

TEST(Collator, DispatchesInTimeOrderAcrossSensors) {
  Collator collator;
  std::vector<double> times;
  collator.AddTrajectory(
      0, {"imu", "lidar"},
      [&times](const std::string&, std::unique_ptr<Data> data) {
        times.push_back(data->GetTime());
      });
  collator.AddSensorData(0, MakeDispatchable("lidar", Stamp{0.2}));
  collator.AddSensorData(0, MakeDispatchable("imu", Stamp{0.1}));
  collator.AddSensorData(0, MakeDispatchable("imu", Stamp{0.3}));
  collator.AddSensorData(0, MakeDispatchable("lidar", Stamp{0.4}));
  collator.FinishTrajectory(0);
  ASSERT_EQ(times.size(), 4u);
  EXPECT_DOUBLE_EQ(times[0], 0.1);
  EXPECT_DOUBLE_EQ(times[1], 0.2);
  EXPECT_DOUBLE_EQ(times[2], 0.3);
  EXPECT_DOUBLE_EQ(times[3], 0.4);
}

TEST(MapByTime, AppendsAndTrimsInsideTheGap) {
  MapByTime<Stamp> series;
  series.Append(0, Stamp{1.0});
  series.Append(0, Stamp{2.0});
  series.Append(0, Stamp{3.0});
  series.Append(0, Stamp{4.0});
  series.Trim(0, 1.0, 4.0);
  std::vector<double> times;
  for (auto it = series.BeginOfTrajectory(0); it != series.EndOfTrajectory(0);
       ++it) {
    times.push_back(it->time);
  }
  ASSERT_EQ(times.size(), 2u);
  EXPECT_DOUBLE_EQ(times.front(), 1.0);
  EXPECT_DOUBLE_EQ(times.back(), 4.0);
}

TEST(CompressedPointCloud, RoundTrip) {
  PointCloud cloud({RangefinderPoint{Eigen::Vector3f(1.25f, -0.5f, 0.25f)},
                    RangefinderPoint{Eigen::Vector3f(1.251f, -0.5f, 0.25f)}});
  const CompressedPointCloud compressed(cloud);
  const PointCloud restored = compressed.Decompress();
  ASSERT_EQ(restored.size(), cloud.size());
  EXPECT_NEAR(restored[0].position.x(), 1.25f, 1e-3);
  EXPECT_NEAR(restored[1].position.y(), -0.5f, 1e-3);
}

}  // namespace
}  // namespace sensor
}  // namespace atlas
