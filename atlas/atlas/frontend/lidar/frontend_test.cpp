#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "frontend/lidar/initialization/initializer.hpp"
#include "frontend/lidar/local_mapping/local_mapping.hpp"
#include "frontend/lidar/odometry/odometry.hpp"
#include "frontend/lidar/odometry/voxel_plane_map.hpp"

namespace atlas {
namespace frontend {
namespace lidar {
namespace {

sensor::ImuData Still(double time) {
  sensor::ImuData sample;
  sample.time = time;
  sample.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  return sample;
}

TEST(Initializer, AlignsUprightSpecificForce) {
  std::vector<sensor::ImuData> imu;
  for (int i = 0; i < 20; ++i) {
    imu.push_back(Still(0.01 * i));
  }
  State state;
  ASSERT_TRUE(Initializer().TryInitialize(imu, &state));
  EXPECT_NEAR(state.gravity.z(), -9.81, 1e-9);
  EXPECT_NEAR(
      state.pose.rotation().angularDistance(Eigen::Quaterniond::Identity()),
      0.0, 1e-6);
  EXPECT_DOUBLE_EQ(state.time, imu.back().time);
}

std::vector<sensor::ImuData> ConstantImu(double start, double end,
                                         const Eigen::Vector3d& acceleration) {
  std::vector<sensor::ImuData> imu;
  const double times[3] = {start, 0.5 * (start + end), end};
  for (double time : times) {
    sensor::ImuData sample;
    sample.time = time;
    sample.linear_acceleration = acceleration;
    imu.push_back(sample);
  }
  return imu;
}

TEST(Initializer, MotionRecoversGravityAndVelocity) {
  const double acceleration = 1.0;
  const double gravity = 9.81;
  std::vector<MotionFrame> frames(4);
  for (int index = 0; index < 4; ++index) {
    const double time = 0.1 * index;
    frames[static_cast<std::size_t>(index)].time = time;
    frames[static_cast<std::size_t>(index)].pose =
        transform::Rigid3d::Translation(
            Eigen::Vector3d(0.5 * acceleration * time * time, 0.0, 0.0));
    if (index + 1 < 4) {
      frames[static_cast<std::size_t>(index)].imu_until_next = ConstantImu(
          time, time + 0.1, Eigen::Vector3d(acceleration, 0.0, gravity));
    }
  }
  std::vector<State> states;
  ASSERT_TRUE(Initializer().TryInitializeMotion(frames, &states));
  ASSERT_EQ(states.size(), 4u);
  EXPECT_NEAR(states[0].gravity.z(), -gravity, 1e-6);
  EXPECT_NEAR(states[0].gravity.head<2>().norm(), 0.0, 1e-6);
  EXPECT_NEAR(states[0].velocity.norm(), 0.0, 1e-6);
  EXPECT_NEAR(states[2].velocity.x(), 0.2, 1e-6);
  EXPECT_NEAR(states[2].pose.translation().x(), 0.5 * acceleration * 0.04,
              1e-9);
}

TEST(Initializer, MotionAlignsATiltedGravity) {
  std::vector<MotionFrame> frames(3);
  for (int index = 0; index < 3; ++index) {
    frames[static_cast<std::size_t>(index)].time = 0.1 * index;
    if (index + 1 < 3) {
      frames[static_cast<std::size_t>(index)].imu_until_next =
          ConstantImu(frames[static_cast<std::size_t>(index)].time,
                      frames[static_cast<std::size_t>(index)].time + 0.1,
                      Eigen::Vector3d(0.0, -9.81, 0.0));
    }
  }
  std::vector<State> states;
  ASSERT_TRUE(Initializer().TryInitializeMotion(frames, &states));
  EXPECT_NEAR(
      states[0].gravity.normalized().dot(Eigen::Vector3d(0.0, 0.0, -1.0)), 1.0,
      1e-6);
  EXPECT_NEAR(
      states[0].pose.rotation().angularDistance(Eigen::Quaterniond::Identity()),
      M_PI / 2.0, 1e-6);
  EXPECT_NEAR(states[0].velocity.norm(), 0.0, 1e-6);
}

TEST(Initializer, RejectsMovingImu) {
  std::vector<sensor::ImuData> imu;
  for (int i = 0; i < 20; ++i) {
    sensor::ImuData sample = Still(0.01 * i);
    sample.linear_acceleration.x() = i;
    imu.push_back(sample);
  }
  State state;
  EXPECT_FALSE(Initializer().TryInitialize(imu, &state));
}

void LoosenPoseCovariance(State* state) {
  state->covariance.topLeftCorner<6, 6>().setIdentity();
  state->covariance.topLeftCorner<6, 6>() *= 1e6;
}

TEST(Odometry, PredictIntegratesConstantAcceleration) {
  State state;
  state.gravity.setZero();
  sensor::ImuData sample;
  sample.time = 1.0;
  sample.linear_acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
  const Odometry::PredictResult result = Odometry{}.Predict(state, {sample});
  ASSERT_TRUE(result.propagated);
  EXPECT_NEAR(result.state.pose.translation().x(), 0.5, 1e-9);
  EXPECT_NEAR(result.state.velocity.x(), 1.0, 1e-9);
}

TEST(Odometry, PredictCouplesVelocityIntoPositionCovariance) {
  State state;
  state.gravity.setZero();
  state.covariance.setIdentity();
  state.covariance *= 1e-4;
  sensor::ImuData sample;
  sample.time = 1.0;
  const Odometry::PredictResult result = Odometry{}.Predict(state, {sample});
  ASSERT_TRUE(result.propagated);
  EXPECT_GT(result.state.covariance(3, 3), 1.5e-4);
  EXPECT_GT(result.state.covariance(3, 6), 0.5e-4);
}

class OrthogonalPlanes : public PlaneMap {
 public:
  bool Query(const Eigen::Vector3d& point, Plane* plane) const override {
    const double distance_x = std::abs(point.x());
    const double distance_y = std::abs(point.y());
    const double distance_z = std::abs(point.z());
    double nearest = distance_x;
    plane->center.setZero();
    plane->normal = Eigen::Vector3d::UnitX();
    if (distance_y < nearest) {
      nearest = distance_y;
      plane->normal = Eigen::Vector3d::UnitY();
    }
    if (distance_z < nearest) {
      nearest = distance_z;
      plane->normal = Eigen::Vector3d::UnitZ();
    }
    return nearest < 0.3;
  }
};

class EmptyPlaneMap : public PlaneMap {
 public:
  bool Query(const Eigen::Vector3d&, Plane*) const override { return false; }
};

sensor::TimedPointCloud PlaneGrid(float offset) {
  sensor::TimedPointCloud cloud;
  cloud.reserve(60);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 4; ++j) {
      const float u = 1.f + 0.3f * static_cast<float>(i);
      const float v = 2.f + 0.25f * static_cast<float>(j);
      cloud.push_back({Eigen::Vector3f(u, v, offset), 0.f});
      cloud.push_back({Eigen::Vector3f(offset, u, v), 0.f});
      cloud.push_back({Eigen::Vector3f(u, offset, v), 0.f});
    }
  }
  return cloud;
}

sensor::TimedPointCloud OffsetCloud() { return PlaneGrid(0.05f); }

TEST(Odometry, UpdatePullsVelocityWithThePose) {
  State predicted;
  predicted.gravity.setZero();
  predicted.covariance.setIdentity();
  sensor::ImuData sample;
  sample.time = 0.1;
  predicted = Odometry{}.Predict(predicted, {sample}).state;
  Odometry::Config config;
  config.min_normal_eigenvalue = 14.0;
  const Odometry::UpdateResult result =
      Odometry(config).Update(predicted, OffsetCloud(), OrthogonalPlanes{});
  ASSERT_TRUE(result.aligned);
  EXPECT_LT(result.state.pose.translation().z(), -0.01);
  EXPECT_LT(result.state.velocity.z(), -1e-3);
}

TEST(Odometry, ScanToMapPullsOntoOrthogonalPlanes) {
  Odometry::Config config;
  config.min_normal_eigenvalue = 14.0;
  State predicted;
  LoosenPoseCovariance(&predicted);
  const Odometry::UpdateResult result =
      Odometry(config).Update(predicted, OffsetCloud(), OrthogonalPlanes{});
  ASSERT_TRUE(result.aligned);
  EXPECT_FALSE(result.diverged);
  EXPECT_EQ(result.match_count, 60);
  EXPECT_NEAR(result.state.pose.translation().x(), -0.05, 1e-6);
  EXPECT_NEAR(result.state.pose.translation().y(), -0.05, 1e-6);
  EXPECT_NEAR(result.state.pose.translation().z(), -0.05, 1e-6);
  EXPECT_NEAR(result.state.pose.rotation().angularDistance(
                  Eigen::Quaterniond::Identity()),
              0.0, 1e-6);
}

TEST(Odometry, MatchesAVoxelPlaneMap) {
  map::voxel::VoxelPlaneMap map;
  const auto insert_grid = [&map](const Eigen::Vector3d& origin,
                                  const Eigen::Vector3d& axis_u,
                                  const Eigen::Vector3d& axis_v) {
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 4; ++j) {
        map.Insert(origin + axis_u * (0.08 * i) + axis_v * (0.08 * j));
      }
    }
  };
  insert_grid(Eigen::Vector3d(0.3, 0.3, 0.5), Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitY());
  insert_grid(Eigen::Vector3d(1.5, 0.3, 0.3), Eigen::Vector3d::UnitY(),
              Eigen::Vector3d::UnitZ());
  insert_grid(Eigen::Vector3d(0.3, 2.5, 0.3), Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitZ());
  map.Commit();
  ASSERT_EQ(map.plane_count(), 3u);

  sensor::TimedPointCloud cloud;
  const auto scan_grid = [&cloud](const Eigen::Vector3f& origin,
                                  const Eigen::Vector3f& axis_u,
                                  const Eigen::Vector3f& axis_v) {
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 4; ++j) {
        const Eigen::Vector3f position =
            origin + axis_u * (0.08f * static_cast<float>(i)) +
            axis_v * (0.08f * static_cast<float>(j));
        cloud.push_back({position, 0.f});
      }
    }
  };
  scan_grid(Eigen::Vector3f(0.3f, 0.3f, 0.55f), Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitY());
  scan_grid(Eigen::Vector3f(1.55f, 0.3f, 0.3f), Eigen::Vector3f::UnitY(),
            Eigen::Vector3f::UnitZ());
  scan_grid(Eigen::Vector3f(0.3f, 2.55f, 0.3f), Eigen::Vector3f::UnitX(),
            Eigen::Vector3f::UnitZ());

  Odometry::Config config;
  config.min_normal_eigenvalue = 14.0;
  State predicted;
  LoosenPoseCovariance(&predicted);
  const Odometry::UpdateResult result =
      Odometry(config).Update(predicted, cloud, VoxelPlaneQuery(map));
  ASSERT_TRUE(result.aligned);
  EXPECT_EQ(result.match_count, 60);
  EXPECT_NEAR(result.state.pose.translation().x(), -0.05, 1e-4);
  EXPECT_NEAR(result.state.pose.translation().y(), -0.05, 1e-4);
  EXPECT_NEAR(result.state.pose.translation().z(), -0.05, 1e-4);
}

TEST(Odometry, DivergesAfterRepeatedFailedScans) {
  Odometry::Config config;
  config.degrade_bound = 2;
  Odometry odometry(config);
  const sensor::TimedPointCloud cloud(3);
  EXPECT_FALSE(odometry.Update(State{}, cloud, EmptyPlaneMap{}).diverged);
  EXPECT_FALSE(odometry.Update(State{}, cloud, EmptyPlaneMap{}).diverged);
  const Odometry::UpdateResult failed =
      odometry.Update(State{}, cloud, EmptyPlaneMap{});
  EXPECT_FALSE(failed.aligned);
  EXPECT_TRUE(failed.diverged);
  EXPECT_EQ(odometry.degrade_count(), 0);
}

sensor::TimedPointCloud CenteredFloor(float height) {
  sensor::TimedPointCloud cloud;
  for (int i = -2; i <= 2; ++i) {
    for (int j = -2; j <= 2; ++j) {
      cloud.push_back({Eigen::Vector3f(0.15f * static_cast<float>(i),
                                       0.15f * static_cast<float>(j), height),
                       0.f});
    }
  }
  return cloud;
}

TEST(LocalMapping, MarginalizesTheAnchorAsAPose3Factor) {
  LocalMapping::Config config;
  config.window_size = 2;
  config.max_iterations = 25;
  LocalMapping mapping(config);
  const OrthogonalPlanes planes;
  const sensor::TimedPointCloud on_planes = CenteredFloor(0.f);
  const sensor::TimedPointCloud offset = CenteredFloor(0.05f);

  EXPECT_TRUE(mapping.Push(10, State{}, on_planes, planes).empty());
  EXPECT_TRUE(mapping.Push(11, State{}, offset, planes).empty());
  ASSERT_EQ(mapping.scans().size(), 2u);
  const Eigen::Vector3d refined =
      mapping.scans().back().state.pose.translation();
  EXPECT_NEAR(refined.x(), 0.0, 1e-6);
  EXPECT_NEAR(refined.y(), 0.0, 1e-6);
  EXPECT_NEAR(refined.z(), -0.05, 1e-6);
  EXPECT_NEAR(mapping.scans().front().state.pose.translation().norm(), 0.0,
              1e-9);

  const std::vector<backend::RelativePoseFactor> factors =
      mapping.Push(12, State{}, on_planes, planes);
  ASSERT_EQ(factors.size(), 1u);
  EXPECT_EQ(factors[0].kind, backend::PoseFactorKind::kPose3);
  EXPECT_EQ(factors[0].from_id, 10);
  EXPECT_EQ(factors[0].to_id, 11);
  EXPECT_DOUBLE_EQ(factors[0].scale, 1.0);
  EXPECT_NEAR(factors[0].relative_pose.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(factors[0].relative_pose.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(factors[0].relative_pose.translation().z(), -0.05, 1e-6);
  EXPECT_NEAR(factors[0].relative_pose.rotation().angularDistance(
                  Eigen::Quaterniond::Identity()),
              0.0, 1e-6);
  ASSERT_EQ(mapping.scans().size(), 2u);
  EXPECT_EQ(mapping.scans().front().id, 11);
  EXPECT_EQ(mapping.scans().back().id, 12);
}

TEST(LocalMapping, ImuPreintegrationRemovesAStationaryGap) {
  LocalMapping mapping(LocalMapping::Config{2, 4});
  const OrthogonalPlanes planes;
  State first;
  first.gravity.setZero();
  State second = first;
  second.time = 0.1;
  second.pose = transform::Rigid3d::Translation(Eigen::Vector3d(0.1, 0.0, 0.0));
  EXPECT_TRUE(mapping.Push(1, first, {}, planes).empty());
  const std::vector<sensor::ImuData> imu =
      ConstantImu(0.0, 0.1, Eigen::Vector3d::Zero());
  EXPECT_TRUE(mapping.Push(2, second, {}, planes, imu).empty());
  ASSERT_EQ(mapping.scans().size(), 2u);
  EXPECT_NEAR(mapping.scans().back().state.pose.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(mapping.scans().front().state.pose.translation().norm(), 0.0,
              1e-9);
}

TEST(LocalMapping, DropsStatesOutsideTheWindow) {
  LocalMapping mapping(LocalMapping::Config{2});
  State state;
  state.time = 1.0;
  mapping.Add(state);
  state.time = 2.0;
  mapping.Add(state);
  state.time = 3.0;
  mapping.Add(state);
  ASSERT_EQ(mapping.size(), 2u);
  EXPECT_DOUBLE_EQ(mapping.window().front().time, 2.0);
  EXPECT_DOUBLE_EQ(mapping.window().back().time, 3.0);
}

}  // namespace
}  // namespace lidar
}  // namespace frontend
}  // namespace atlas
