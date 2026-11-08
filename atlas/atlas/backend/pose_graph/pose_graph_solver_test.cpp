/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/pose_graph/pose_graph_solver.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace pose_graph {
namespace {

gtsam::Pose3 ToNativePose(const transform::Rigid3d& pose) {
  const Eigen::Quaterniond& rotation = pose.rotation();
  return gtsam::Pose3(
      gtsam::Rot3(rotation.w(), rotation.x(), rotation.y(), rotation.z()),
      gtsam::Point3(pose.translation()));
}

transform::Rigid3d MakePose(double x, double y, double yaw) {
  return transform::Rigid3d(
      Eigen::Vector3d(x, y, 0.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
}

gtsam::SharedNoiseModel MakeNoiseModel(
    const std::array<double, 6>& diagonal_information) {
  gtsam::Vector6 standard_deviations;
  for (int axis = 0; axis < 6; ++axis) {
    standard_deviations[axis] =
        1.0 / std::sqrt(diagonal_information[static_cast<std::size_t>(axis)]);
  }
  return gtsam::noiseModel::Diagonal::Sigmas(standard_deviations);
}

FactorGraph MakeFixedGraph() {
  FactorGraph factor_graph;
  const transform::Rigid3d first_pose = MakePose(0.0, 0.0, 0.0);
  const transform::Rigid3d second_pose = MakePose(1.2, 0.05, 0.08);
  const transform::Rigid3d third_pose = MakePose(2.05, 0.62, 0.17);
  EXPECT_TRUE(factor_graph.AddPose(0, first_pose, true));
  EXPECT_TRUE(factor_graph.AddPose(1, second_pose, false));
  EXPECT_TRUE(factor_graph.AddPose(2, third_pose, false));

  BetweenFactor first_odometry;
  first_odometry.from_id = 0;
  first_odometry.to_id = 1;
  first_odometry.relative_pose = transform::Rigid3d(
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())));
  first_odometry.diagonal_information = {
      {100.0, 100.0, 100.0, 25.0, 25.0, 25.0}};
  first_odometry.diagonal_information_provided = true;
  factor_graph.AddBetweenFactor(first_odometry);

  BetweenFactor second_odometry;
  second_odometry.from_id = 1;
  second_odometry.to_id = 2;
  second_odometry.relative_pose = transform::Rigid3d(
      Eigen::Vector3d(0.9, 0.2, 0.05),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ())));
  second_odometry.diagonal_information = {{80.0, 80.0, 80.0, 20.0, 20.0, 20.0}};
  second_odometry.diagonal_information_provided = true;
  factor_graph.AddBetweenFactor(second_odometry);

  BetweenFactor loop_closure;
  loop_closure.from_id = 0;
  loop_closure.to_id = 2;
  loop_closure.relative_pose = transform::Rigid3d(
      Eigen::Vector3d(1.85, 0.35, 0.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitZ())));
  loop_closure.diagonal_information = {{60.0, 60.0, 60.0, 30.0, 30.0, 30.0}};
  loop_closure.diagonal_information_provided = true;
  factor_graph.AddBetweenFactor(loop_closure);

  return factor_graph;
}

void AddNativeFactor(const BetweenFactor& factor,
                     gtsam::NonlinearFactorGraph* factors,
                     gtsam::NonlinearFactorGraph* measurement_factors) {
  const gtsam::BetweenFactor<gtsam::Pose3> native_factor(
      static_cast<gtsam::Key>(factor.from_id),
      static_cast<gtsam::Key>(factor.to_id), ToNativePose(factor.relative_pose),
      MakeNoiseModel(factor.diagonal_information));
  factors->add(native_factor);
  measurement_factors->add(native_factor);
}

std::pair<gtsam::Values, double> SolveNativeGraph() {
  const FactorGraph factor_graph = MakeFixedGraph();
  gtsam::NonlinearFactorGraph native_factors;
  gtsam::NonlinearFactorGraph measurement_factors;
  gtsam::Values initial_values;
  for (const auto& [pose_id, node] : factor_graph.GetPoses()) {
    initial_values.insert(static_cast<gtsam::Key>(pose_id),
                          ToNativePose(node.pose));
  }
  const transform::Rigid3d first_pose = factor_graph.GetPoses().at(0).pose;
  gtsam::Vector6 fixed_pose_standard_deviations =
      gtsam::Vector6::Constant(1.0 / std::sqrt(1e9));
  native_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      0, ToNativePose(first_pose),
      gtsam::noiseModel::Diagonal::Sigmas(fixed_pose_standard_deviations)));
  for (const BetweenFactor& factor : factor_graph.GetBetweenFactors()) {
    AddNativeFactor(factor, &native_factors, &measurement_factors);
  }
  gtsam::ISAM2Params solver_parameters;
  solver_parameters.relinearizeThreshold = 1e-8;
  solver_parameters.relinearizeSkip = 1;
  gtsam::ISAM2 native_solver(solver_parameters);
  native_solver.update(native_factors, initial_values);
  for (int update_index = 1; update_index < 6; ++update_index) {
    native_solver.update();
  }
  const gtsam::Values native_estimate = native_solver.calculateEstimate();
  return {native_estimate, measurement_factors.error(native_estimate)};
}

TEST(PoseGraphSolver, MatchesNativeIncrementalSolverOnFixedGraph) {
  const FactorGraph factor_graph = MakeFixedGraph();
  const auto native_result = SolveNativeGraph();

  SolverOptions solver_options;
  solver_options.maximum_update_count = 6;
  solver_options.convergence_tolerance = 1e-8;
  PoseGraphSolver solver;
  const UpdateResult update = solver.Update(factor_graph, solver_options);
  ASSERT_TRUE(update.Succeeded()) << update.diagnostic_message;
  const SolverResult result = solver.CalculateEstimate();

  ASSERT_TRUE(result.Succeeded()) << result.diagnostic_message;
  ASSERT_GE(update.update_count, 1);
  ASSERT_LE(update.update_count, 6);
  ASSERT_EQ(result.optimized_poses.size(), 3U);
  EXPECT_NEAR(result.final_cost, native_result.second, 1e-10);
  for (std::int64_t pose_id = 0; pose_id < 3; ++pose_id) {
    const gtsam::Pose3& native_pose =
        native_result.first.at<gtsam::Pose3>(pose_id);
    const transform::Rigid3d& optimized_pose =
        result.optimized_poses.at(pose_id);
    EXPECT_LT((optimized_pose.translation() -
               Eigen::Vector3d(native_pose.translation()))
                  .norm(),
              1e-10);
    EXPECT_LT((optimized_pose.rotation().toRotationMatrix() -
               native_pose.rotation().matrix())
                  .norm(),
              1e-10);
  }
}

TEST(PoseGraphSolver, UpdatesWithFactorsReferencingExistingPoses) {
  PoseGraphSolver solver;
  SolverOptions options;
  options.maximum_update_count = 6;

  FactorGraph first_batch;
  ASSERT_TRUE(first_batch.AddPose(0, MakePose(0.0, 0.0, 0.0), true));
  ASSERT_TRUE(first_batch.AddPose(1, MakePose(1.1, 0.0, 0.0), false));
  BetweenFactor first_edge;
  first_edge.from_id = 0;
  first_edge.to_id = 1;
  first_edge.relative_pose = MakePose(1.0, 0.0, 0.0);
  first_batch.AddBetweenFactor(first_edge);
  const UpdateResult first_update = solver.Update(first_batch, options);
  ASSERT_TRUE(first_update.Succeeded()) << first_update.diagnostic_message;

  FactorGraph second_batch;
  ASSERT_TRUE(second_batch.AddPose(2, MakePose(2.2, 0.0, 0.0), false));
  BetweenFactor second_edge;
  second_edge.from_id = 1;
  second_edge.to_id = 2;
  second_edge.relative_pose = MakePose(1.0, 0.0, 0.0);
  second_batch.AddBetweenFactor(second_edge);
  const UpdateResult second_update = solver.Update(second_batch, options);
  ASSERT_TRUE(second_update.Succeeded()) << second_update.diagnostic_message;
  const SolverResult second_result = solver.CalculateEstimate();

  ASSERT_TRUE(second_result.Succeeded()) << second_result.diagnostic_message;
  EXPECT_EQ(second_result.optimized_poses.size(), 3U);
  EXPECT_LT((second_result.optimized_poses.at(2).translation() -
             Eigen::Vector3d(2.0, 0.0, 0.0))
                .norm(),
            1e-6);
  EXPECT_TRUE(std::isfinite(second_result.final_cost));

  FactorGraph duplicate_batch;
  ASSERT_TRUE(duplicate_batch.AddPose(1, MakePose(1.0, 0.0, 0.0), false));
  EXPECT_EQ(solver.Update(duplicate_batch, options).status,
            SolverStatus::kInvalidInput);

  FactorGraph loop_batch;
  BetweenFactor loop_edge;
  loop_edge.from_id = 0;
  loop_edge.to_id = 2;
  loop_edge.relative_pose = MakePose(2.0, 0.0, 0.0);
  loop_batch.AddBetweenFactor(loop_edge);
  const UpdateResult loop_update = solver.Update(loop_batch, options);
  ASSERT_TRUE(loop_update.Succeeded()) << loop_update.diagnostic_message;
  const SolverResult loop_result = solver.CalculateEstimate();
  ASSERT_TRUE(loop_result.Succeeded()) << loop_result.diagnostic_message;
  EXPECT_EQ(loop_result.optimized_poses.size(), 3U);
  EXPECT_LT((loop_result.optimized_poses.at(2).translation() -
             Eigen::Vector3d(2.0, 0.0, 0.0))
                .norm(),
            1e-6);
}

}  // namespace
}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas
