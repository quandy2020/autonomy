#include <gtest/gtest.h>

#include "optimization/graph_solver.hpp"

namespace argus {
namespace optimization {
namespace {

TEST(EigenGraphSolver, AlignsATranslationChain) {
  FactorGraph graph;
  ASSERT_TRUE(graph.AddPose(0, transform::Rigid3d::Identity(), true));
  ASSERT_TRUE(graph.AddPose(
      1, transform::Rigid3d::Translation(Eigen::Vector3d(1.2, 0.1, 0.0)),
      false));
  ASSERT_TRUE(graph.AddPose(
      2, transform::Rigid3d::Translation(Eigen::Vector3d(2.4, -0.2, 0.0)),
      false));
  BetweenFactor first;
  first.from = 0;
  first.to = 1;
  first.relative =
      transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  BetweenFactor second = first;
  second.from = 1;
  second.to = 2;
  graph.AddBetween(first);
  graph.AddBetween(second);
  ASSERT_TRUE(EigenGraphSolver{}.Optimize(&graph));
  EXPECT_NEAR(graph.poses().at(1).pose.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(graph.poses().at(1).pose.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(graph.poses().at(2).pose.translation().x(), 2.0, 1e-6);
  EXPECT_NEAR(graph.poses().at(2).pose.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(graph.poses().at(0).pose.translation().norm(), 0.0, 1e-12);
}

TEST(EigenGraphSolver, AppliesSimilarityScaleToTranslation) {
  FactorGraph graph;
  ASSERT_TRUE(graph.AddPose(0, transform::Rigid3d::Identity(), true));
  ASSERT_TRUE(graph.AddPose(
      1, transform::Rigid3d::Translation(Eigen::Vector3d(2.4, 0.0, 0.0)),
      false));
  BetweenFactor between;
  between.from = 0;
  between.to = 1;
  between.relative =
      transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  between.similarity = true;
  between.scale = 2.0;
  graph.AddBetween(between);
  ASSERT_TRUE(EigenGraphSolver{}.Optimize(&graph));
  EXPECT_NEAR(graph.poses().at(1).pose.translation().x(), 2.0, 1e-6);
}

}  // namespace
}  // namespace optimization
}  // namespace argus
