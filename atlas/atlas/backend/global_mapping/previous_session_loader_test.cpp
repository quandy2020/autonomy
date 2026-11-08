/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/previous_session_loader.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

TEST(PreviousSessionLoader, KeepsExistingSessionsAndEdgesWhenLoadFails) {
  PreviousSessionOptions options;
  options.base_path = "/tmp/atlas_missing_previous_sessions";
  options.session_list = "../outside:0.5";

  std::vector<PreviousSession> sessions;
  sessions.emplace_back();
  sessions.back().name = "existing_session";

  PgoEdges edges;
  Eigen::Matrix<double, 6, 1> information;
  information.setConstant(1e-6);
  edges.Push(0, 1, 2, 3, Eigen::Matrix3d::Identity(), Eigen::Vector3d::UnitX(),
             information);

  EXPECT_FALSE(PreviousSessionLoader().Load(options, &sessions, &edges));
  ASSERT_EQ(sessions.size(), 1u);
  EXPECT_EQ(sessions[0].name, "existing_session");
  ASSERT_EQ(edges.edges.size(), 1u);
  EXPECT_EQ(edges.edges[0].ids1[0], 2);
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
