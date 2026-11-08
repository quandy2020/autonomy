#include "map/sliding_voxel_tree.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace atlas {
namespace map {
namespace {

TEST(SlidingVoxelTree, ReleasesNodeLockWhenMarginalizationFactorIsUnavailable) {
  auto options = std::make_shared<SlidingVoxelMapOptions>();
  options->maximum_layer = 0;
  options->minimum_point_count_by_layer = {5.0};
  options->plane_eigenvalue_thresholds = {0.25};
  options->sliding_window_order = {0};
  SlidingVoxelTree tree(0, 1, options);
  tree.sw = std::make_unique<FrameWindow>(1);
  tree.isexist = true;
  tree.opt_state = 0;

  PointCloudFactorData empty_factor_data(1);
  std::vector<PoseState> pose_window(1);
  tree.Marginalize(1, 1, pose_window, empty_factor_data);

  EXPECT_EQ(tree.opt_state, -1);
  ASSERT_TRUE(tree.mVox.try_lock());
  tree.mVox.unlock();
}

TEST(SlidingVoxelMapOptions, AdvancesFrameSlotsAfterMarginalization) {
  SlidingVoxelMapOptions options;
  options.sliding_window_order = {0, 1, 2, 3, 4};

  options.AdvanceWindowOrder(1);

  EXPECT_EQ(options.sliding_window_order,
            (std::vector<int>{1, 2, 3, 4, 0}));
}

}  // namespace
}  // namespace map
}  // namespace atlas
