#include "autonomy/localization/cartographer/node/occupancy_grid_builder.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid_cells.hpp>
#include "autonomy/localization/cartographer/mapping/2d/probability_grid.hpp"
#include "gtest/gtest.h"

namespace autonomy::localization::cartographer::node {
namespace {

TEST(OccupancyGridBuilderTest, PreservesThinWallsProbabilitiesAndUnknownSpace) {
    constexpr double kResolution = 0.05;
    ::cartographer::mapping::ValueConversionTables tables;
    ::cartographer::mapping::ProbabilityGrid probabilities(
        ::cartographer::mapping::MapLimits(
            kResolution, Eigen::Vector2d::Zero(),
            ::cartographer::mapping::CellLimits(9, 9)), &tables);
    for (int y = 1; y <= 7; ++y) {
        for (int x = 1; x <= 7; ++x) {
            probabilities.SetProbability({x, y},
                x == 4 && y == 4 ? 0.9f : 0.1f);
        }
    }
    ::cartographer::mapping::proto::SubmapQuery::Response::SubmapTexture texture;
    ASSERT_TRUE(probabilities.DrawToSubmapTexture(
        &texture, ::cartographer::transform::Rigid3d::Identity()));
    const auto pixels = ::cartographer::io::UnpackTextureData(
        texture.cells(), texture.width(), texture.height());
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> slices;
    auto& slice = slices[{0, 0}];
    slice.width = texture.width();
    slice.height = texture.height();
    slice.resolution = texture.resolution();
    slice.pose = ::cartographer::transform::Rigid3d::Identity();
    slice.slice_pose = ::cartographer::transform::ToRigid3(texture.slice_pose());
    slice.surface = ::cartographer::io::DrawTexture(
        pixels.intensity, pixels.alpha, slice.width, slice.height, &slice.cairo_data);

    automsgs::msgs::builtin_interfaces::Time stamp;
    stamp.set_sec(123);
    const auto grid = BuildOccupancyGrid(slices, kResolution, "map", stamp);
    ASSERT_NE(grid, nullptr);
    EXPECT_EQ(grid->header().frame_id(), "map");
    EXPECT_EQ(grid->header().stamp().sec(), 123);
    EXPECT_FLOAT_EQ(grid->info().resolution(), kResolution);
    int known = 0;
    int occupied = 0;
    int black = 0;
    for (int i = 0; i < automsgs::msgs::map_msgs::OccupancyGridCellCount(*grid); ++i) {
        const int value = automsgs::msgs::map_msgs::OccupancyGridCell(*grid, i);
        known += value >= 0;
        occupied += value > 50;
        black += value == 100;
    }
    // Seven by seven observed cells, with one wall and no invented outline.
    EXPECT_EQ(known, 49);
    EXPECT_EQ(occupied, 1);
    EXPECT_EQ(black, 0);
}

TEST(OccupancyGridBuilderTest, EmptySubmapsDoNotPublishAMap) {
    EXPECT_EQ(BuildOccupancyGrid({}, 0.05, "map", {}), nullptr);
}

}  // namespace
}  // namespace autonomy::localization::cartographer::node
