// Copyright 2026 The Openbot Authors
#include "autonomy/localization/cartographer/node/map_builder_bridge.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"
#include "autonomy/localization/cartographer/mapping/internal/testing/mock_map_builder.hpp"
#include "autonomy/localization/cartographer/mapping/internal/testing/mock_pose_graph.hpp"

namespace autonomy::localization::cartographer::node {
namespace {
namespace mapping = ::cartographer::mapping;
using Rigid = ::cartographer::transform::Rigid3d;
using ::testing::Invoke;
using ::testing::Return;

class VisualizationTest : public ::testing::Test {
 protected:
    VisualizationTest() {
        auto builder = std::make_unique<::testing::NiceMock<mapping::testing::MockMapBuilder>>();
        builder_ = builder.get();
        ON_CALL(*builder_, pose_graph()).WillByDefault(Return(&graph_));
        ON_CALL(graph_, GetAllSubmapPoses()).WillByDefault(Invoke([this] { return submaps_; }));
        ON_CALL(graph_, GetTrajectoryNodePoses()).WillByDefault(Invoke([this] { return nodes_; }));
        ON_CALL(graph_, constraints()).WillByDefault(Invoke([this] { return constraints_; }));
        NodeOptions options{}; options.map_frame = "map";
        bridge_ = std::make_unique<MapBuilderBridge>(options, std::move(builder), nullptr);
    }
    ::testing::NiceMock<mapping::testing::MockPoseGraph> graph_;
    mapping::testing::MockMapBuilder* builder_;
    mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapPose> submaps_;
    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> nodes_;
    std::vector<mapping::PoseGraphInterface::Constraint> constraints_;
    std::unique_ptr<MapBuilderBridge> bridge_;
};

TEST_F(VisualizationTest, ReplaysAllTexturesAndEncodesOnlyChangedVersions) {
    for (int i = 0; i < 20; ++i) submaps_.Insert({0, i}, {1, Rigid::Identity()});
    int encodes = 0;
    ON_CALL(*builder_, SubmapToProto(::testing::_, ::testing::_)).WillByDefault(Invoke(
        [&](const mapping::SubmapId& id, mapping::proto::SubmapQuery::Response* response) {
            ++encodes;
            response->set_submap_version(submaps_.at(id).version);
            auto* texture = response->add_textures();
            texture->set_width(1); texture->set_height(1); texture->set_resolution(.05);
            texture->mutable_slice_pose()->mutable_rotation()->set_w(1);
            texture->set_cells("cached texture");
            return std::string();
        }));
    bridge_->GetSubmapList({});
    EXPECT_EQ(encodes, 0);
    std::set<int> received;
    for (int n = 0; n < 20; ++n) {
        const auto list = bridge_->GetSubmapList({}, true);
        int attached = 0;
        for (const auto& entry : list.submap()) if (!entry.textures().empty()) {
            received.insert(entry.submap_index()); ++attached;
        }
        EXPECT_LE(attached, 6);
    }
    EXPECT_EQ(received.size(), 20);
    EXPECT_EQ(encodes, 20);
    // A later subscriber receives old textures despite no new versions.
    received.clear();
    for (int n = 0; n < 20; ++n) {
        const auto list = bridge_->GetSubmapList({}, true);
        for (const auto& entry : list.submap())
            if (!entry.textures().empty()) received.insert(entry.submap_index());
    }
    EXPECT_EQ(received.size(), 20);
    EXPECT_EQ(encodes, 20);
    submaps_.at({0, 5}).version = 2;
    submaps_.at({0, 6}).pose = Rigid::Translation({1, 2, 3});
    for (int n = 0; n < 20; ++n) bridge_->GetSubmapList({}, true);
    EXPECT_EQ(encodes, 21);
    submaps_.Trim({0, 5});
    for (int n = 0; n < 20; ++n) {
        const auto list = bridge_->GetSubmapList({}, true);
        for (const auto& entry : list.submap()) EXPECT_NE(entry.submap_index(), 5);
    }
}

TEST_F(VisualizationTest, FullTrajectoryRetainsOldNodesAndLoopClosureCorrections) {
    automsgs::msgs::builtin_interfaces::Time stamp;
    stamp.set_sec(100);
    constexpr int kNodes = 20005;
    for (int i = 0; i < kNodes; ++i) {
        nodes_.Insert({0, i}, {Rigid::Translation({i * .1, 0, 0}),
            mapping::TrajectoryNodePose::ConstantPoseData{
                FromCommsgs(stamp), Rigid::Identity()}});
    }
    auto path = bridge_->GetTrajectoryPath(stamp);
    ASSERT_EQ(path.poses_size(), kNodes);
    EXPECT_EQ(path.header().frame_id(), "map");
    EXPECT_EQ(path.header().stamp().sec(), 100);
    for (int i = 0; i < kNodes; ++i) {
        EXPECT_FLOAT_EQ(path.poses(i).pose().position().x(), i * .1);
        EXPECT_EQ(path.poses(i).header().frame_id(), "map");
        EXPECT_EQ(path.poses(i).header().stamp().sec(), 100);
    }
    nodes_.at({0, 0}).global_pose = Rigid::Translation({-5, 2, 0});
    nodes_.at({0, 10000}).global_pose = Rigid::Translation({999, 3, 0});
    nodes_.Insert({0, kNodes}, {Rigid::Translation({2001, 4, 0}), {}});
    stamp.set_sec(101);
    path = bridge_->GetTrajectoryPath(stamp);
    ASSERT_EQ(path.poses_size(), kNodes + 1);
    EXPECT_DOUBLE_EQ(path.poses(0).pose().position().x(), -5);
    EXPECT_DOUBLE_EQ(path.poses(0).pose().position().y(), 2);
    EXPECT_DOUBLE_EQ(path.poses(10000).pose().position().y(), 3);
    EXPECT_DOUBLE_EQ(path.poses(kNodes).pose().position().x(), 2001);
    EXPECT_EQ(path.header().stamp().sec(), 101);
}

TEST_F(VisualizationTest, EmptyTrajectoryHasNoStaleHistory) {
    EXPECT_EQ(bridge_->GetTrajectoryPath({}).poses_size(), 0);
    nodes_.Insert({0, 0}, {Rigid::Identity(), {}});
    EXPECT_EQ(bridge_->GetTrajectoryPath({}).poses_size(), 1);
    nodes_.Trim({0, 0});
    EXPECT_EQ(bridge_->GetTrajectoryPath({}).poses_size(), 0);
}

TEST_F(VisualizationTest, LargeChangingTextureDoesNotStarveTheReplay) {
    for (int i=0;i<6;++i) submaps_.Insert({0,i},{1,Rigid::Identity()});
    ON_CALL(*builder_, SubmapToProto(::testing::_, ::testing::_)).WillByDefault(Invoke(
        [&](const mapping::SubmapId& id, mapping::proto::SubmapQuery::Response* response) {
            response->set_submap_version(submaps_.at(id).version);
            auto* texture=response->add_textures();
            texture->set_width(1); texture->set_height(1); texture->set_resolution(.05);
            texture->mutable_slice_pose()->mutable_rotation()->set_w(1);
            texture->set_cells(std::string(id.submap_index==3 ? 768*1024 : 1024, 'x'));
            return std::string();
        }));
    // Populate the cache first, then join late while the newest submap changes.
    for (int n=0;n<10;++n) bridge_->GetSubmapList({},true);
    std::set<int> received;
    for (int n=0;n<20;++n) {
        ++submaps_.at({0,5}).version;
        const auto list=bridge_->GetSubmapList({},true);
        for (const auto& entry:list.submap()) if (!entry.textures().empty()) received.insert(entry.submap_index());
        EXPECT_LT(list.ByteSizeLong(),1024*1024);
    }
    EXPECT_EQ(received.size(),6);
}

TEST_F(VisualizationTest, ConstraintTypesColorsAndResidualEndpointsMatchRviz) {
    submaps_.Insert({0, 0}, {1, Rigid::Translation({10, 0, 0})});
    nodes_.Insert({0, 0}, {Rigid::Translation({13, 0, 0}), {}});
    nodes_.Insert({1, 0}, {Rigid::Translation({14, 0, 0}), {}});
    using Constraint = mapping::PoseGraphInterface::Constraint;
    constraints_.push_back({{0, 0}, {0, 0}, {Rigid::Translation({2, 0, 0}), 1, 1}, Constraint::INTRA_SUBMAP});
    constraints_.push_back({{0, 0}, {0, 0}, {Rigid::Translation({2, 0, 0}), 1, 1}, Constraint::INTER_SUBMAP});
    constraints_.push_back({{0, 0}, {1, 0}, {Rigid::Translation({2, 0, 0}), 1, 1}, Constraint::INTER_SUBMAP});
    constraints_.push_back({{9, 9}, {0, 0}, {Rigid::Identity(), 1, 1}, Constraint::INTER_SUBMAP});
    const auto markers = bridge_->GetConstraintList();
    ASSERT_EQ(markers.markers_size(), 6);
    for (const auto& marker : markers.markers()) {
        EXPECT_EQ(marker.points_size(), 2); EXPECT_EQ(marker.colors_size(), 2);
        EXPECT_EQ(marker.header().frame_id(), "map"); EXPECT_DOUBLE_EQ(marker.scale().x(), .025);
    }
    EXPECT_DOUBLE_EQ(markers.markers(0).points(0).x(), 10);
    EXPECT_DOUBLE_EQ(markers.markers(0).points(1).x(), 12);
    EXPECT_DOUBLE_EQ(markers.markers(1).points(0).x(), 12);
    EXPECT_DOUBLE_EQ(markers.markers(1).points(1).x(), 13);
    EXPECT_FLOAT_EQ(markers.markers(2).colors(0).r(), 1);
    EXPECT_FLOAT_EQ(markers.markers(2).colors(0).g(), 1);
    EXPECT_FLOAT_EQ(markers.markers(4).colors(0).g(), 165.f / 255);
    EXPECT_FLOAT_EQ(markers.markers(5).colors(0).g(), 1);
    EXPECT_FLOAT_EQ(markers.markers(5).colors(0).b(), 1);
    EXPECT_DOUBLE_EQ(markers.markers(5).points(1).x(), 14);
}
}  // namespace
}  // namespace autonomy::localization::cartographer::node
