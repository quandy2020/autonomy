/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/frame_align.hpp"

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

TEST(FrameAlignTest, EmptyFrameFailsWhenRequired) {
    ObservationSnapshot snapshot;
    auto scan = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
    scan->mutable_header()->set_frame_id("");
    snapshot.laser_scan = scan;

    ObservationSnapshot aligned;
    std::string error;
    EXPECT_FALSE(AlignObservationFrames(snapshot, nullptr, "base_link", 0.1,
                                        /*require_frame_match=*/true, &aligned,
                                        &error));
    EXPECT_NE(error.find("empty"), std::string::npos);
}

TEST(FrameAlignTest, MismatchedFrameFailsWhenRequired) {
    ObservationSnapshot snapshot;
    auto scan = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
    scan->mutable_header()->set_frame_id("laser");
    snapshot.laser_scan = scan;

    ObservationSnapshot aligned;
    std::string error;
    EXPECT_FALSE(AlignObservationFrames(snapshot, nullptr, "base_link", 0.1,
                                        /*require_frame_match=*/true, &aligned,
                                        &error));
    EXPECT_NE(error.find("must be"), std::string::npos);
}

TEST(FrameAlignTest, MatchingFramePasses) {
    ObservationSnapshot snapshot;
    auto scan = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
    scan->mutable_header()->set_frame_id("base_link");
    snapshot.laser_scan = scan;

    ObservationSnapshot aligned;
    std::string error;
    EXPECT_TRUE(AlignObservationFrames(snapshot, nullptr, "base_link", 0.1,
                                       /*require_frame_match=*/true, &aligned,
                                       &error));
}

TEST(FrameAlignTest, EmptyFrameOkWhenNotRequired) {
    ObservationSnapshot snapshot;
    auto scan = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
    snapshot.laser_scan = scan;

    ObservationSnapshot aligned;
    std::string error;
    EXPECT_TRUE(AlignObservationFrames(snapshot, nullptr, "base_link", 0.1,
                                       /*require_frame_match=*/false, &aligned,
                                       &error));
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
