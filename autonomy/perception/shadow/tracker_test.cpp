/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file tracker_test.cpp
 * @brief Contract tests for stable Shadow selected-person tracking.
 */

#include "autonomy/perception/shadow/tracker.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;

proto::ShadowOptions ValidOptions() {
    proto::ShadowOptions options;
    options.set_track_high_threshold(0.50F);
    options.set_track_low_threshold(0.10F);
    options.set_association_iou_threshold(0.30F);
    options.set_min_confirmed_hits(2);
    options.set_prediction_timeout_sec(0.35F);
    options.set_lost_timeout_sec(1.5F);
    return options;
}

Detection2D Box(double center_x, double center_y, double width, double height,
                double score) {
    Detection2D detection;
    auto* hypothesis = detection.add_results()->mutable_hypothesis();
    hypothesis->set_class_id("person");
    hypothesis->set_score(score);
    auto* bbox = detection.mutable_bbox();
    bbox->mutable_center()->mutable_position()->set_x(center_x);
    bbox->mutable_center()->mutable_position()->set_y(center_y);
    bbox->set_size_x(width);
    bbox->set_size_y(height);
    return detection;
}

Detection2DArray Detections(std::initializer_list<Detection2D> boxes) {
    Detection2DArray detections;
    for (const auto& box : boxes) {
        *detections.add_detections() = box;
    }
    return detections;
}

TEST(PersonTrackerTest, ConfirmsTrackOnlyAfterMinimumHighConfidenceHits) {
    PersonTracker tracker(ValidOptions());
    auto first = Detections({Box(20, 30, 40, 80, 0.9)});

    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    ASSERT_EQ(first.detections_size(), 1);
    EXPECT_EQ(first.detections(0).id(), "1");
    Detection2DArray confirmed;
    tracker.Confirmed(&confirmed);
    EXPECT_EQ(confirmed.detections_size(), 0);

    auto second = Detections({Box(22, 30, 40, 80, 0.9)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &second));
    EXPECT_EQ(second.detections(0).id(), "1");
    tracker.Confirmed(&confirmed);
    ASSERT_EQ(confirmed.detections_size(), 1);
    EXPECT_EQ(confirmed.detections(0).id(), "1");
}

TEST(PersonTrackerTest, AssignsMonotonicStringIdsWithoutReusingLostIds) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first =
        Detections({Box(10, 10, 20, 40, 0.9), Box(100, 10, 20, 40, 0.9)});

    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    EXPECT_EQ(first.detections(0).id(), "1");
    EXPECT_EQ(first.detections(1).id(), "2");

    auto empty = Detections({});
    ASSERT_TRUE(tracker.Update(2'600'000'001, &empty));
    auto replacement = Detections({Box(200, 10, 20, 40, 0.9)});
    ASSERT_TRUE(tracker.Update(2'700'000'000, &replacement));
    EXPECT_EQ(replacement.detections(0).id(), "3");
}

TEST(PersonTrackerTest, AssociatesHighConfidenceDetectionWithStableId) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));

    auto moved = Detections({Box(55, 40, 30, 60, 0.8)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &moved));
    EXPECT_EQ(moved.detections(0).id(), first.detections(0).id());
}

TEST(PersonTrackerTest, RecoversConfirmedTrackWithLowConfidenceDetection) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));

    auto weak = Detections({Box(53, 40, 30, 60, 0.2)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &weak));
    EXPECT_EQ(weak.detections(0).id(), first.detections(0).id());

    tracker.Select(first.detections(0).id());
    Detection2D selected;
    EXPECT_TRUE(tracker.Selected(&selected));
    EXPECT_TRUE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
}

TEST(PersonTrackerTest, DoesNotCreateTrackFromLowConfidenceDetection) {
    PersonTracker tracker(ValidOptions());
    auto weak = Detections({Box(50, 40, 30, 60, 0.2)});

    ASSERT_TRUE(tracker.Update(1'000'000'000, &weak));
    EXPECT_TRUE(weak.detections(0).id().empty());
    Detection2DArray confirmed;
    tracker.Confirmed(&confirmed);
    EXPECT_EQ(confirmed.detections_size(), 0);
}

TEST(PersonTrackerTest, DoesNotSwitchSelectedPersonAtCrossing) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first =
        Detections({Box(10, 10, 40, 80, 0.9), Box(100, 10, 40, 80, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    tracker.Select(first.detections(0).id());

    auto crossed =
        Detections({Box(95, 10, 40, 80, 0.9), Box(15, 10, 40, 80, 0.9)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &crossed));
    Detection2D selected;
    ASSERT_TRUE(tracker.Selected(&selected));
    EXPECT_EQ(selected.id(), first.detections(0).id());
    EXPECT_DOUBLE_EQ(selected.bbox().center().position().x(), 15.0);
}

TEST(PersonTrackerTest, ExposesPredictionOnlyWithinPredictionTimeout) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    tracker.Select(first.detections(0).id());

    auto empty = Detections({});
    ASSERT_TRUE(tracker.Update(1'300'000'000, &empty));
    Detection2D selected;
    EXPECT_TRUE(tracker.Selected(&selected));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_TRUE(tracker.selected_predicted());
    EXPECT_EQ(selected.id(), first.detections(0).id());

    ASSERT_TRUE(tracker.Update(1'400'000'000, &empty));
    EXPECT_FALSE(tracker.Selected(&selected));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
}

TEST(PersonTrackerTest, LostSelectedIdIsNeverSilentlyReplaced) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    const std::string selected_id = first.detections(0).id();
    tracker.Select(selected_id);

    auto empty = Detections({});
    ASSERT_TRUE(tracker.Update(2'500'000'001, &empty));
    Detection2D selected;
    EXPECT_FALSE(tracker.Selected(&selected));

    auto newcomer = Detections({Box(150, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(2'600'000'000, &newcomer));
    EXPECT_NE(newcomer.detections(0).id(), selected_id);
    EXPECT_FALSE(tracker.Selected(&selected));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
}

TEST(PersonTrackerTest, ExplicitUnknownIdLocksOnlyThatId) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));

    tracker.Select("999");
    Detection2D selected;
    EXPECT_FALSE(tracker.Selected(&selected));
    auto next = Detections({Box(52, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &next));
    EXPECT_FALSE(tracker.Selected(&selected));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
}

TEST(PersonTrackerTest, EmptySelectionClearsLockForComponentLevelSelection) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    tracker.Select(first.detections(0).id());
    ASSERT_TRUE(tracker.Selected(nullptr));

    tracker.Select("");
    EXPECT_FALSE(tracker.Selected(nullptr));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
    Detection2DArray candidates;
    tracker.Confirmed(&candidates);
    ASSERT_EQ(candidates.detections_size(), 1);
    EXPECT_EQ(candidates.detections(0).id(), first.detections(0).id());
}

TEST(PersonTrackerTest, EnumeratesOnlyConfirmedVisibleTracksInStableIdOrder) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first =
        Detections({Box(10, 10, 20, 40, 0.9), Box(100, 10, 20, 40, 0.9),
                    Box(200, 10, 20, 40, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));

    auto visible =
        Detections({Box(202, 10, 20, 40, 0.9), Box(12, 10, 20, 40, 0.9)});
    ASSERT_TRUE(tracker.Update(1'100'000'000, &visible));
    Detection2DArray confirmed;
    tracker.Confirmed(&confirmed);
    ASSERT_EQ(confirmed.detections_size(), 2);
    EXPECT_EQ(confirmed.detections(0).id(), "1");
    EXPECT_EQ(confirmed.detections(1).id(), "3");
}

TEST(PersonTrackerTest, ClearDropsTracksAndSelectionWithoutReusingIds) {
    auto options = ValidOptions();
    options.set_min_confirmed_hits(1);
    PersonTracker tracker(options);
    auto first = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(1'000'000'000, &first));
    tracker.Select(first.detections(0).id());

    tracker.Clear();
    EXPECT_FALSE(tracker.Selected(nullptr));
    EXPECT_FALSE(tracker.selected_visible());
    EXPECT_FALSE(tracker.selected_predicted());
    Detection2DArray confirmed;
    tracker.Confirmed(&confirmed);
    EXPECT_EQ(confirmed.detections_size(), 0);

    auto next = Detections({Box(50, 40, 30, 60, 0.9)});
    ASSERT_TRUE(tracker.Update(2'000'000'000, &next));
    EXPECT_EQ(next.detections(0).id(), "2");
}

TEST(PersonTrackerTest, ReportsNullDetectionInputWithoutChangingState) {
    PersonTracker tracker(ValidOptions());
    std::string error;

    EXPECT_FALSE(tracker.Update(1'000'000'000, nullptr, &error));
    EXPECT_EQ(error, "Shadow tracker: detections are null.");
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
