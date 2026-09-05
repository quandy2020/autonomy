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
 * @file options_test.cpp
 * @brief Contract tests for the single Hestia protobuf configuration model.
 */

#include "autonomy/perception/hestia/options.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

namespace autonomy::perception::hestia {
namespace {

proto::HestiaOptions ValidOptions() {
    proto::HestiaOptions o;
    o.set_mode("open");
    o.set_open_model_path("/models/hestia_open.onnx");
    o.set_backend("onnx");
    o.set_open_width(640);
    o.set_open_height(640);
    o.set_max_detections(100);
    o.set_confidence_threshold(0.25F);
    o.add_open_prompts("chair");
    o.add_open_prompts("cup");
    o.set_depth_scale(0.001F);
    o.set_min_depth_m(0.2F);
    o.set_max_depth_m(8.0F);
    o.set_min_depth_samples(12);
    o.set_inner_box_scale(0.5F);
    o.set_depth_outlier_m(0.25F);
    o.set_camera_frame("camera_optical");
    o.set_base_frame("base_link");
    o.set_association_iou_threshold(0.3F);
    o.set_lost_timeout_sec(1.5F);
    o.set_merge_iou_threshold(0.5F);
    o.set_detections_2d_topic("/perception/hestia/detections_2d");
    o.set_detections_3d_topic("/perception/hestia/detections_3d");
    o.set_max_input_skew_sec(0.05F);
    o.set_max_data_age_sec(0.2F);
    o.set_use_fathom_depth(false);
    return o;
}

proto::HestiaOptions ValidDualOptions() {
    auto o = ValidOptions();
    o.set_mode("dual");
    o.set_home_model_path("/models/hestia_home.onnx");
    o.set_home_width(640);
    o.set_home_height(640);
    o.add_home_labels("person");
    o.add_home_labels("chair");
    return o;
}

TEST(HestiaOptionsTest, AcceptsValidOpenMode) {
    std::string error = "stale";
    EXPECT_TRUE(ValidateHestiaOptions(ValidOptions(), &error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(HestiaOptionsTest, AcceptsValidDualMode) {
    std::string error;
    EXPECT_TRUE(ValidateHestiaOptions(ValidDualOptions(), &error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(HestiaOptionsTest, RejectsUnknownMode) {
    auto o = ValidOptions();
    o.set_mode("all");
    std::string error;
    EXPECT_FALSE(ValidateHestiaOptions(o, &error));
    EXPECT_FALSE(error.empty());
}

TEST(HestiaOptionsTest, DualRequiresHomeModelAndLabels) {
    auto o = ValidDualOptions();
    std::string error;
    EXPECT_TRUE(ValidateHestiaOptions(o, &error)) << error;
    o.clear_home_labels();
    EXPECT_FALSE(ValidateHestiaOptions(o, &error));
    EXPECT_FALSE(error.empty());
}

TEST(HestiaOptionsTest, RejectsInvalidIndividualFields) {
    struct InvalidCase {
        std::string name;
        std::function<void(proto::HestiaOptions*)> mutate;
    };
    const std::vector<InvalidCase> cases = {
        {"empty open model path in open mode",
         [](proto::HestiaOptions* o) { o->clear_open_model_path(); }},
        {"empty open prompts in open mode",
         [](proto::HestiaOptions* o) { o->clear_open_prompts(); }},
        {"unknown backend",
         [](proto::HestiaOptions* o) { o->set_backend("rknn"); }},
        {"zero open width",
         [](proto::HestiaOptions* o) { o->set_open_width(0); }},
        {"zero open height",
         [](proto::HestiaOptions* o) { o->set_open_height(0); }},
        {"zero max detections",
         [](proto::HestiaOptions* o) { o->set_max_detections(0); }},
        {"min depth not less than max",
         [](proto::HestiaOptions* o) {
             o->set_min_depth_m(8.0F);
             o->set_max_depth_m(8.0F);
         }},
        {"empty detections 2d topic",
         [](proto::HestiaOptions* o) { o->clear_detections_2d_topic(); }},
        {"empty detections 3d topic",
         [](proto::HestiaOptions* o) { o->clear_detections_3d_topic(); }},
        {"identical 2d and 3d topics",
         [](proto::HestiaOptions* o) {
             o->set_detections_3d_topic(o->detections_2d_topic());
         }},
        {"inner box scale zero",
         [](proto::HestiaOptions* o) { o->set_inner_box_scale(0.0F); }},
        {"inner box scale above one",
         [](proto::HestiaOptions* o) { o->set_inner_box_scale(1.1F); }},
        {"dual missing home model",
         [](proto::HestiaOptions* o) {
             *o = ValidDualOptions();
             o->clear_home_model_path();
         }},
        {"dual zero home width",
         [](proto::HestiaOptions* o) {
             *o = ValidDualOptions();
             o->set_home_width(0);
         }},
    };

    for (const auto& test_case : cases) {
        SCOPED_TRACE(test_case.name);
        auto options = ValidOptions();
        test_case.mutate(&options);
        std::string error;
        EXPECT_FALSE(ValidateHestiaOptions(options, &error)) << test_case.name;
        EXPECT_FALSE(error.empty()) << test_case.name;
        EXPECT_NE(error.find("Hestia:"), std::string::npos) << error;
    }
}

}  // namespace
}  // namespace autonomy::perception::hestia
