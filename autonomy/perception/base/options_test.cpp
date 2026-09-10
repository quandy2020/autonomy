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
 * @brief Contract tests for YOLO26 base perception options.
 */

#include "autonomy/perception/base/options.hpp"

#include <gtest/gtest.h>

#include <string>

namespace autonomy::perception::base {
namespace {

proto::TaskOptions* AddTask(proto::BaseOptions* options, proto::TaskKind kind,
                            const char* path, const char* topic) {
    auto* task = options->add_tasks();
    task->set_kind(kind);
    task->set_enable(true);
    task->set_model_path(path);
    task->set_topic(topic);
    return task;
}

proto::BaseOptions ValidOptions() {
    proto::BaseOptions options;
    options.set_backend(proto::BACKEND_TENSORRT);
    options.set_input_width(640);
    options.set_input_height(640);
    options.set_confidence_threshold(0.25F);
    options.set_nms_iou_threshold(0.7F);
    options.set_max_detections(100);
    options.add_class_names("person");
    options.add_class_names("obstacle");
    options.set_camera_frame("camera_color_optical_frame");
    AddTask(&options, proto::TASK_DETECT, "/models/yolo26-detect.engine",
            "/perception/base/detections");
    return options;
}

TEST(BaseOptionsTest, AcceptsDetectOnly) {
    std::string error;
    EXPECT_TRUE(ValidateBaseOptions(ValidOptions(), &error));
    EXPECT_TRUE(error.empty());
}

TEST(BaseOptionsTest, RequiresATask) {
    auto options = ValidOptions();
    options.mutable_tasks(0)->set_enable(false);
    std::string error;
    EXPECT_FALSE(ValidateBaseOptions(options, &error));
    EXPECT_NE(error.find("at least one task"), std::string::npos);
}

TEST(BaseOptionsTest, RequiresPoseKeypoints) {
    auto options = ValidOptions();
    AddTask(&options, proto::TASK_POSE, "/models/yolo26-pose.engine",
            "/perception/base/poses");
    std::string error;
    EXPECT_FALSE(ValidateBaseOptions(options, &error));
    EXPECT_NE(error.find("num_keypoints"), std::string::npos);
}

TEST(BaseOptionsTest, RejectsUnalignedInputForYolo) {
    auto options = ValidOptions();
    options.set_input_width(848);
    std::string error;
    EXPECT_FALSE(ValidateBaseOptions(options, &error));
    EXPECT_NE(error.find("divisible by 32"), std::string::npos);
}

TEST(BaseOptionsTest, AcceptsDepthOnlyNonAlignedInput) {
    proto::BaseOptions options;
    options.set_backend(proto::BACKEND_TENSORRT);
    options.set_input_width(518);
    options.set_input_height(518);
    options.set_confidence_threshold(0.25F);
    options.set_nms_iou_threshold(0.7F);
    options.set_max_detections(100);
    options.set_camera_frame("camera_color_optical_frame");
    auto* depth =
        AddTask(&options, proto::TASK_DEPTH, "/models/moge.engine",
                "/perception/base/depth");
    depth->set_depth_backend(proto::DEPTH_BACKEND_MOGE);
    std::string error;
    EXPECT_TRUE(ValidateBaseOptions(options, &error)) << error;
}

}  // namespace
}  // namespace autonomy::perception::base
