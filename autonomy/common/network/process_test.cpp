/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/network/process.hpp"
#include "autonomy/common/network/tensor.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace common {
namespace network {
namespace {

TEST(NetworkProcessTest, FindExactOutputName) {
    TensorMap outputs;
    outputs["depth"] = {1.f, 2.f};
    outputs["aux"] = {3.f};

    std::vector<ModelTensorInfo> infos(2);
    infos[0].name = "depth";
    infos[1].name = "aux";

    std::string name;
    const std::vector<float>* data = nullptr;
    ASSERT_TRUE(Find(outputs, infos, &name, &data, "depth", nullptr));
    EXPECT_EQ(name, "depth");
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data->size(), 2u);
}

TEST(NetworkProcessTest, TopK) {
  std::vector<float> logits = {0.1f, 0.9f, 0.3f};
  std::vector<ClassScore> scores;
  ASSERT_TRUE(TopK(logits, 2, &scores, nullptr));
  ASSERT_EQ(scores.size(), 2u);
  EXPECT_EQ(scores[0].class_id, 1);
  EXPECT_NEAR(scores[0].score, 0.9f, 1e-6f);
}

TEST(NetworkProcessTest, ResolveShapeForFloatCount) {
    ModelTensorInfo info;
    info.name = "input";
    info.shape = TensorShape({1, 3, 4, 4});

    std::vector<int64_t> resolved;
    ASSERT_TRUE(ResolveShapeForFloatCount(info, 48, &resolved, nullptr));
    ASSERT_EQ(resolved.size(), 4u);
    EXPECT_EQ(resolved[0], 1);
}

TEST(NetworkProcessTest, MakeBoundOptions) {
    const PreprocessOptions options = MakeBound(504, 14, 518, 518);
    EXPECT_EQ(options.resize, ResizePolicy::kUpperBound);
    EXPECT_EQ(options.bound_resize_target, 504);
    EXPECT_EQ(options.align_multiple, 14);
    EXPECT_EQ(options.normalize, NormalizePolicy::kImageNet);
}

}  // namespace
}  // namespace network
}  // namespace common
}  // namespace autonomy
