/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "atlas/trajectory_export_policy.hpp"

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

class FakeTrajectoryMap
{
public:
    explicit FakeTrajectoryMap(std::size_t keyframe_count)
        : keyframes_(keyframe_count) {}

    const std::vector<int>& GetAllKeyFrames() {
        return keyframes_;
    }

private:
    std::vector<int> keyframes_;
};

TEST(TrajectoryExportPolicyTest, SelectsLargestNonemptyMapDeterministically) {
    FakeTrajectoryMap empty(0);
    FakeTrajectoryMap smaller(3);
    FakeTrajectoryMap largest(8);
    FakeTrajectoryMap tied_largest(8);
    const std::vector<FakeTrajectoryMap*> maps = {nullptr, &empty, &smaller,
                                                  &largest, &tied_largest};

    EXPECT_EQ(SelectLargestTrajectoryMap(maps), &largest);
}

TEST(TrajectoryExportPolicyTest, ReturnsNullWithoutAUsableMap) {
    FakeTrajectoryMap empty(0);
    const std::vector<FakeTrajectoryMap*> maps = {nullptr, &empty};

    EXPECT_EQ(SelectLargestTrajectoryMap(maps), nullptr);
}

TEST(TrajectoryExportPolicyTest, ExcludesReferencesFromIndependentMaps) {
    FakeTrajectoryMap selected_map(8);
    FakeTrajectoryMap independent_map(5);

    EXPECT_TRUE(
        ShouldExportTrajectoryReferenceFromMap(&selected_map, &selected_map));
    EXPECT_FALSE(ShouldExportTrajectoryReferenceFromMap(&independent_map,
                                                        &selected_map));
    EXPECT_FALSE(ShouldExportTrajectoryReferenceFromMap(
        static_cast<FakeTrajectoryMap*>(nullptr), &selected_map));
    EXPECT_FALSE(ShouldExportTrajectoryReferenceFromMap(
        &selected_map, static_cast<FakeTrajectoryMap*>(nullptr)));
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
