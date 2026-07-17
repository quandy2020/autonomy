/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/apps/teleop/intent_path_selector.hpp"

#include <memory>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "gtest/gtest.h"

namespace autonomy::task::teleop {
namespace {

using map::costmap_2d::Costmap2D;
using map::costmap_2d::FREE_SPACE;
using map::costmap_2d::LETHAL_OBSTACLE;

std::shared_ptr<Costmap2D> MakeEmptyMap() {
    auto map = std::make_shared<Costmap2D>();
    map->setDefaultValue(FREE_SPACE);
    map->resizeMap(200, 200, 0.05, -5.0, -5.0);
    return map;
}

TEST(IntentPathSelectorTest, PrefersForwardWhenOpen) {
    IntentPathSelector selector;
    selector.GenerateDefaultLibrary(/*num_dirs=*/9, /*num_lengths=*/3,
                                    /*max_range=*/3.0, /*ds=*/0.1);
    auto map = MakeEmptyMap();
    auto path = selector.Select(*map, 0.0, 0.5);
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->poses.size(), 2u);
    EXPECT_GT(path->poses[1].pose.position.x, path->poses[0].pose.position.x);
}

TEST(IntentPathSelectorTest, AvoidsLethalAhead) {
    IntentPathSelector selector;
    selector.GenerateDefaultLibrary(9, 3, 3.0, 0.1);
    auto map = MakeEmptyMap();
    for (double y = -0.4; y <= 0.4; y += 0.05) {
        unsigned int mx, my;
        ASSERT_TRUE(map->worldToMap(1.0, y, mx, my));
        map->setCost(mx, my, LETHAL_OBSTACLE);
    }
    auto path = selector.Select(*map, 0.0, 0.5);
    ASSERT_TRUE(path.has_value());
    for (const auto& ps : path->poses) {
        unsigned int mx, my;
        if (map->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my)) {
            EXPECT_LT(map->getCost(mx, my), LETHAL_OBSTACLE);
        }
    }
}

TEST(IntentPathSelectorTest, ZeroSpeedReturnsNullopt) {
    IntentPathSelector selector;
    selector.GenerateDefaultLibrary(5, 2, 2.0, 0.1);
    auto map = MakeEmptyMap();
    EXPECT_FALSE(selector.Select(*map, 0.0, 0.0).has_value());
}

}  // namespace
}  // namespace autonomy::task::teleop
