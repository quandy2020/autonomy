/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file path_reference_test.cpp
 * @brief Route replacement and compaction contracts using the production helper.
 */

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomy/task/navigation/path_reference.hpp"

namespace {
using Pose = autonomy::task::navigation::PathReferencePose;
using Reference = autonomy::task::navigation::PathReference;

/**
 * @brief Fail immediately with the violated route contract.
 */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/**
 * @brief Construct metric test input, without substituting production logic.
 */
Pose Point(double x, double y = 0, double z = 0) {
    return {{x, y, z}, {0, 0, 0, 1}};
}

/**
 * @brief Dense straight paths compress, while a narrow detour is fully
          * retained.
 */
void CompleteGeometry() {
    std::vector<Pose> straight;
    for (int i = 0; i <= 200; ++i) straight.push_back(Point(i * 0.01));
    Reference line("map", "", straight);
    Require(line.retained_indices() == std::vector<std::size_t>({0, 200}),
            "dense straight route did not preserve exact endpoints");
    straight[101].position[1] = 0.04;
    Reference detour("map", "", straight);
    const auto& kept = detour.retained_indices();
    Require(kept == std::vector<std::size_t>({0, 100, 101, 102, 200}),
            "a necessary narrow detour was removed");
    Require(!line.HasSameRemainingPath(detour), "same endpoint concealed a changed route");
    Reference origin("map", "", {Point(1), Point(0)});
    Require(origin.HasSameRemainingPath(origin), "origin goal was treated as a sentinel");
}

/**
 * @brief Forward start motion and resampling do not spuriously replace a
          * route.
 */
void ProgressAndResampling() {
    Reference original("map", "controller", {Point(0), Point(2), Point(2, 2), Point(4, 2)});
    Reference moved("map", "controller", {Point(0.75), Point(1), Point(2), Point(2, 2), Point(4, 2)});
    Require(original.HasSameRemainingPath(moved), "forward progress forced replacement");
    Reference past_corner("map", "controller", {Point(2, 0.5), Point(2, 2), Point(4, 2)});
    Require(original.HasSameRemainingPath(past_corner), "passed corner forced replacement");
    Reference vertex("map", "controller", {Point(2, 2), Point(4, 2)});
    Require(original.HasSameRemainingPath(vertex), "exact vertex progress was rejected");
    Reference goal("map", "controller", {Point(4, 2)});
    Require(original.HasSameRemainingPath(goal), "final remaining point was rejected");
    Reference lateral("map", "controller", {Point(0.75, 0.01), Point(2), Point(2, 2), Point(4, 2)});
    Require(!original.HasSameRemainingPath(lateral), "off-route recovery was hidden");
    Reference reverse("map", "controller", {Point(-1), Point(2), Point(2, 2), Point(4, 2)});
    Require(!original.HasSameRemainingPath(reverse), "backward extension was ignored");
}

/**
 * @brief Changed controller, frame, height and terminal yaw are new intent.
 */
void ReferenceIdentity() {
    Reference original("map", "first", {Point(0), Point(1)});
    Require(!original.HasSameRemainingPath(Reference("odom", "first", {Point(0), Point(1)})),
            "frame change was ignored");
    Require(!original.HasSameRemainingPath(Reference("map", "second", {Point(0), Point(1)})),
            "controller change was ignored");
    Require(!original.HasSameRemainingPath(Reference("map", "first", {Point(0), Point(1, 0, 0.1)})),
            "height change was ignored");
    auto turned = Point(1); turned.orientation = {0, 0, 1, 0};
    Require(!original.HasSameRemainingPath(Reference("map", "first", {Point(0), turned})),
            "terminal yaw change was ignored");
    auto negated = Point(1); negated.orientation = {0, 0, 0, -1};
    Require(original.HasSameRemainingPath(Reference("map", "first", {Point(0), negated})),
            "quaternion sign alone changed reference identity");
    auto middle = Point(0.5); middle.orientation = turned.orientation;
    Reference rotate("map", "first", {Point(0), middle, Point(1)});
    Require(rotate.retained_indices().size() == 3, "intermediate orientation was removed");
}

/**
 * @brief Revisiting a coordinate cannot conceal removal of a future loop.
 */
void LoopsAndBacktracking() {
    Reference loop("map", "", {Point(0), Point(1), Point(1, 1), Point(0, 1), Point(0), Point(2)});
    Require(!loop.HasSameRemainingPath(Reference("map", "", {Point(0), Point(2)})),
            "a later duplicate anchor concealed a removed loop");
    Reference reversal("map", "", {Point(0), Point(2), Point(1), Point(3)});
    Require(reversal.retained_indices().size() == 4, "backtracking was compressed away");
    Reference duplicate("map", "", {Point(0), Point(0), Point(1)});
    Require(duplicate.retained_indices() == std::vector<std::size_t>({0, 2}),
            "an exactly redundant pose was not compacted");
}

/**
 * @brief Rechecking all original poses prevents accumulated near-collinear
          * drift.
 */
void BoundedCompaction() {
    std::vector<Pose> arc;
    for (int i = 0; i < 200; ++i) arc.push_back(Point(i, i * i * 1e-10));
    Reference reference("map", "", arc);
    const auto& kept = reference.retained_indices();
    Require(kept.size() > 2, "local tolerances accumulated into a large shortcut");
    for (std::size_t j = 1; j < kept.size(); ++j) {
        const auto& a = arc[kept[j - 1]].position;
        const auto& b = arc[kept[j]].position;
        for (std::size_t i = kept[j - 1] + 1; i < kept[j]; ++i) {
            const double expected = a[1] + (arc[i].position[0] - a[0]) * (b[1] - a[1]) / (b[0] - a[0]);
            Require(std::abs(arc[i].position[1] - expected) <= 1.000001e-9,
                    "compacted segment exceeded its numerical tolerance");
        }
    }
}

/**
 * @brief Missing frames and nonfinite geometry must fail before transmission.
 */
void InvalidInput() {
    for (int kind = 0; kind < 4; ++kind) {
        bool rejected = false;
        try {
            auto bad = Point(0);
            if (kind == 2) bad.position[0] = std::numeric_limits<double>::quiet_NaN();
            if (kind == 3) bad.orientation[3] = std::numeric_limits<double>::infinity();
            Reference reference(kind == 0 ? "" : "map", "", kind == 1 ? std::vector<Pose>{} : std::vector<Pose>{bad});
        } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "invalid route evidence was accepted");
    }
}
}  // namespace

/**
 * @brief Execute named geometry contracts; this does not validate middleware.
 */
int main() {
    try {
        for (const auto& test : std::vector<std::pair<const char*, void(*)()>>{
                 {"complete geometry and origin", CompleteGeometry},
                 {"forward progress and resampling", ProgressAndResampling},
                 {"frame controller height and orientation", ReferenceIdentity},
                 {"loops and backtracking", LoopsAndBacktracking},
                 {"bounded compaction over original poses", BoundedCompaction},
                 {"invalid input rejected", InvalidInput}}) {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
