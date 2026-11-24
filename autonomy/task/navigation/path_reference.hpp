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
 * @file path_reference.hpp
 * @brief Preserve complete route geometry when compacting and comparing paths.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_PATH_REFERENCE_HPP_
#define AUTONOMY_TASK_NAVIGATION_PATH_REFERENCE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autonomy::task::navigation {

/**
 * @brief Geometry and orientation of one controller reference pose.
 */
struct PathReferencePose {
    std::array<double, 3> position{};     ///< Metric XYZ in the path frame.
    std::array<double, 4> orientation{};  ///< Quaternion XYZW; zero stays unspecified.
};

/**
 * @brief An immutable route reference independent of message publication stamps.
 *
 * Frame, controller, every bend, height and orientation remain part of identity.
 * A moving start on the same remaining polyline does not force preemption.
 */
class PathReference {
public:
    /**
     * @brief Validate and retain one complete route before dispatch.
     * @param frame Coordinate frame of every pose.
     * @param controller Selected controller; an empty value uses server policy.
     * @param poses Complete original poses, including the final goal.
     * @throws std::invalid_argument For an empty frame/path or nonfinite data.
     */
    PathReference(std::string frame, std::string controller,
                  std::vector<PathReferencePose> poses)
        : frame_(std::move(frame)), controller_(std::move(controller)),
          poses_(std::move(poses)) {
        if (frame_.empty() || poses_.empty())
            throw std::invalid_argument("path reference requires a frame and poses");
        for (const auto& pose : poses_) {
            for (double value : pose.position)
                if (!std::isfinite(value)) throw std::invalid_argument("nonfinite path position");
            for (double value : pose.orientation)
                if (!std::isfinite(value)) throw std::invalid_argument("nonfinite path orientation");
        }
        retained_.push_back(0);
        std::size_t anchor = 0;
        for (std::size_t end = 2; end < poses_.size(); ++end) {
            if (!PreservesInterval(anchor, end)) {
                retained_.push_back(end - 1);
                anchor = end - 1;
            }
        }
        if (poses_.size() > 1) retained_.push_back(poses_.size() - 1);
    }

    /**
     * @brief Original indices which preserve all bends and orientation changes.
     * @return An ordered subset retaining both endpoints and the final goal.
     * Every removed original pose is checked against the resulting segment;
     * a chain of small deviations cannot accumulate into a shortcut.
     */
    const std::vector<std::size_t>& retained_indices() const { return retained_; }

    /**
     * @brief Check whether a new route is only a forward trim of this reference.
     * @param candidate New complete path to the same final goal.
     * @return True only when its remaining geometry, frame and controller agree.
     *
     * The first matching anchor is decisive. A repeated point in a loop cannot
     * hide removal of a future loop by matching a later occurrence instead.
     */
    bool HasSameRemainingPath(const PathReference& candidate) const {
        if (frame_ != candidate.frame_ || controller_ != candidate.controller_) return false;
        const auto& first = candidate.poses_.at(candidate.retained_.front());
        for (std::size_t i = 0; i < retained_.size(); ++i) {
            const auto& here = poses_[retained_[i]];
            if (SamePosition(here, first)) {
                return SamePose(here, first) && SameTail(i, 0, candidate);
            }
            if (i + 1 < retained_.size()) {
                const auto& next = poses_[retained_[i + 1]];
                // A vertex is compared by the next iteration; treating it as
                // an interior anchor would require that vertex twice.
                if (!SamePosition(first, next) && OnSegment(first, here, next)) {
                    return SameOrientation(first, here) && SameOrientation(first, next) &&
                           SameTail(i + 1, 1, candidate);
                }
            }
        }
        return false;
    }

private:
    static constexpr long double kPositionTolerance = 1e-9L; ///< Numerical tolerance in metres.
    static constexpr double kOrientationTolerance = 1e-12;   ///< Quaternion component tolerance.

    /**
     * @brief Compare metric positions without quantizing or wrapping
              * coordinates.
     */
    static bool SamePosition(const PathReferencePose& a, const PathReferencePose& b) {
        long double squared = 0;
        for (std::size_t j = 0; j < 3; ++j) {
            const long double delta = static_cast<long double>(a.position[j]) - b.position[j];
            squared += delta * delta;
        }
        return squared <= kPositionTolerance * kPositionTolerance;
    }

    /**
     * @brief Quaternion sign changes describe the same orientation.
     */
    static bool SameOrientation(const PathReferencePose& a, const PathReferencePose& b) {
        bool same = true;
        bool negated = true;
        for (std::size_t j = 0; j < 4; ++j) {
            same = same && std::abs(a.orientation[j] - b.orientation[j]) <= kOrientationTolerance;
            negated = negated && std::abs(a.orientation[j] + b.orientation[j]) <= kOrientationTolerance;
        }
        return same || negated;
    }

    /**
     * @brief Compare all controller-visible pose geometry.
     */
    static bool SamePose(const PathReferencePose& a, const PathReferencePose& b) {
        return SamePosition(a, b) && SameOrientation(a, b);
    }

    /**
     * @brief Test a point against the bounded 3D segment, never its infinite
              * line.
     */
    static bool OnSegment(const PathReferencePose& point, const PathReferencePose& a,
                          const PathReferencePose& b) {
        std::array<long double, 3> direction{};
        std::array<long double, 3> offset{};
        long double length_squared = 0;
        long double projection = 0;
        for (std::size_t j = 0; j < 3; ++j) {
            direction[j] = static_cast<long double>(b.position[j]) - a.position[j];
            offset[j] = static_cast<long double>(point.position[j]) - a.position[j];
            length_squared += direction[j] * direction[j];
            projection += offset[j] * direction[j];
        }
        if (length_squared == 0) return SamePosition(point, a);
        const long double fraction = projection / length_squared;
        if (fraction < 0 || fraction > 1) return false;
        long double error_squared = 0;
        for (std::size_t j = 0; j < 3; ++j) {
            const auto error = offset[j] - fraction * direction[j];
            error_squared += error * error;
        }
        return error_squared <= kPositionTolerance * kPositionTolerance;
    }

    /**
     * @brief Validate every omitted pose against the actual compacted
              * segment.
     */
    bool PreservesInterval(std::size_t first, std::size_t last) const {
        if (!SameOrientation(poses_[first], poses_[last])) return false;
        for (std::size_t i = first + 1; i < last; ++i) {
            if (!SameOrientation(poses_[first], poses_[i]) ||
                !OnSegment(poses_[i], poses_[first], poses_[last])) return false;
        }
        return true;
    }

    /**
     * @brief Compare the entire remaining ordered sequence, not just its
              * endpoint.
     */
    bool SameTail(std::size_t here, std::size_t there, const PathReference& candidate) const {
        if (retained_.size() - here != candidate.retained_.size() - there) return false;
        for (; here < retained_.size(); ++here, ++there) {
            if (!SamePose(poses_[retained_[here]], candidate.poses_[candidate.retained_[there]]))
                return false;
        }
        return true;
    }

    std::string frame_;                       ///< Stable coordinate frame.
    std::string controller_;                  ///< Controller selection affects execution.
    std::vector<PathReferencePose> poses_;      ///< Complete original path evidence.
    std::vector<std::size_t> retained_;         ///< Geometry-preserving original indices.
};

}  // namespace autonomy::task::navigation

#endif  // AUTONOMY_TASK_NAVIGATION_PATH_REFERENCE_HPP_
