/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/planning/smoother/savitzky_golay_smoother.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "autolink/common/log.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"

namespace autonomy {
namespace planning {
namespace smoother {

using namespace std::chrono;  // NOLINT

void SavitzkyGolaySmoother::Configure(std::string name, std::shared_ptr<void> /*costmap_sub*/,
                                      std::shared_ptr<map::costmap_2d::Costmap2DWrapper> /*costmap_wrapper*/) {
    // declare_parameter_if_not_declared(
    // node, name + ".do_refinement", rclcpp::ParameterValue(true));
    // declare_parameter_if_not_declared(
    // node, name + ".refinement_num", rclcpp::ParameterValue(2));
    // declare_parameter_if_not_declared(
    // node, name + ".enforce_path_inversion", rclcpp::ParameterValue(true));
    // declare_parameter_if_not_declared(
    // node, name + ".window_size", rclcpp::ParameterValue(7));
    // declare_parameter_if_not_declared(
    // node, name + ".poly_order", rclcpp::ParameterValue(3));
    // node->get_parameter(name + ".do_refinement", do_refinement_);
    // node->get_parameter(name + ".refinement_num", refinement_num_);
    // node->get_parameter(name + ".enforce_path_inversion",
    // enforce_path_inversion_); node->get_parameter(name + ".window_size",
    // window_size_); node->get_parameter(name + ".poly_order", poly_order_); if
    // (window_size_ % 2 == 0 || window_size_ <= 2) { throw
    // nav2_core::SmootherException(
    //         "Savitzky-Golay Smoother requires an odd window size of 3 or
    //         greater");
    // }
    // half_window_size_ = (window_size_ - 1) / 2;
    // calculateCoefficients();
}

// For more details on calculating Savitzky–Golay filter coefficients,
// see: https://www.colmryan.org/posts/savitsky_golay/
void SavitzkyGolaySmoother::CalculateCoefficients() {
    // We construct the Vandermonde matrix here
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(window_size_, -half_window_size_, half_window_size_);
    Eigen::MatrixXd x = Eigen::MatrixXd::Ones(window_size_, poly_order_ + 1);
    for (int i = 1; i <= poly_order_; i++) {
        x.col(i) = (x.col(i - 1).array() * v.array()).matrix();
    }
    // Compute the pseudoinverse of X, (X^T * X)^-1 * X^T
    Eigen::MatrixXd coeff_mat = (x.transpose() * x).inverse() * x.transpose();

    // Extract the smoothing coefficients
    sg_coeffs_ = coeff_mat.row(0).transpose();
}

bool SavitzkyGolaySmoother::Smooth(commsgs::planning_msgs::Path& path, const std::chrono::milliseconds& max_time) {
    steady_clock::time_point start = steady_clock::now();
    double time_remaining = max_time.count() / 1000.0;  // Convert milliseconds to seconds

    bool success = true, reversing_segment;
    commsgs::planning_msgs::Path curr_path_segment;
    curr_path_segment.header = path.header;

    // Simple path segment: entire path as one segment
    // For now, we don't split paths into directional segments
    struct PathSegment {
        unsigned int start;
        unsigned int end;
    };
    std::vector<PathSegment> path_segments{PathSegment{0u, static_cast<unsigned int>(path.poses.size() - 1)}};

    // Minimum point size to smooth is SG filter size + start + end
    unsigned int minimum_points = window_size_ + 2;
    for (unsigned int i = 0; i != path_segments.size(); i++) {
        if (path_segments[i].end - path_segments[i].start > minimum_points) {
            // Populate path segment
            curr_path_segment.poses.clear();
            std::copy(path.poses.begin() + path_segments[i].start, path.poses.begin() + path_segments[i].end + 1,
                      std::back_inserter(curr_path_segment.poses));

            // Make sure we're still able to smooth with time remaining
            steady_clock::time_point now = steady_clock::now();
            time_remaining = max_time.count() / 1000.0 - duration_cast<duration<double>>(now - start).count();

            if (time_remaining <= 0.0) {
                AWARN << "Smoothing time exceeded allowed duration of " << max_time.count() / 1000.0 << " seconds";
                throw common::SmootherTimedOut("Smoothing time exceed allowed duration");
            }

            // Smooth path segment
            success = success && SmoothImpl(curr_path_segment, reversing_segment);

            // Assemble the path changes to the main path
            std::copy(curr_path_segment.poses.begin(), curr_path_segment.poses.end(),
                      path.poses.begin() + path_segments[i].start);
        }
    }

    return success;
}

bool SavitzkyGolaySmoother::SmoothImpl(commsgs::planning_msgs::Path& path, bool& reversing_segment) {
    const unsigned int& path_size = path.poses.size();

    // Convert PoseStamped to Eigen
    auto toEigenVec = [](const commsgs::geometry_msgs::PoseStamped& pose) -> Eigen::Vector2d {
        return {pose.pose.position.x, pose.pose.position.y};
    };

    auto ApplyFilterOverAxes = [&](std::vector<commsgs::geometry_msgs::PoseStamped>& plan_pts,
                                   const std::vector<Eigen::Vector2d>& init_plan_pts) -> void {
        // First point is fixed
        for (unsigned int idx = 1; idx != path_size - 1; idx++) {
            Eigen::Vector2d accum(0.0, 0.0);

            for (int j = -half_window_size_; j <= half_window_size_; j++) {
                int path_idx = std::clamp<int>(idx + j, 0, path_size - 1);
                accum += sg_coeffs_(j + half_window_size_) * init_plan_pts[path_idx];
            }
            plan_pts[idx].pose.position.x = accum.x();
            plan_pts[idx].pose.position.y = accum.y();
        }
    };

    std::vector<Eigen::Vector2d> initial_path_poses(path.poses.size());
    std::transform(path.poses.begin(), path.poses.end(), initial_path_poses.begin(), toEigenVec);
    ApplyFilterOverAxes(path.poses, initial_path_poses);

    // Let's do additional refinement, it shouldn't take more than a couple
    // milliseconds
    if (do_refinement_) {
        for (int i = 0; i < refinement_num_; i++) {
            std::vector<Eigen::Vector2d> reined_initial_path_poses(path.poses.size());
            std::transform(path.poses.begin(), path.poses.end(), reined_initial_path_poses.begin(), toEigenVec);
            ApplyFilterOverAxes(path.poses, reined_initial_path_poses);
        }
    }

    map::costmap_2d::utils::updateApproximatePathOrientations(path, reversing_segment);
    return true;
}

}  // namespace smoother
}  // namespace planning
}  // namespace autonomy