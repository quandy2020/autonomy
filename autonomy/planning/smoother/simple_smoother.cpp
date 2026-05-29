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

#include "autonomy/planning/smoother/simple_smoother.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/utils/smoother_utils.hpp"
namespace autonomy {
namespace planning {
namespace smoother {

using namespace std::chrono;  // NOLINT

SimpleSmoother::~SimpleSmoother() = default;

void SimpleSmoother::ApplyOptions(const proto::SimpleSmootherOptions& options) {
    if (options.tolerance() > 0.0) {
        tolerance_ = options.tolerance();
    }
    if (options.max_iterations() > 0) {
        max_its_ = options.max_iterations();
    }
    if (options.w_data() >= 0.0) {
        data_w_ = options.w_data();
    }
    if (options.w_smooth() >= 0.0) {
        smooth_w_ = options.w_smooth();
    }
    do_refinement_ = options.do_refinement();
    if (options.refinement_num() > 0) {
        refinement_num_ = options.refinement_num();
    }
    enforce_path_inversion_ = options.enforce_path_inversion();
}

void SimpleSmoother::Configure(
    std::string name, std::shared_ptr<void> /*costmap_sub*/,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = costmap_wrapper;

    ApplyOptions(proto::SimpleSmootherOptions{});
    tolerance_ = tolerance_ > 0.0 ? tolerance_ : 1e-10;
    max_its_ = max_its_ > 0 ? max_its_ : 1000;
    data_w_ = data_w_ >= 0.0 ? data_w_ : 0.2;
    smooth_w_ = smooth_w_ >= 0.0 ? smooth_w_ : 0.3;
    refinement_num_ = refinement_num_ > 0 ? refinement_num_ : 2;

    // declare_parameter_if_not_declared(
    // node, name + ".tolerance", rclcpp::ParameterValue(1e-10));
    // declare_parameter_if_not_declared(
    // node, name + ".max_its", rclcpp::ParameterValue(1000));
    // declare_parameter_if_not_declared(
    // node, name + ".w_data", rclcpp::ParameterValue(0.2));
    // declare_parameter_if_not_declared(
    // node, name + ".w_smooth", rclcpp::ParameterValue(0.3));
    // declare_parameter_if_not_declared(
    // node, name + ".do_refinement", rclcpp::ParameterValue(true));
    // declare_parameter_if_not_declared(
    // node, name + ".refinement_num", rclcpp::ParameterValue(2));
    // declare_parameter_if_not_declared(
    // node, name + ".enforce_path_inversion", rclcpp::ParameterValue(true));

    // node->get_parameter(name + ".tolerance", tolerance_);
    // node->get_parameter(name + ".max_its", max_its_);
    // node->get_parameter(name + ".w_data", data_w_);
    // node->get_parameter(name + ".w_smooth", smooth_w_);
    // node->get_parameter(name + ".do_refinement", do_refinement_);
    // node->get_parameter(name + ".refinement_num", refinement_num_);
    // node->get_parameter(name + ".enforce_path_inversion",
    // enforce_path_inversion_);
}

bool SimpleSmoother::Smooth(commsgs::planning_msgs::Path& path,
                            const std::chrono::milliseconds& max_time) {
    // Get costmap from wrapper if available, otherwise use nullptr
    map::costmap_2d::Costmap2D* costmap = nullptr;
    if (costmap_wrapper_) {
        costmap = costmap_wrapper_->getCostmap();
    }

    steady_clock::time_point start = steady_clock::now();
    double max_time_seconds =
        max_time.count() / 1000.0;  // Convert milliseconds to seconds
    double time_remaining = max_time_seconds;

    bool reversing_segment;
    commsgs::planning_msgs::Path curr_path_segment;
    curr_path_segment.header = path.header;

    const auto directional_segments =
        utils::findDirectionalPathSegments(path, false);
    struct PathSegment {
        unsigned int start;
        unsigned int end;
    };
    std::vector<PathSegment> path_segments;
    path_segments.reserve(directional_segments.size());
    for (const auto& segment : directional_segments) {
        path_segments.push_back(
            PathSegment{segment.start, segment.end});
    }
    if (path_segments.empty() && path.poses.size() >= 2) {
        path_segments.push_back(PathSegment{
            0u, static_cast<unsigned int>(path.poses.size() - 1)});
    }

    // Note: Costmap2D mutex locking removed - costmap access should be
    // thread-safe via wrapper

    for (unsigned int i = 0; i != path_segments.size(); i++) {
        if (path_segments[i].end - path_segments[i].start > 3) {
            // Populate path segment
            curr_path_segment.poses.clear();
            std::copy(path.poses.begin() + path_segments[i].start,
                      path.poses.begin() + path_segments[i].end + 1,
                      std::back_inserter(curr_path_segment.poses));

            // Make sure we're still able to smooth with time remaining
            steady_clock::time_point now = steady_clock::now();
            time_remaining =
                max_time.count() / 1000.0 -
                duration_cast<duration<double>>(now - start).count();
            if (time_remaining <= 0.0) {
                AWARN << "No smoothing time remaining (" << time_remaining
                      << " s); skipping remaining segments";
                break;
            }
            refinement_ctr_ = 0;

            // Attempt to smooth the segment
            // May throw SmootherTimedOut
            SmoothImpl(curr_path_segment, reversing_segment, costmap,
                       time_remaining);

            // Assemble the path changes to the main path
            std::copy(curr_path_segment.poses.begin(),
                      curr_path_segment.poses.end(),
                      path.poses.begin() + path_segments[i].start);
        }
    }

    return true;
}

void SimpleSmoother::SmoothImpl(commsgs::planning_msgs::Path& path,
                                bool& reversing_segment,
                                const map::costmap_2d::Costmap2D* costmap,
                                const double& max_time_seconds) {
    steady_clock::time_point a = steady_clock::now();

    int its = 0;
    double change = tolerance_;
    const unsigned int& path_size = path.poses.size();
    double x_i, y_i, y_m1, y_ip1, y_i_org;
    unsigned int mx, my;

    commsgs::planning_msgs::Path new_path = path;
    commsgs::planning_msgs::Path last_path = path;

    while (change >= tolerance_) {
        its += 1;
        change = 0.0;

        // Make sure the smoothing function will converge
        if (its >= max_its_) {
            AWARN << "Number of iterations has exceeded limit of " << max_its_;
            path = last_path;
            map::costmap_2d::utils::updateApproximatePathOrientations(
                path, reversing_segment);
            return;
        }

        // Make sure still have time left to process
        steady_clock::time_point b = steady_clock::now();
        double elapsed_seconds = duration_cast<duration<double>>(b - a).count();
        if (max_time_seconds > 0.0 && elapsed_seconds > max_time_seconds) {
            AWARN << "Smoothing time exceeded limit of " << max_time_seconds
                  << " seconds (elapsed " << elapsed_seconds << " s)";
            path = last_path;
            map::costmap_2d::utils::updateApproximatePathOrientations(
                path, reversing_segment);
            throw common::SmootherException(
                "Smoothing time exceed allowed duration");
        }

        for (unsigned int i = 1; i != path_size - 1; i++) {
            for (unsigned int j = 0; j != 2; j++) {
                x_i = GetFieldByDim(path.poses[i], j);
                y_i = GetFieldByDim(new_path.poses[i], j);
                y_m1 = GetFieldByDim(new_path.poses[i - 1], j);
                y_ip1 = GetFieldByDim(new_path.poses[i + 1], j);
                y_i_org = y_i;

                // Smooth based on local 3 point neighborhood and original data
                // locations
                y_i += data_w_ * (x_i - y_i) +
                       smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
                SetFieldByDim(new_path.poses[i], j, y_i);
                change += abs(y_i - y_i_org);
            }

            // validate update is admissible, only checks cost if a valid
            // costmap pointer is provided
            float cost = 0.0;
            if (costmap) {
                costmap->worldToMap(GetFieldByDim(new_path.poses[i], 0),
                                    GetFieldByDim(new_path.poses[i], 1), mx,
                                    my);
                cost = static_cast<float>(costmap->getCost(mx, my));
            }

            if (cost > map::costmap_2d::MAX_NON_OBSTACLE &&
                cost != map::costmap_2d::NO_INFORMATION) {
                AWARN
                    << "Smoothing process resulted in an infeasible collision. "
                       "Returning the last path before the infeasibility was "
                       "introduced.";
                path = last_path;
                map::costmap_2d::utils::updateApproximatePathOrientations(
                    path, reversing_segment);
                throw common::SmootherException(
                    "Smoothing process resulted in an infeasible collision. "
                    "Returning the last path before the infeasibility was "
                    "introduced.");
            }
        }

        last_path = new_path;
    }

    // Let's do additional refinement, it shouldn't take more than a couple
    // milliseconds but really puts the path quality over the top.
    if (do_refinement_ && refinement_ctr_ < refinement_num_) {
        refinement_ctr_++;
        SmoothImpl(new_path, reversing_segment, costmap, max_time_seconds);
    }

    map::costmap_2d::utils::updateApproximatePathOrientations(
        new_path, reversing_segment);
    path = new_path;
}

double SimpleSmoother::GetFieldByDim(
    const commsgs::geometry_msgs::PoseStamped& msg, const unsigned int& dim) {
    if (dim == 0) {
        return msg.pose.position.x;
    } else if (dim == 1) {
        return msg.pose.position.y;
    } else {
        return msg.pose.position.z;
    }
}

void SimpleSmoother::SetFieldByDim(commsgs::geometry_msgs::PoseStamped& msg,
                                   const unsigned int dim,
                                   const double& value) {
    if (dim == 0) {
        msg.pose.position.x = value;
    } else if (dim == 1) {
        msg.pose.position.y = value;
    } else {
        msg.pose.position.z = value;
    }
}

}  // namespace smoother
}  // namespace planning
}  // namespace autonomy

using autonomy::planning::common::Smoother;
using autonomy::planning::smoother::SimpleSmoother;

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(SimpleSmoother, Smoother);
