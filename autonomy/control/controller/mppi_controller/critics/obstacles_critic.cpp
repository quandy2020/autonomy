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

#include "autonomy/control/controller/mppi_controller/critics/obstacles_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void ObstaclesCritic::initialize() {
  if (!options_) {
    AWARN << "Options not set, using defaults";
    power_ = 1;
    repulsion_weight_ = 1.5f;
    critical_weight_ = 20.0f;
    collision_cost_ = 10000.0f;
    collision_margin_distance_ = 0.1f;
    near_goal_distance_ = 0.5f;
    inflation_layer_name_ = "";
    consider_footprint_ = false;
    return;
  }

  // Load from proto options
  if (options_->has_obstacles_critic()) {
    const auto& critic = options_->obstacles_critic();
    enabled_ = critic.enabled();
    power_ = critic.cost_power();
    repulsion_weight_ = static_cast<float>(critic.repulsion_weight());
    critical_weight_ = static_cast<float>(critic.critical_weight());
    collision_cost_ = static_cast<float>(critic.collision_cost());
    collision_margin_distance_ = static_cast<float>(critic.collision_margin_distance());
    near_goal_distance_ = static_cast<float>(critic.near_goal_distance());
    consider_footprint_ = critic.consider_footprint();
  } else {
    enabled_ = true;
    power_ = 1;
    repulsion_weight_ = 1.5f;           // Default
    critical_weight_ = 20.0f;           // Default
    collision_cost_ = 10000.0f;         // Default
    collision_margin_distance_ = 0.1f;  // Default
    near_goal_distance_ = 0.5f;         // Default
    consider_footprint_ = false;        // Default
  }

  collision_checker_.setCostmap(costmap_);
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f) {
    AERROR << "Inflation layer either not found or inflation is not set "
           << "sufficiently for optimized non-circular collision checking "
           << "capabilities. It is HIGHLY recommended to set the inflation "
           << "radius to be at MINIMUM half of the robot's largest "
           << "cross-section.";
  }

  if (costmap_ros_->getUseRadius() == consider_footprint_) {
    AWARN << "Inconsistent configuration in collision checking. Please "
          << "verify the robot's shape settings in both the costmap and "
          << "the obstacle critic.";
    if (costmap_ros_->getUseRadius()) {
      throw common::ControllerException(
          "Considering footprint in collision checking but no robot "
          "footprint provided in the costmap.");
    }
  }

  AINFO << "ObstaclesCritic instantiated with " << power_ << " power and " << critical_weight_ << " / "
        << repulsion_weight_ << " weights. "
        << "Critic will collision check based on " << (consider_footprint_ ? "footprint" : "circular") << " cost.";
}

float ObstaclesCritic::findCircumscribedCost(std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = map::costmap_2d::InflationLayer::getInflationLayer(costmap, inflation_layer_name_);
  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
    inflation_scale_factor_ = static_cast<float>(inflation_layer->getCostScalingFactor());
    inflation_radius_ = static_cast<float>(inflation_layer->getInflationRadius());
  } else {
    AWARN << "No inflation layer found in costmap configuration. "
          << "If this is an SE2-collision checking plugin, it cannot use "
          << "costmap potential field to speed up collision checking by "
          << "only checking the full footprint when robot is within "
          << "possibly-inscribed radius of an obstacle. This may "
          << "significantly slow down planning times and not avoid "
          << "anything but absolute collisions!";
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

float ObstaclesCritic::distanceToObstacle(const CollisionCost& cost) {
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point
  // cost and needs the radius subtracted to obtain the closest distance to
  // the object
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic
    // footprints
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  commsgs::geometry_msgs::Pose goal = tools::getCriticGoal(data, enforce_path_inversion_);

  // If near the goal, don't apply the preferential term since the goal is
  // near obstacles
  bool near_goal = false;
  if (tools::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, goal)) {
    near_goal = true;
  }

  Eigen::ArrayXf raw_cost = Eigen::ArrayXf::Zero(data.costs.size());
  Eigen::ArrayXf repulsive_cost = Eigen::ArrayXf::Zero(data.costs.size());

  const unsigned int traj_len = data.trajectories.x.cols();
  const unsigned int batch_size = data.trajectories.x.rows();
  bool all_trajectories_collide = true;

  for (unsigned int i = 0; i != batch_size; i++) {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;
    const auto& traj = data.trajectories;
    CollisionCost pose_cost;
    raw_cost(i) = 0.0f;
    repulsive_cost(i) = 0.0f;

    for (unsigned int j = 0; j != traj_len; j++) {
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
      if (pose_cost.cost < 1.0f) {
        continue;
      }  // In free space

      if (inCollision(pose_cost.cost)) {
        trajectory_collide = true;
        break;
      }

      // Cannot process repulsion if inflation layer does not exist
      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
        continue;
      }

      const float dist_to_obj = distanceToObstacle(pose_cost);

      // Let near-collision trajectory points be punished severely
      if (dist_to_obj < collision_margin_distance_) {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      }

      // Generally prefer trajectories further from obstacles
      if (!near_goal) {
        repulsive_cost[i] += inflation_radius_ - dist_to_obj;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    }
    raw_cost(i) = trajectory_collide ? collision_cost_ : traj_cost;
  }

  // Normalize repulsive cost by trajectory length & lowest score to not
  // overweight importance This is a preferential cost, not collision cost, to
  // be tuned relative to desired behaviors
  auto repulsive_cost_normalized = (repulsive_cost - repulsive_cost.minCoeff()) / traj_len;

  if (power_ > 1u) {
    data.costs += ((critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized)).pow(power_);
  } else {
    data.costs += (critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized);
  }

  data.fail_flag = all_trajectories_collide;
}

/**
 * @brief Checks if cost represents a collision
 * @param cost Costmap cost
 * @return bool if in collision
 */
bool ObstaclesCritic::inCollision(float cost) const {
  bool is_tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  switch (static_cast<unsigned char>(cost)) {
    case (map::costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (map::costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta) {
  CollisionCost collision_cost;
  float& cost = collision_cost.cost;
  collision_cost.using_footprint = false;
  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    cost = map::costmap_2d::NO_INFORMATION;
    return collision_cost;
  }
  cost = collision_checker_.pointCost(x_i, y_i);

  if (consider_footprint_ && (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f)) {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(x, y, theta, costmap_ros_->getRobotFootprint()));
    collision_cost.using_footprint = true;
  }

  return collision_cost;
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::mppi_controller::critics::ObstaclesCritic,
                            autonomy::control::controller::mppi_controller::CriticFunction)