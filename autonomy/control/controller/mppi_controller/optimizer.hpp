/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/controller/mppi_controller/critic_data.hpp"
#include "autonomy/control/controller/mppi_controller/critic_manager.hpp"
#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/optimizer_settings.hpp"
#include "autonomy/control/controller/mppi_controller/models/path.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"
#include "autonomy/control/controller/mppi_controller/models/trajectories.hpp"
#include "autonomy/control/controller/mppi_controller/motion_models.hpp"
#include "autonomy/control/controller/mppi_controller/tools/noise_generator.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {

class Optimizer
{
public:
    void initialize(const proto::MPPIControllerOptions& options,
                    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
                    double controller_frequency);

    void shutdown();

    commsgs::geometry_msgs::TwistStamped evalControl(
        const commsgs::geometry_msgs::PoseStamped& robot_pose,
        const commsgs::geometry_msgs::Twist& robot_speed,
        const commsgs::planning_msgs::Path& plan,
        const commsgs::geometry_msgs::Pose& goal,
        common::GoalChecker* goal_checker);

    void setSpeedLimit(double speed_limit, bool percentage);

    void reset(bool reset_dynamic_speed_limits = true);

    const models::OptimizerSettings& getSettings() const { return settings_; }

    models::Trajectories& getGeneratedTrajectories() { return generated_trajectories_; }

    Eigen::ArrayXXf getOptimizedTrajectory();

    const models::ControlSequence& getOptimalControlSequence() const {
        return control_sequence_;
    }

protected:
    void optimize();
    void prepare(const commsgs::geometry_msgs::PoseStamped& robot_pose,
                 const commsgs::geometry_msgs::Twist& robot_speed,
                 const commsgs::planning_msgs::Path& plan,
                 const commsgs::geometry_msgs::Pose& goal,
                 common::GoalChecker* goal_checker);
    void setMotionModel(const std::string& model,
                        const proto::MPPIControllerOptions& options);
    void shiftControlSequence();
    void generateNoisedTrajectories();
    void applyControlSequenceConstraints();
    void updateStateVelocities(models::State& state) const;
    void updateInitialStateVelocities(models::State& state) const;
    void propagateStateVelocitiesFromInitials(models::State& state) const;
    void integrateStateVelocities(models::Trajectories& trajectories,
                                  const models::State& state) const;
    void integrateStateVelocities(Eigen::Array<float, Eigen::Dynamic, 3>& trajectories,
                                 const Eigen::ArrayXXf& sequence) const;
    void updateControlSequence();
    commsgs::geometry_msgs::TwistStamped getControlFromSequenceAsTwist(
        const commsgs::builtin_interfaces::Time& stamp) const;
    bool isHolonomic() const;
    void setOffset(double controller_frequency);
    bool fallback(bool fail);

    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    map::costmap_2d::Costmap2D* costmap_{nullptr};

    std::shared_ptr<MotionModel> motion_model_;
    CriticManager critic_manager_;
    NoiseGenerator noise_generator_;

    models::OptimizerSettings settings_;
    models::State state_;
    models::ControlSequence control_sequence_;
    std::array<models::Control, 4> control_history_;
    models::Trajectories generated_trajectories_;
    models::Path path_;
    commsgs::geometry_msgs::Pose goal_;
    Eigen::ArrayXf costs_;

    std::unique_ptr<CriticData> critics_data_;
};

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
