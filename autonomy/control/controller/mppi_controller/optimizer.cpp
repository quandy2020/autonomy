/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/mppi_controller/optimizer.hpp"

#include <cmath>
#include <stdexcept>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {

void Optimizer::initialize(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
    double controller_frequency) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    costmap_ = costmap_wrapper_->getCostmap();

    auto& s = settings_;
    s.model_dt = options.model_dt() > 0.0 ? static_cast<float>(options.model_dt())
                                          : 0.05f;
    s.time_steps = options.time_steps() > 0
                       ? static_cast<unsigned int>(options.time_steps())
                       : 56u;
    s.batch_size = options.batch_size() > 0
                       ? static_cast<unsigned int>(options.batch_size())
                       : 1000u;
    s.iteration_count = options.iteration_count() > 0
                            ? static_cast<unsigned int>(options.iteration_count())
                            : 1u;
    s.temperature = options.temperature() > 0.0
                        ? static_cast<float>(options.temperature())
                        : 0.3f;
    s.gamma =
        options.gamma() > 0.0 ? static_cast<float>(options.gamma()) : 0.015f;
    s.base_constraints.vx_max = static_cast<float>(options.vx_max());
    s.base_constraints.vx_min = static_cast<float>(options.vx_min());
    s.base_constraints.vy = static_cast<float>(options.vy_max());
    s.base_constraints.wz = static_cast<float>(options.wz_max());
    s.base_constraints.ax_max = 3.0f;
    s.base_constraints.ax_min = -3.0f;
    s.base_constraints.ay_max = 3.0f;
    s.base_constraints.ay_min = -3.0f;
    s.base_constraints.az_max = 3.5f;
    s.sampling_std.vx = static_cast<float>(options.vx_std());
    s.sampling_std.vy = static_cast<float>(options.vy_std());
    s.sampling_std.wz = static_cast<float>(options.wz_std());
    s.retry_attempt_limit = 1u;
    s.constraints = s.base_constraints;

    const std::string motion_model_name =
        options.motion_model().empty() ? "DiffDrive" : options.motion_model();
    setMotionModel(motion_model_name, options);
    setOffset(controller_frequency);

    critic_manager_.configure(options, costmap_wrapper_);
    noise_generator_.initialize(settings_, isHolonomic());

    critics_data_ = std::make_unique<CriticData>(CriticData{
        state_, generated_trajectories_, path_, goal_, costs_,
        settings_.model_dt, false, nullptr, motion_model_, std::nullopt,
        std::nullopt});

    reset();
}

void Optimizer::shutdown() {
    noise_generator_.shutdown();
}

void Optimizer::setOffset(double controller_frequency) {
    if (controller_frequency <= 0.0) {
        settings_.shift_control_sequence = false;
        return;
    }
    const double controller_period = 1.0 / controller_frequency;
    constexpr double eps = 1e-6;
    if ((controller_period + eps) < settings_.model_dt) {
        AWARN << "MPPI: controller period < model_dt; control sequence shifting "
                 "disabled";
        settings_.shift_control_sequence = false;
    } else if (std::abs(controller_period - settings_.model_dt) < eps) {
        settings_.shift_control_sequence = true;
    } else {
        throw common::ControllerException(
            "MPPI: controller period must equal model_dt for sequence shifting");
    }
}

void Optimizer::reset(bool reset_dynamic_speed_limits) {
    state_.reset(settings_.batch_size, settings_.time_steps);
    control_sequence_.reset(settings_.time_steps);
    control_history_ = {};
    if (reset_dynamic_speed_limits) {
        settings_.constraints = settings_.base_constraints;
    }
    costs_.setZero(static_cast<Eigen::Index>(settings_.batch_size));
    generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
    noise_generator_.reset(settings_, isHolonomic());
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
}

bool Optimizer::isHolonomic() const {
    return motion_model_->isHolonomic();
}

commsgs::geometry_msgs::TwistStamped Optimizer::evalControl(
    const commsgs::geometry_msgs::PoseStamped& robot_pose,
    const commsgs::geometry_msgs::Twist& robot_speed,
    const commsgs::planning_msgs::Path& plan,
    const commsgs::geometry_msgs::Pose& goal,
    common::GoalChecker* goal_checker) {
    prepare(robot_pose, robot_speed, plan, goal, goal_checker);

    do {
        optimize();
    } while (fallback(critics_data_->fail_flag));

    utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    auto control = getControlFromSequenceAsTwist(plan.header.stamp);

    if (settings_.shift_control_sequence) {
        shiftControlSequence();
    }
    return control;
}

void Optimizer::optimize() {
    for (unsigned int i = 0; i < settings_.iteration_count; ++i) {
        (void)i;
        generateNoisedTrajectories();
        critic_manager_.evalTrajectoriesScores(*critics_data_);
        updateControlSequence();
    }
}

bool Optimizer::fallback(bool fail) {
    static size_t counter = 0;
    if (!fail) {
        counter = 0;
        return false;
    }
    reset();
    if (++counter > settings_.retry_attempt_limit) {
        counter = 0;
        throw common::NoValidControl("MPPI optimizer failed to find valid control");
    }
    return true;
}

void Optimizer::prepare(const commsgs::geometry_msgs::PoseStamped& robot_pose,
                        const commsgs::geometry_msgs::Twist& robot_speed,
                        const commsgs::planning_msgs::Path& plan,
                        const commsgs::geometry_msgs::Pose& goal,
                        common::GoalChecker* goal_checker) {
    state_.pose = robot_pose;
    state_.speed = robot_speed;
    path_ = utils::toTensor(plan);
    costs_.setZero();
    goal_ = goal;
    critics_data_->model_dt = settings_.model_dt;
    critics_data_->fail_flag = false;
    critics_data_->goal_checker = goal_checker;
    critics_data_->motion_model = motion_model_;
    critics_data_->furthest_reached_path_point.reset();
    critics_data_->path_pts_valid.reset();
}

void Optimizer::shiftControlSequence() {
    const auto size = control_sequence_.vx.size();
    utils::shiftVectorByOnePlace(control_sequence_.vx, -1);
    utils::shiftVectorByOnePlace(control_sequence_.wz, -1);
    control_sequence_.vx(size - 1) = control_sequence_.vx(size - 2);
    control_sequence_.wz(size - 1) = control_sequence_.wz(size - 2);
    if (isHolonomic()) {
        utils::shiftVectorByOnePlace(control_sequence_.vy, -1);
        control_sequence_.vy(size - 1) = control_sequence_.vy(size - 2);
    }
}

void Optimizer::generateNoisedTrajectories() {
    noise_generator_.setNoisedControls(state_, control_sequence_);
    noise_generator_.generateNextNoises();
    updateStateVelocities(state_);
    integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceConstraints() {
    auto& s = settings_;
    const float max_delta_vx = s.model_dt * s.constraints.ax_max;
    const float min_delta_vx = s.model_dt * s.constraints.ax_min;
    const float max_delta_vy = s.model_dt * s.constraints.ay_max;
    const float min_delta_vy = s.model_dt * s.constraints.ay_min;
    const float max_delta_wz = s.model_dt * s.constraints.az_max;

    float vx_last = utils::clamp(s.constraints.vx_min, s.constraints.vx_max,
                                 control_sequence_.vx(0));
    float wz_last =
        utils::clamp(-s.constraints.wz, s.constraints.wz, control_sequence_.wz(0));
    control_sequence_.vx(0) = vx_last;
    control_sequence_.wz(0) = wz_last;
    float vy_last = 0.0f;
    if (isHolonomic()) {
        vy_last = utils::clamp(-s.constraints.vy, s.constraints.vy,
                               control_sequence_.vy(0));
        control_sequence_.vy(0) = vy_last;
    }

    for (unsigned int i = 1; i != static_cast<unsigned int>(control_sequence_.vx.size());
         ++i) {
        float& vx_curr = control_sequence_.vx(static_cast<Eigen::Index>(i));
        vx_curr = utils::clamp(s.constraints.vx_min, s.constraints.vx_max, vx_curr);
        if (vx_last > 0) {
            vx_curr = utils::clamp(vx_last + min_delta_vx, vx_last + max_delta_vx,
                                   vx_curr);
        } else {
            vx_curr = utils::clamp(vx_last - max_delta_vx, vx_last - min_delta_vx,
                                   vx_curr);
        }
        vx_last = vx_curr;

        float& wz_curr = control_sequence_.wz(static_cast<Eigen::Index>(i));
        wz_curr = utils::clamp(-s.constraints.wz, s.constraints.wz, wz_curr);
        wz_curr = utils::clamp(wz_last - max_delta_wz, wz_last + max_delta_wz, wz_curr);
        wz_last = wz_curr;

        if (isHolonomic()) {
            float& vy_curr = control_sequence_.vy(static_cast<Eigen::Index>(i));
            vy_curr = utils::clamp(-s.constraints.vy, s.constraints.vy, vy_curr);
            if (vy_last > 0) {
                vy_curr = utils::clamp(vy_last + min_delta_vy, vy_last + max_delta_vy,
                                       vy_curr);
            } else {
                vy_curr = utils::clamp(vy_last - max_delta_vy, vy_last - min_delta_vy,
                                       vy_curr);
            }
            vy_last = vy_curr;
        }
    }
    motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(models::State& state) const {
    updateInitialStateVelocities(state);
    propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(models::State& state) const {
    state.vx.col(0) = state.speed.linear.x;
    state.wz.col(0) = state.speed.angular.z;
    if (isHolonomic()) {
        state.vy.col(0) = state.speed.linear.y;
    }
}

void Optimizer::propagateStateVelocitiesFromInitials(models::State& state) const {
    motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
    Eigen::Array<float, Eigen::Dynamic, 3>& trajectory,
    const Eigen::ArrayXXf& sequence) const {
    const float initial_yaw =
        static_cast<float>(transform::tf2::getYaw(state_.pose.pose.orientation));
    const auto vx = sequence.col(0);
    const auto wz = sequence.col(1);
    auto traj_x = trajectory.col(0);
    auto traj_y = trajectory.col(1);
    auto traj_yaws = trajectory.col(2);

    float last_yaw = initial_yaw;
    for (int i = 0; i < traj_yaws.size(); ++i) {
        last_yaw += wz(i) * settings_.model_dt;
        traj_yaws(i) = last_yaw;
    }

    Eigen::ArrayXf yaw_cos = traj_yaws.cos();
    Eigen::ArrayXf yaw_sin = traj_yaws.sin();
    utils::shiftVectorByOnePlace(yaw_cos, 1);
    utils::shiftVectorByOnePlace(yaw_sin, 1);
    yaw_cos(0) = std::cos(initial_yaw);
    yaw_sin(0) = std::sin(initial_yaw);

    auto dx = (vx * yaw_cos).eval();
    auto dy = (vx * yaw_sin).eval();
    if (isHolonomic()) {
        auto vy = sequence.col(2);
        dx = (dx - vy * yaw_sin).eval();
        dy = (dy + vy * yaw_cos).eval();
    }

    float last_x = state_.pose.pose.position.x;
    float last_y = state_.pose.pose.position.y;
    for (int i = 0; i < traj_yaws.size(); ++i) {
        last_x += dx(i) * settings_.model_dt;
        last_y += dy(i) * settings_.model_dt;
        traj_x(i) = last_x;
        traj_y(i) = last_y;
    }
}

void Optimizer::integrateStateVelocities(models::Trajectories& trajectories,
                                         const models::State& state) const {
    const float initial_yaw =
        static_cast<float>(transform::tf2::getYaw(state.pose.pose.orientation));
    const size_t n_cols = static_cast<size_t>(trajectories.yaws.cols());

    Eigen::ArrayXf last_yaws =
        Eigen::ArrayXf::Constant(trajectories.yaws.rows(), initial_yaw);
    for (size_t i = 0; i < n_cols; ++i) {
        last_yaws += state.wz.col(static_cast<Eigen::Index>(i)) * settings_.model_dt;
        trajectories.yaws.col(static_cast<Eigen::Index>(i)) = last_yaws;
    }

    Eigen::ArrayXXf yaw_cos = trajectories.yaws.cos();
    Eigen::ArrayXXf yaw_sin = trajectories.yaws.sin();
    utils::shiftColumnsByOnePlace(yaw_cos, 1);
    utils::shiftColumnsByOnePlace(yaw_sin, 1);
    yaw_cos.col(0) = std::cos(initial_yaw);
    yaw_sin.col(0) = std::sin(initial_yaw);

    auto dx = (state.vx * yaw_cos).eval();
    auto dy = (state.vx * yaw_sin).eval();
    if (isHolonomic()) {
        dx -= state.vy * yaw_sin;
        dy += state.vy * yaw_cos;
    }

    Eigen::ArrayXf last_x = Eigen::ArrayXf::Constant(
        trajectories.x.rows(), state.pose.pose.position.x);
    Eigen::ArrayXf last_y = Eigen::ArrayXf::Constant(
        trajectories.y.rows(), state.pose.pose.position.y);

    for (size_t i = 0; i < n_cols; ++i) {
        last_x += dx.col(static_cast<Eigen::Index>(i)) * settings_.model_dt;
        last_y += dy.col(static_cast<Eigen::Index>(i)) * settings_.model_dt;
        trajectories.x.col(static_cast<Eigen::Index>(i)) = last_x;
        trajectories.y.col(static_cast<Eigen::Index>(i)) = last_y;
    }
}

Eigen::ArrayXXf Optimizer::getOptimizedTrajectory() {
    const bool is_holo = isHolonomic();
    Eigen::ArrayXXf sequence =
        Eigen::ArrayXXf(settings_.time_steps, is_holo ? 3 : 2);
    Eigen::Array<float, Eigen::Dynamic, 3> trajectories(settings_.time_steps, 3);
    sequence.col(0) = control_sequence_.vx;
    sequence.col(1) = control_sequence_.wz;
    if (is_holo) {
        sequence.col(2) = control_sequence_.vy;
    }
    integrateStateVelocities(trajectories, sequence);
    return trajectories;
}

void Optimizer::updateControlSequence() {
    const bool is_holo = isHolonomic();
    auto& s = settings_;

    auto vx_T = control_sequence_.vx.transpose();
    auto bounded_noises_vx = state_.cvx.rowwise() - vx_T;
    const float gamma_vx = s.gamma / (s.sampling_std.vx * s.sampling_std.vx);
    costs_ += (gamma_vx * (bounded_noises_vx.rowwise() * vx_T).rowwise().sum())
                 .eval();

    if (s.sampling_std.wz > 0.0f) {
        auto wz_T = control_sequence_.wz.transpose();
        auto bounded_noises_wz = state_.cwz.rowwise() - wz_T;
        const float gamma_wz = s.gamma / (s.sampling_std.wz * s.sampling_std.wz);
        costs_ += (gamma_wz * (bounded_noises_wz.rowwise() * wz_T).rowwise().sum())
                     .eval();
    }

    if (is_holo) {
        auto vy_T = control_sequence_.vy.transpose();
        auto bounded_noises_vy = state_.cvy.rowwise() - vy_T;
        const float gamma_vy = s.gamma / (s.sampling_std.vy * s.sampling_std.vy);
        costs_ += (gamma_vy * (bounded_noises_vy.rowwise() * vy_T).rowwise().sum())
                     .eval();
    }

    auto costs_normalized = costs_ - costs_.minCoeff();
    const float inv_temp = 1.0f / s.temperature;
    auto softmaxes = (-inv_temp * costs_normalized).exp().eval();
    softmaxes /= softmaxes.sum();

    const auto softmax_mat = softmaxes.matrix();
    control_sequence_.vx =
        state_.cvx.transpose().matrix() * softmax_mat;
    control_sequence_.wz =
        state_.cwz.transpose().matrix() * softmax_mat;
    if (is_holo) {
        control_sequence_.vy =
            state_.cvy.transpose().matrix() * softmax_mat;
    }
    applyControlSequenceConstraints();
}

commsgs::geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    const commsgs::builtin_interfaces::Time& stamp) const {
    const unsigned int offset = settings_.shift_control_sequence ? 1u : 0u;
    commsgs::geometry_msgs::TwistStamped cmd;
    cmd.header.frame_id = costmap_wrapper_->getBaseFrameID();
    cmd.header.stamp = stamp;
    cmd.twist.linear.x = control_sequence_.vx(static_cast<Eigen::Index>(offset));
    cmd.twist.angular.z = control_sequence_.wz(static_cast<Eigen::Index>(offset));
    if (isHolonomic()) {
        cmd.twist.linear.y = control_sequence_.vy(static_cast<Eigen::Index>(offset));
    }
    return cmd;
}

void Optimizer::setMotionModel(const std::string& model,
                               const proto::MPPIControllerOptions& options) {
    if (model == "DiffDrive") {
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
    } else if (model == "Omni") {
        motion_model_ = std::make_shared<OmniMotionModel>();
    } else if (model == "Ackermann") {
        const float min_r =
            options.has_ackermann_constraints() &&
                    options.ackermann_constraints().min_turning_r() > 0.0
                ? static_cast<float>(options.ackermann_constraints().min_turning_r())
                : 0.2f;
        motion_model_ = std::make_shared<AckermannMotionModel>(min_r);
    } else {
        throw common::ControllerException(
            "MPPI motion model '" + model +
            "' invalid; use DiffDrive, Omni, or Ackermann");
    }
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage) {
    auto& s = settings_;
    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
        s.constraints = s.base_constraints;
        return;
    }
    if (percentage) {
        const double ratio = speed_limit / 100.0;
        s.constraints.vx_max =
            static_cast<float>(s.base_constraints.vx_max * ratio);
        s.constraints.vx_min =
            static_cast<float>(s.base_constraints.vx_min * ratio);
        s.constraints.vy = static_cast<float>(s.base_constraints.vy * ratio);
        s.constraints.wz = static_cast<float>(s.base_constraints.wz * ratio);
    } else if (s.base_constraints.vx_max > 0.0f) {
        const double ratio = speed_limit / s.base_constraints.vx_max;
        s.constraints.vx_max =
            static_cast<float>(s.base_constraints.vx_max * ratio);
        s.constraints.vx_min =
            static_cast<float>(s.base_constraints.vx_min * ratio);
        s.constraints.vy = static_cast<float>(s.base_constraints.vy * ratio);
        s.constraints.wz = static_cast<float>(s.base_constraints.wz * ratio);
    }
}

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
