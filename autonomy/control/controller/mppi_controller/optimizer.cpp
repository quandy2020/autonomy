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

#include "autonomy/control/controller/mppi_controller/optimizer.hpp"

#include <algorithm>

#include "autonomy/map/costmap_2d/filters/filter_values.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {

void Optimizer::initialize(std::shared_ptr<autolink::Node> parent, const std::string& name,
                           std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_ros,
                           const proto::MPPIControllerOptions* options,
                           double controller_frequency) {
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  costmap_ = costmap_ros_->getCostmap();
  options_ = options;

  getParams();

  critic_manager_.configure(parent_, name_, costmap_ros_, options_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, options_);

  const double freq = controller_frequency > 0.0 ? controller_frequency : 20.0;
  setOffset(freq);

  reset();
}

void Optimizer::shutdown() { noise_generator_.shutdown(); }

void Optimizer::getParams() {
  if (!options_) {
    AWARN << "Options not set, using defaults";
    // 为避免后续使用未初始化参数导致崩溃，这里设置一组安全的默认值
    auto& s = settings_;
    s.model_dt = 0.05;
    s.time_steps = 56;
    s.batch_size = 2000;
    s.iteration_count = 1;
    s.temperature = 0.3;
    s.gamma = 0.015;
    s.base_constraints.vx_max = 0.5f;
    s.base_constraints.vx_min = -0.35f;
    s.base_constraints.vy = 0.5f;
    s.base_constraints.wz = 1.9f;
    s.sampling_std.vx = 0.2f;
    s.sampling_std.vy = 0.2f;
    s.sampling_std.wz = 0.4f;
    s.base_constraints.ax_max = 3.0f;
    s.base_constraints.ax_min = -3.0f;
    s.base_constraints.ay_max = 3.0f;
    s.base_constraints.ay_min = -3.0f;
    s.base_constraints.az_max = 3.5f;
    s.retry_attempt_limit = 1;
    s.open_loop = false;
    s.clamp_raw_controls = false;
    s.model_delay_vx = 0.0f;
    s.model_delay_vy = 0.0f;
    s.model_delay_wz = 0.0f;
    s.sgf_order = 2;
    setMotionModel("DiffDrive");
    setOffset(20.0);
    return;
  }

  auto& s = settings_;

  // Load from proto options
  // 如果 proto 中未设置（为 0），则回退到一组与 nav2_mppi_controller 类似的默认值，
  // 以保证示例应用即使未加载 Lua 配置也能安全运行。
  s.model_dt = options_->model_dt() > 0.0 ? options_->model_dt() : 0.05;
  s.time_steps = options_->time_steps() > 0 ? options_->time_steps() : 56;
  s.batch_size = options_->batch_size() > 0 ? options_->batch_size() : 2000;
  s.iteration_count = options_->iteration_count() > 0 ? options_->iteration_count() : 1;
  s.temperature = options_->temperature() > 0.0 ? options_->temperature() : 0.3;
  s.gamma = options_->gamma() > 0.0 ? options_->gamma() : 0.015;
  s.base_constraints.vx_max = options_->vx_max() != 0.0 ? options_->vx_max() : 0.5f;
  s.base_constraints.vx_min = options_->vx_min() != 0.0 ? options_->vx_min() : -0.35f;
  s.base_constraints.vy = options_->vy_max() != 0.0 ? options_->vy_max() : 0.5f;
  s.base_constraints.wz = options_->wz_max() != 0.0 ? options_->wz_max() : 1.9f;
  s.sampling_std.vx = options_->vx_std() != 0.0 ? options_->vx_std() : 0.2f;
  s.sampling_std.vy = options_->vy_std() != 0.0 ? options_->vy_std() : 0.2f;
  s.sampling_std.wz = options_->wz_std() != 0.0 ? options_->wz_std() : 0.4f;

  s.base_constraints.ax_max =
      options_->ax_max() != 0.0 ? options_->ax_max() : 3.0f;
  s.base_constraints.ax_min =
      options_->ax_min() != 0.0 ? options_->ax_min() : -3.0f;
  s.base_constraints.ay_max =
      options_->ay_max() != 0.0 ? options_->ay_max() : 3.0f;
  s.base_constraints.ay_min =
      options_->ay_min() != 0.0 ? options_->ay_min() : -3.0f;
  s.base_constraints.az_max =
      options_->az_max() != 0.0 ? options_->az_max() : 3.5f;

  if (options_->retry_attempt_limit() > 0) {
    s.retry_attempt_limit = static_cast<size_t>(options_->retry_attempt_limit());
  } else {
    s.retry_attempt_limit = 1;
  }
  s.open_loop = options_->open_loop();
  s.clamp_raw_controls = options_->clamp_raw_controls();
  s.model_delay_vx = static_cast<float>(options_->model_delay_vx());
  s.model_delay_vy = static_cast<float>(options_->model_delay_vy());
  s.model_delay_wz = static_cast<float>(options_->model_delay_wz());

  if (options_->sgf_order() >= 1 && options_->sgf_order() <= 2) {
    s.sgf_order = options_->sgf_order();
  } else if (options_->sgf_order() != 0) {
    AWARN << "sgf_order must be 1 or 2, defaulting to 2";
    s.sgf_order = 2;
  } else {
    s.sgf_order = 2;
  }

  s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
  if (s.base_constraints.ax_min > 0.0) {
    s.base_constraints.ax_min = -1.0 * s.base_constraints.ax_min;
    AWARN << "Sign of the parameter ax_min is incorrect, consider "
          << "setting it negative.";
  }

  if (s.base_constraints.ay_min > 0.0) {
    s.base_constraints.ay_min = -1.0 * s.base_constraints.ay_min;
    AWARN << "Sign of the parameter ay_min is incorrect, consider "
          << "setting it negative.";
  }

  std::string motion_model_name = options_->motion_model();
  if (motion_model_name.empty()) {
    motion_model_name = "DiffDrive";
  }

  s.constraints = s.base_constraints;

  setMotionModel(motion_model_name);
}

void Optimizer::configureMotionModel() {
  if (!motion_model_) {
    return;
  }
  motion_model_->setConstraints(settings_.constraints, settings_.model_dt,
                               settings_.model_delay_vx, settings_.model_delay_vy,
                               settings_.model_delay_wz, settings_.clamp_raw_controls);
}

void Optimizer::setOffset(double controller_frequency) {
  const double controller_period = 1.0 / controller_frequency;
  settings_.controller_period = static_cast<float>(controller_period);
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    AWARN << "Controller period is less then model dt, consider setting "
          << "it equal";
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    AINFO << "Controller period is equal to model dt. Control sequence "
          << "shifting is ON";
    settings_.shift_control_sequence = true;
  } else {
    throw common::ControllerException("Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset(bool reset_dynamic_speed_limits) {
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0f, 0.0f, 0.0f};
  control_history_[1] = {0.0f, 0.0f, 0.0f};
  control_history_[2] = {0.0f, 0.0f, 0.0f};
  control_history_[3] = {0.0f, 0.0f, 0.0f};
  last_command_vel_ = automsgs::msgs::geometry_msgs::Twist();

  if (reset_dynamic_speed_limits) {
    settings_.constraints = settings_.base_constraints;
  }

  costs_.setZero(settings_.batch_size);
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  motion_model_->clearCommandHistory();
  configureMotionModel();

  AINFO << "Optimizer reset";
}

bool Optimizer::isHolonomic() const { return motion_model_->isHolonomic(); }

automsgs::msgs::geometry_msgs::TwistStamped Optimizer::evalControl(const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
                                                            const automsgs::msgs::geometry_msgs::Twist& robot_speed,
                                                            const automsgs::msgs::nav_msgs::Path& plan,
                                                            const automsgs::msgs::geometry_msgs::Pose& goal,
                                                            common::GoalChecker* goal_checker) {
  prepare(robot_pose, robot_speed, plan, goal, goal_checker);

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  tools::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  auto control = getControlFromSequenceAsTwist(plan.header().stamp());

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

void Optimizer::optimize() {
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
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
    throw common::NoValidControl("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
                        const automsgs::msgs::geometry_msgs::Twist& robot_speed, const automsgs::msgs::nav_msgs::Path& plan,
                        const automsgs::msgs::geometry_msgs::Pose& goal, common::GoalChecker* goal_checker) {
  if (settings_.open_loop) {
    state_.speed = last_command_vel_;
  } else {
    const auto& c = settings_.constraints;
    const double dt = settings_.controller_period;
    state_.speed = robot_speed;
    state_.speed.mutable_linear()->set_x(std::clamp(
        static_cast<double>(last_command_vel_.linear().x()),
        robot_speed.linear().x() + dt * static_cast<double>(c.ax_min),
        robot_speed.linear().x() + dt * static_cast<double>(c.ax_max)));
    state_.speed.mutable_angular()->set_z(std::clamp(
        static_cast<double>(last_command_vel_.angular().z()),
        robot_speed.angular().z() - dt * static_cast<double>(c.az_max),
        robot_speed.angular().z() + dt * static_cast<double>(c.az_max)));
    if (isHolonomic()) {
      state_.speed.mutable_linear()->set_y(std::clamp(
          static_cast<double>(last_command_vel_.linear().y()),
          robot_speed.linear().y() + dt * static_cast<double>(c.ay_min),
          robot_speed.linear().y() + dt * static_cast<double>(c.ay_max)));
    }
  }

  state_.pose = robot_pose;
  path_ = tools::toTensor(plan);
  costs_.setZero();
  goal_ = goal;

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence() {
  auto size = control_sequence_.vx.size();
  tools::shiftColumnsByOnePlace(control_sequence_.vx, -1);
  tools::shiftColumnsByOnePlace(control_sequence_.wz, -1);
  control_sequence_.vx(size - 1) = control_sequence_.vx(size - 2);
  control_sequence_.wz(size - 1) = control_sequence_.wz(size - 2);

  if (isHolonomic()) {
    tools::shiftColumnsByOnePlace(control_sequence_.vy, -1);
    control_sequence_.vy(size - 1) = control_sequence_.vy(size - 2);
  }
}

void Optimizer::generateNoisedTrajectories() {
  applyControlSequenceInterIterationConstraints();
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceInterIterationConstraints() {
  auto& s = settings_;
  const float first_dt = s.controller_period;
  const float max_delta_vx = first_dt * s.constraints.ax_max;
  const float min_delta_vx = first_dt * s.constraints.ax_min;
  const float max_delta_vy = first_dt * s.constraints.ay_max;
  const float min_delta_vy = first_dt * s.constraints.ay_min;
  const float max_delta_wz = first_dt * s.constraints.az_max;

  const float speed_vx = static_cast<float>(state_.speed.linear().x());
  const float speed_wz = static_cast<float>(state_.speed.angular().z());
  if (s.shift_control_sequence) {
    control_sequence_.vx(0) = speed_vx;
    control_sequence_.wz(0) = speed_wz;
    if (isHolonomic()) {
      control_sequence_.vy(0) = static_cast<float>(state_.speed.linear().y());
    }
  } else {
    control_sequence_.vx(0) = tools::clampVelocityByAccel(
        speed_vx, control_sequence_.vx(0), min_delta_vx, max_delta_vx);
    control_sequence_.wz(0) = tools::clampVelocityByAccel(
        speed_wz, control_sequence_.wz(0), -max_delta_wz, max_delta_wz);
    if (isHolonomic()) {
      const float speed_vy = static_cast<float>(state_.speed.linear().y());
      control_sequence_.vy(0) = tools::clampVelocityByAccel(
          speed_vy, control_sequence_.vy(0), min_delta_vy, max_delta_vy);
    }
  }
}

void Optimizer::applyControlSequenceConstraints() {
  auto& s = settings_;

  motion_model_->applyConstraints(control_sequence_);

  float max_delta_vx = s.controller_period * s.constraints.ax_max;
  float min_delta_vx = s.controller_period * s.constraints.ax_min;
  float max_delta_vy = s.controller_period * s.constraints.ay_max;
  float min_delta_vy = s.controller_period * s.constraints.ay_min;
  float max_delta_wz = s.controller_period * s.constraints.az_max;

  float vx_last = static_cast<float>(state_.speed.linear().x());
  float wz_last = static_cast<float>(state_.speed.angular().z());
  float vy_last = isHolonomic() ? static_cast<float>(state_.speed.linear().y()) : 0.0f;

  if (s.shift_control_sequence) {
    control_sequence_.vx(0) = vx_last;
    control_sequence_.wz(0) = wz_last;
    if (isHolonomic()) {
      control_sequence_.vy(0) = vy_last;
    }
  }

  for (unsigned int i = 0; i != control_sequence_.vx.size(); i++) {
    if (i == 1) {
      max_delta_vx = s.model_dt * s.constraints.ax_max;
      min_delta_vx = s.model_dt * s.constraints.ax_min;
      max_delta_vy = s.model_dt * s.constraints.ay_max;
      min_delta_vy = s.model_dt * s.constraints.ay_min;
      max_delta_wz = s.model_dt * s.constraints.az_max;
    }

    float& vx_curr = control_sequence_.vx(i);
    vx_curr = tools::clamp(s.constraints.vx_min, s.constraints.vx_max, vx_curr);
    vx_curr = tools::clampVelocityByAccel(
        vx_last, vx_curr, min_delta_vx, max_delta_vx);
    vx_last = vx_curr;

    float& wz_curr = control_sequence_.wz(i);
    wz_curr = tools::clamp(-s.constraints.wz, s.constraints.wz, wz_curr);
    wz_curr = tools::clampVelocityByAccel(
        wz_last, wz_curr, -max_delta_wz, max_delta_wz);
    wz_last = wz_curr;

    if (isHolonomic()) {
      float& vy_curr = control_sequence_.vy(i);
      vy_curr = tools::clamp(-s.constraints.vy, s.constraints.vy, vy_curr);
      vy_curr = tools::clampVelocityByAccel(
          vy_last, vy_curr, min_delta_vy, max_delta_vy);
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
  state.vx.col(0) = static_cast<float>(state.speed.linear().x());
  state.wz.col(0) = static_cast<float>(state.speed.angular().z());

  if (isHolonomic()) {
    state.vy.col(0) = static_cast<float>(state.speed.linear().y());
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(models::State& state) const { motion_model_->predict(state); }

void Optimizer::integrateStateVelocities(Eigen::Array<float, Eigen::Dynamic, 3>& trajectory,
                                         const Eigen::ArrayXXf& sequence) const {
  float initial_yaw = static_cast<float>(autonomy::transform::tf2::getYaw(state_.pose.pose().orientation()));

  const auto vx = sequence.col(0);
  const auto wz = sequence.col(1);

  auto traj_x = trajectory.col(0);
  auto traj_y = trajectory.col(1);
  auto traj_yaws = trajectory.col(2);

  const size_t n_size = traj_yaws.size();

  float last_yaw = initial_yaw;
  for (size_t i = 0; i != n_size; i++) {
    last_yaw += wz(i) * settings_.model_dt;
    traj_yaws(i) = last_yaw;
  }

  Eigen::ArrayXf yaw_cos = traj_yaws.cos();
  Eigen::ArrayXf yaw_sin = traj_yaws.sin();
  tools::shiftColumnsByOnePlace(yaw_cos, 1);
  tools::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos(0) = cosf(initial_yaw);
  yaw_sin(0) = sinf(initial_yaw);

  auto dx = (vx * yaw_cos).eval();
  auto dy = (vx * yaw_sin).eval();

  if (isHolonomic()) {
    auto vy = sequence.col(2);
    dx = (dx - vy * yaw_sin).eval();
    dy = (dy + vy * yaw_cos).eval();
  }

  float last_x = state_.pose.pose().position().x();
  float last_y = state_.pose.pose().position().y();
  for (size_t i = 0; i != n_size; i++) {
    last_x += dx(i) * settings_.model_dt;
    last_y += dy(i) * settings_.model_dt;
    traj_x(i) = last_x;
    traj_y(i) = last_y;
  }
}

void Optimizer::integrateStateVelocities(models::Trajectories& trajectories, const models::State& state) const {
  auto initial_yaw = static_cast<float>(autonomy::transform::tf2::getYaw(state.pose.pose().orientation()));
  const size_t n_cols = trajectories.yaws.cols();

  Eigen::ArrayXf last_yaws = Eigen::ArrayXf::Constant(trajectories.yaws.rows(), initial_yaw);
  for (size_t i = 0; i != n_cols; i++) {
    last_yaws += state.wz.col(i) * settings_.model_dt;
    trajectories.yaws.col(i) = last_yaws;
  }

  Eigen::ArrayXXf yaw_cos = trajectories.yaws.cos();
  Eigen::ArrayXXf yaw_sin = trajectories.yaws.sin();
  tools::shiftColumnsByOnePlace(yaw_cos, 1);
  tools::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos.col(0) = cosf(initial_yaw);
  yaw_sin.col(0) = sinf(initial_yaw);

  auto dx = (state.vx * yaw_cos).eval();
  auto dy = (state.vx * yaw_sin).eval();

  if (isHolonomic()) {
    dx -= state.vy * yaw_sin;
    dy += state.vy * yaw_cos;
  }

  Eigen::ArrayXf last_x = Eigen::ArrayXf::Constant(trajectories.x.rows(), state.pose.pose().position().x());
  Eigen::ArrayXf last_y = Eigen::ArrayXf::Constant(trajectories.y.rows(), state.pose.pose().position().y());

  for (size_t i = 0; i != n_cols; i++) {
    last_x += dx.col(i) * settings_.model_dt;
    last_y += dy.col(i) * settings_.model_dt;
    trajectories.x.col(i) = last_x;
    trajectories.y.col(i) = last_y;
  }
}

Eigen::ArrayXXf Optimizer::getOptimizedTrajectory() {
  const bool is_holo = isHolonomic();
  Eigen::ArrayXXf sequence = Eigen::ArrayXXf(settings_.time_steps, is_holo ? 3 : 2);
  Eigen::Array<float, Eigen::Dynamic, 3> trajectories = Eigen::Array<float, Eigen::Dynamic, 3>(settings_.time_steps, 3);

  sequence.col(0) = control_sequence_.vx;
  sequence.col(1) = control_sequence_.wz;

  if (is_holo) {
    sequence.col(2) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return trajectories;
}

const models::ControlSequence& Optimizer::getOptimalControlSequence() { return control_sequence_; }

void Optimizer::updateControlSequence() {
  const bool is_holo = isHolonomic();
  auto& s = settings_;

  auto vx_T = control_sequence_.vx.transpose();
  auto bounded_noises_vx = state_.cvx.rowwise() - vx_T;
  const float gamma_vx = s.gamma / (s.sampling_std.vx * s.sampling_std.vx);
  costs_ += (gamma_vx * (bounded_noises_vx.rowwise() * vx_T).rowwise().sum()).eval();

  if (s.sampling_std.wz > 0.0f) {
    auto wz_T = control_sequence_.wz.transpose();
    auto bounded_noises_wz = state_.cwz.rowwise() - wz_T;
    const float gamma_wz = s.gamma / (s.sampling_std.wz * s.sampling_std.wz);
    costs_ += (gamma_wz * (bounded_noises_wz.rowwise() * wz_T).rowwise().sum()).eval();
  }

  if (is_holo) {
    auto vy_T = control_sequence_.vy.transpose();
    auto bounded_noises_vy = state_.cvy.rowwise() - vy_T;
    const float gamma_vy = s.gamma / (s.sampling_std.vy * s.sampling_std.vy);
    costs_ += (gamma_vy * (bounded_noises_vy.rowwise() * vy_T).rowwise().sum()).eval();
  }

  auto costs_normalized = costs_ - costs_.minCoeff();
  const float inv_temp = 1.0f / s.temperature;
  auto softmaxes = (-inv_temp * costs_normalized).exp().eval();
  softmaxes /= softmaxes.sum();

  auto softmax_mat = softmaxes.matrix();
  control_sequence_.vx = state_.cvx.transpose().matrix() * softmax_mat;
  control_sequence_.wz = state_.cwz.transpose().matrix() * softmax_mat;

  if (is_holo) {
    control_sequence_.vy = state_.cvy.transpose().matrix() * softmax_mat;
  }

  applyControlSequenceConstraints();
}

automsgs::msgs::geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    const automsgs::msgs::builtin_interfaces::Time& stamp) {
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  last_command_vel_.mutable_linear()->set_x(vx);
  last_command_vel_.mutable_angular()->set_z(wz);

  float vy = 0.0f;
  if (isHolonomic()) {
    vy = control_sequence_.vy(offset);
    last_command_vel_.mutable_linear()->set_y(vy);
  } else {
    last_command_vel_.mutable_linear()->set_y(0.0);
  }

  motion_model_->pushCommandHistory(vx, vy, wz);

  if (isHolonomic()) {
    return tools::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return tools::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string& model) {
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(options_, name_);
  } else {
    throw common::ControllerException(std::string("Model " + model +
                                                  " is not valid! Valid options are DiffDrive, Omni, "
                                                  "or Ackermann"));
  }
  configureMotionModel();
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage) {
  auto& s = settings_;
  if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
  configureMotionModel();
}

models::Trajectories& Optimizer::getGeneratedTrajectories() { return generated_trajectories_; }
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy