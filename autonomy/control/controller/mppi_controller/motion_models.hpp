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

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/mppi_controller/models/constraints.hpp"
#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel {
 public:
  MotionModel() = default;
  virtual ~MotionModel() = default;

  void setConstraints(const models::ControlConstraints& control_constraints,
                      float model_dt, float model_delay_vx,
                      float model_delay_vy, float model_delay_wz,
                      bool clamp_raw_controls) {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
    model_delay_vx_ = model_delay_vx;
    model_delay_vy_ = model_delay_vy;
    model_delay_wz_ = model_delay_wz;
    clamp_raw_controls_ = clamp_raw_controls;

    cmd_history_vx_.resize(offsetSteps(model_delay_vx_), 0.0f);
    cmd_history_vy_.resize(offsetSteps(model_delay_vy_), 0.0f);
    cmd_history_wz_.resize(offsetSteps(model_delay_wz_), 0.0f);
  }

  void pushCommandHistory(float vx, float vy, float wz) {
    pushOne(cmd_history_vx_, vx);
    pushOne(cmd_history_vy_, vy);
    pushOne(cmd_history_wz_, wz);
  }

  void clearCommandHistory() {
    std::fill(cmd_history_vx_.begin(), cmd_history_vx_.end(), 0.0f);
    std::fill(cmd_history_vy_.begin(), cmd_history_vy_.end(), 0.0f);
    std::fill(cmd_history_wz_.begin(), cmd_history_wz_.end(), 0.0f);
  }

  virtual void predict(models::State& state) {
    const bool is_holo = isHolonomic();
    const float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    const float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    const float min_delta_vy = model_dt_ * control_constraints_.ay_min;
    const float max_delta_wz = model_dt_ * control_constraints_.az_max;
    const unsigned int n_cols = state.vx.cols();

    for (unsigned int i = 1; i < n_cols; i++) {
      const auto lower_bound_vx =
          (state.vx.col(i - 1) > 0)
              .select(state.vx.col(i - 1) + min_delta_vx,
                      state.vx.col(i - 1) - max_delta_vx);
      const auto upper_bound_vx =
          (state.vx.col(i - 1) > 0)
              .select(state.vx.col(i - 1) + max_delta_vx,
                      state.vx.col(i - 1) - min_delta_vx);
      state.vx.col(i) = state.cvx.col(i - 1)
                            .cwiseMax(lower_bound_vx)
                            .cwiseMin(upper_bound_vx);
      if (clamp_raw_controls_) {
        state.cvx.col(i - 1) = state.vx.col(i);
      }

      state.wz.col(i) = state.cwz.col(i - 1)
                            .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
                            .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      if (clamp_raw_controls_) {
        state.cwz.col(i - 1) = state.wz.col(i);
      }

      if (is_holo) {
        const auto lower_bound_vy =
            (state.vy.col(i - 1) > 0)
                .select(state.vy.col(i - 1) + min_delta_vy,
                        state.vy.col(i - 1) - max_delta_vy);
        const auto upper_bound_vy =
            (state.vy.col(i - 1) > 0)
                .select(state.vy.col(i - 1) + max_delta_vy,
                        state.vy.col(i - 1) - min_delta_vy);
        state.vy.col(i) = state.cvy.col(i - 1)
                              .cwiseMax(lower_bound_vy)
                              .cwiseMin(upper_bound_vy);
        if (clamp_raw_controls_) {
          state.cvy.col(i - 1) = state.vy.col(i);
        }
      }
    }

    const unsigned int offset_vx =
        static_cast<unsigned int>(std::floor((model_delay_vx_ / model_dt_) + 0.5));
    const unsigned int offset_vy =
        static_cast<unsigned int>(std::floor((model_delay_vy_ / model_dt_) + 0.5));
    const unsigned int offset_wz =
        static_cast<unsigned int>(std::floor((model_delay_wz_ / model_dt_) + 0.5));

    if (offset_vx > 0u || offset_wz > 0u || (is_holo && offset_vy > 0u)) {
      applyDelayShift(state, is_holo, offset_vx, offset_vy, offset_wz);
    }
  }

  virtual bool isHolonomic() const = 0;

  virtual void applyConstraints(models::ControlSequence& /*control_sequence*/) {}

 protected:
  void applyDelayShift(models::State& state, bool is_holo, unsigned int offset_vx,
                       unsigned int offset_vy, unsigned int offset_wz) const {
    auto shift = [](Eigen::ArrayXXf& velocities, unsigned int offset,
                    const std::vector<float>& history) {
      const unsigned int cols = static_cast<unsigned int>(velocities.cols());
      if (offset == 0u || cols == 0u) {
        return;
      }

      for (unsigned int k = (offset < cols) ? cols - offset : 0u; k > 0; --k) {
        velocities.col(offset + k - 1) = velocities.col(k);
      }

      const unsigned int end = std::min(offset, cols);
      for (unsigned int j = 1; j < end; ++j) {
        velocities.col(j).setConstant(history[j]);
      }
    };

    shift(state.vx, offset_vx, cmd_history_vx_);
    shift(state.wz, offset_wz, cmd_history_wz_);

    if (is_holo) {
      shift(state.vy, offset_vy, cmd_history_vy_);
    }
  }

  std::size_t offsetSteps(float delay) const {
    if (delay <= 0.0f || model_dt_ <= 0.0f) {
      return 0u;
    }
    return static_cast<std::size_t>(std::floor(delay / model_dt_ + 0.5f));
  }

  static void pushOne(std::vector<float>& buf, float v) {
    if (buf.empty()) {
      return;
    }
    std::rotate(buf.begin(), buf.begin() + 1, buf.end());
    buf.back() = v;
  }

  float model_dt_{0.0f};
  float model_delay_vx_{0.0f};
  float model_delay_vy_{0.0f};
  float model_delay_wz_{0.0f};
  bool clamp_raw_controls_{false};

  std::vector<float> cmd_history_vx_;
  std::vector<float> cmd_history_vy_;
  std::vector<float> cmd_history_wz_;

  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                  0.0f, 0.0f, 0.0f, 0.0f};
};

class AckermannMotionModel : public MotionModel {
 public:
  explicit AckermannMotionModel(const proto::MPPIControllerOptions* options,
                                const std::string& /*name*/) {
    if (options && options->has_ackermann_constraints()) {
      min_turning_r_ = options->ackermann_constraints().min_turning_r();
    } else {
      min_turning_r_ = 0.2f;
    }
  }

  bool isHolonomic() const override { return false; }

  void applyConstraints(models::ControlSequence& control_sequence) override {
    const auto wz_constrained = control_sequence.vx.abs() / min_turning_r_;
    control_sequence.wz =
        control_sequence.wz.max(-wz_constrained).min(wz_constrained);
  }

  float getMinTurningRadius() const { return min_turning_r_; }

 private:
  float min_turning_r_{0.0f};
};

class DiffDriveMotionModel : public MotionModel {
 public:
  DiffDriveMotionModel() = default;

  bool isHolonomic() const override { return false; }
};

class OmniMotionModel : public MotionModel {
 public:
  OmniMotionModel() = default;

  bool isHolonomic() const override { return true; }
};

}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
