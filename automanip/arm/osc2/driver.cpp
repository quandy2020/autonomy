/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file driver.cpp
 * @brief osc2 arm backend (implementation).
 */

#include "arm/osc2/driver.hpp"

#include <mutex>

#include "arm/backend_register.hpp"
#include "arm/codec.hpp"
#include "arm/osc2/controller.hpp"
#include "autolink/common/log.hpp"

namespace automanip {
namespace arm {
namespace osc2 {
namespace {

class Osc2ArmDriver final : public ArmDriver {
 public:
  Osc2ArmDriver(std::string id, ArmPlantOptions options)
      : id_(std::move(id)),
        frame_id_(std::move(options.frame_id)),
        controller_(options.chain.dof() == 0 ? MakeDefaultArm()
                                             : std::move(options.chain),
                    options.osc2) {
    q_ = controller_.chain().Home();
    qdot_ = Eigen::VectorXd::Zero(controller_.chain().dof());
    controller_.SetPoseTarget(controller_.chain().Forward(q_));
    controller_.SetJointTarget(q_);
  }

  const std::string& GetArmId() const override { return id_; }

  bool Start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    mode_ = ArmMode::kHold;
    q_ = controller_.chain().Home();
    qdot_.setZero();
    controller_.SetJointTarget(q_);
    controller_.SetPoseTarget(controller_.chain().Forward(q_));
    controller_.Reset(q_);
    AINFO << "osc2 arm started id=" << id_
          << " dof=" << controller_.chain().dof();
    return true;
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    qdot_.setZero();
  }

  bool IsRunning() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  bool Step(double dt) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return false;
    }
    if (mode_ == ArmMode::kEstop || mode_ == ArmMode::kFault ||
        mode_ == ArmMode::kHold) {
      qdot_.setZero();
      return true;
    }
    if (mode_ == ArmMode::kHome) {
      controller_.SetObjective(Osc2Objective::kJoints);
      controller_.SetJointTarget(controller_.chain().Home());
      qdot_ = controller_.Compute(q_);
    } else {
      controller_.SetObjective(Osc2Objective::kPose);
      qdot_ = controller_.Compute(q_);
    }
    if (qdot_.size() != q_.size()) {
      qdot_ = Eigen::VectorXd::Zero(q_.size());
      return false;
    }
    if (dt > 0.0) {
      q_ += qdot_ * dt;
    }
    controller_.chain().ClampPosition(&q_);
    return true;
  }

  bool ApplyPoseTarget(const PoseCommand& pose) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || mode_ == ArmMode::kEstop || mode_ == ArmMode::kFault) {
      return false;
    }
    controller_.SetPoseTarget(ReadPose(pose));
    return true;
  }

  bool ApplyJointCommand(const JointCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || mode_ == ArmMode::kEstop || mode_ == ArmMode::kFault) {
      return false;
    }
    if (!ApplyJointValues(controller_.chain(), command, &q_, &qdot_)) {
      return false;
    }
    controller_.chain().ClampPosition(&q_);
    controller_.chain().ClampVelocity(&qdot_);
    mode_ = ArmMode::kHold;
    controller_.Reset(q_);
    return true;
  }

  bool ApplyMode(const std::string& mode) override {
    ArmMode parsed = ArmMode::kHold;
    if (!ParseArmModeCommand(mode, &parsed)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return false;
    }
    mode_ = parsed;
    if (mode_ != ArmMode::kTrack) {
      controller_.Reset(q_);
    }
    if (mode_ == ArmMode::kEstop || mode_ == ArmMode::kHold ||
        mode_ == ArmMode::kFault) {
      qdot_.setZero();
    }
    return true;
  }

  ArmMode GetMode() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
  }

  bool ReadJointState(JointState* state) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    WriteJointState(controller_.chain(), q_, qdot_, frame_id_, state);
    return state != nullptr;
  }

  bool ReadEndEffectorPose(PoseCommand* pose) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    WritePose(controller_.chain().Forward(q_), frame_id_, pose);
    return pose != nullptr;
  }

  bool TriggerEmergencyStop() override { return ApplyMode("estop"); }

 private:
  std::string id_;
  std::string frame_id_;
  Osc2Controller controller_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  ArmMode mode_ = ArmMode::kHold;
  bool running_ = false;
  mutable std::mutex mutex_;
};

}  // namespace

ArmDriver* CreateOsc2ArmDriver(const std::string& id,
                               const ArmPlantOptions& options) {
  return new Osc2ArmDriver(id, options);
}

REGISTER_ARM_BACKEND(osc2, "osc2", CreateOsc2ArmDriver, "ocs2")

}  // namespace osc2
}  // namespace arm
}  // namespace automanip
