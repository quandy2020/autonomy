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
 * @brief Stub arm backend (implementation).
 */

#include "arm/stub/driver.hpp"

#include <mutex>

#include "arm/backend_register.hpp"
#include "arm/codec.hpp"
#include "autolink/common/log.hpp"

namespace automanip {
namespace arm {
namespace {

class StubArmDriver final : public ArmDriver {
 public:
  StubArmDriver(std::string id, ArmPlantOptions options)
      : id_(std::move(id)),
        chain_(std::move(options.chain)),
        frame_id_(std::move(options.frame_id)) {
    if (chain_.dof() == 0) {
      chain_ = MakeDefaultArm();
    }
    q_ = chain_.Home();
    qdot_ = Eigen::VectorXd::Zero(chain_.dof());
  }

  const std::string& GetArmId() const override { return id_; }

  bool Start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    mode_ = ArmMode::kHold;
    q_ = chain_.Home();
    qdot_.setZero();
    AINFO << "stub arm started id=" << id_ << " dof=" << chain_.dof();
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
    if (!running_ || mode_ == ArmMode::kEstop || mode_ == ArmMode::kFault) {
      qdot_.setZero();
      return running_;
    }
    if (dt > 0.0) {
      q_ += qdot_ * dt;
    }
    chain_.ClampPosition(&q_);
    return true;
  }

  bool ApplyPoseTarget(const PoseCommand&) override { return IsRunning(); }

  bool ApplyJointCommand(const JointCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || mode_ == ArmMode::kEstop || mode_ == ArmMode::kFault) {
      return false;
    }
    if (!ApplyJointValues(chain_, command, &q_, &qdot_)) {
      return false;
    }
    chain_.ClampPosition(&q_);
    chain_.ClampVelocity(&qdot_);
    mode_ = ArmMode::kHold;
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
    if (parsed == ArmMode::kTrack) {
      parsed = ArmMode::kHold;
    }
    mode_ = parsed;
    if (mode_ == ArmMode::kHome) {
      q_ = chain_.Home();
      qdot_.setZero();
      mode_ = ArmMode::kHold;
    }
    if (mode_ == ArmMode::kEstop || mode_ == ArmMode::kHold) {
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
    WriteJointState(chain_, q_, qdot_, frame_id_, state);
    return state != nullptr;
  }

  bool ReadEndEffectorPose(PoseCommand* pose) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    WritePose(chain_.Forward(q_), frame_id_, pose);
    return pose != nullptr;
  }

  bool TriggerEmergencyStop() override { return ApplyMode("estop"); }

 private:
  std::string id_;
  SerialChain chain_;
  std::string frame_id_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  ArmMode mode_ = ArmMode::kHold;
  bool running_ = false;
  mutable std::mutex mutex_;
};

}  // namespace

ArmDriver* CreateStubArmDriver(const std::string& id,
                               const ArmPlantOptions& options) {
  return new StubArmDriver(id, options);
}

REGISTER_ARM_BACKEND(stub, "stub", CreateStubArmDriver, "sim")

}  // namespace arm
}  // namespace automanip
