/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/chassis/stub/driver.hpp"

#include <chrono>
#include <cmath>
#include <mutex>

#include "autodriver/chassis/backend_register.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace chassis {
namespace {

std::uint64_t NowNs() {
  using clock = std::chrono::steady_clock;
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          clock::now().time_since_epoch())
          .count());
}

class StubChassisDriver final : public ChassisDriver {
public:
  StubChassisDriver(ChassisId id, hardware::DriverParams params)
  : id_(std::move(id)), params_(std::move(params)) {}

  const ChassisId& GetChassisId() const override { return id_; }
  ChassisKind GetKind() const override { return ChassisKind::kDifferential; }

  bool Start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    last_integrate_ns_ = NowNs();
    AINFO << "stub chassis started id=" << id_;
    return true;
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = ChassisCommand{};
    running_ = false;
  }

  bool IsRunning() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  bool ApplyCommand(const ChassisCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return false;
    }
    if (command.emergency_stop) {
      command_ = ChassisCommand{};
      command_.emergency_stop = true;
      state_.emergency_stop_active = true;
      return true;
    }
    command_ = command;
    state_.emergency_stop_active = false;
    return true;
  }

  bool GetState(ChassisState* state) override {
    if (state == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    IntegrateLocked(NowNs());
    *state = state_;
    state->kind = ChassisKind::kDifferential;
    return true;
  }

private:
  void IntegrateLocked(std::uint64_t now_ns) {
    if (!running_ || last_integrate_ns_ == 0) {
      last_integrate_ns_ = now_ns;
      return;
    }
    const double dt =
        static_cast<double>(now_ns - last_integrate_ns_) * 1e-9;
    last_integrate_ns_ = now_ns;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const double vx = command_.emergency_stop ? 0.0 : command_.linear_x;
    const double wz = command_.emergency_stop ? 0.0 : command_.angular_z;
    state_.pose_x += vx * std::cos(state_.pose_yaw) * dt;
    state_.pose_y += vx * std::sin(state_.pose_yaw) * dt;
    state_.pose_yaw += wz * dt;
    state_.linear_x = vx;
    state_.linear_y = 0.0;
    state_.angular_z = wz;
    state_.stamp_ns = now_ns;
    state_.battery_soc = hardware::ParseDouble(params_, "battery_soc", 1.0);
  }

  ChassisId id_;
  hardware::DriverParams params_;
  mutable std::mutex mutex_;
  bool running_ = false;
  ChassisCommand command_;
  ChassisState state_;
  std::uint64_t last_integrate_ns_ = 0;
};

}  // namespace

std::shared_ptr<ChassisDriver> CreateStubChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params) {
  return std::make_shared<StubChassisDriver>(id, params);
}

}  // namespace chassis
}  // namespace autodriver

REGISTER_CHASSIS_BACKEND(stub, "stub",
                         autodriver::chassis::CreateStubChassisDriver,
                         "sim", "fake");
