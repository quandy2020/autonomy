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

#include "examples/runtime/demo_loop.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/transport/qos/qos_profile_conf.hpp"

#include <automsgs/msgs/std_msgs/string.pb.h>
#include "automanip/core/reference/target_trajectories.hpp"
#include "automanip/core/thread_support/execute_and_sleep.hpp"
#include "automanip/core/thread_support/set_thread_priority.hpp"
#include "automanip/mpc/mpc_mrt_interface.hpp"
#include "examples/runtime/markers.hpp"

namespace automanip {
namespace examples {
namespace {

std::atomic<bool> g_running{true};

// Shared-memory channels start at 16 KiB. A full mesh URDF is larger than
// that, so publish a path and let autoviz read the file.
std::string PublishableDescription(const std::string& name, const std::string& xml) {
  constexpr std::size_t kInlineLimit = 12000;
  if (xml.size() <= kInlineLimit || xml.find("<robot") == std::string::npos) {
    return xml;
  }
  std::error_code error;
  std::filesystem::create_directories("/tmp/automanip", error);
  const std::string path = "/tmp/automanip/" + name + "_description.urdf";
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    return xml;
  }
  output << xml;
  return path;
}

void HandleSignal(int) { g_running = false; }

bool StateNeedsRecovery(const std::string& name, const vector_t& state) {
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    if (!std::isfinite(state(i)) || std::abs(state(i)) > 1e3) {
      return true;
    }
  }
  // Euler pitch near ±pi/2 makes the quadrotor kinematics singular.
  if (name == "quadrotor" && state.size() >= 12) {
    if (std::abs(state(3)) > 1.4 || std::abs(state(4)) > 1.4) {
      return true;
    }
    for (Eigen::Index i = 6; i < state.size(); ++i) {
      if (std::abs(state(i)) > 25.0) {
        return true;
      }
    }
  }
  return false;
}

/** Drop a diverged lean and rate so the next solve can balance again. */
void RecoverState(const std::string& name, vector_t* state) {
  if (state == nullptr) {
    return;
  }
  auto& value = *state;
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (!std::isfinite(value(i)) || std::abs(value(i)) > 1e3) {
      value(i) = 0;
    }
  }
  if (name == "ballbot" && value.size() >= 5) {
    value(2) = std::atan2(std::sin(value(2)), std::cos(value(2)));
    value(3) = 0;
    value(4) = 0;
    if (value.size() > 5) {
      value.tail(value.size() - 5).setZero();
    }
  }
  if (name == "quadrotor" && value.size() >= 12) {
    if (!std::isfinite(value(0)) || std::abs(value(0)) > 50.0) {
      value(0) = 0;
    }
    if (!std::isfinite(value(1)) || std::abs(value(1)) > 50.0) {
      value(1) = 0;
    }
    if (!std::isfinite(value(2)) || value(2) < 0.2 || value(2) > 20.0) {
      value(2) = 1.0;
    }
    value(3) = 0;
    value(4) = 0;
    value(5) = std::atan2(std::sin(value(5)), std::cos(value(5)));
    value.tail(value.size() - 6).setZero();
  }
  // Centroidal state is [momentum, base pose, joints]. Zeroing the tail
  // drops the base onto the ground and straightens the legs.
  if (name == "legged_robot" && value.size() >= 12) {
    value.head(6).setZero();
    if (!std::isfinite(value(8)) || value(8) < 0.3) {
      value(8) = 0.575;
    }
    value(10) = 0;
    value(11) = 0;
  }
}

}  // namespace

int RunDemo(const DemoRequest& request) {
  if (request.mpc == nullptr || request.rollout == nullptr ||
      !request.publish || !request.on_target) {
    AERROR << "example demo is missing mpc, rollout, or callbacks";
    return 1;
  }

  g_running = true;
  autolink::Init(request.name.c_str());
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  auto node = autolink::CreateNode(request.name);
  if (!node) {
    AERROR << "autolink node failed for " << request.name;
    autolink::Clear();
    return 1;
  }

  const std::string prefix = "/" + request.name;
  auto joint_states_writer = node->CreateWriter<JointStateMsg>("/joint_states");
  auto marker_writer = node->CreateWriter<MarkerArray>(prefix + "/markers");
  auto path_writer = node->CreateWriter<PathMsg>(prefix + "/path");
  auto tf_writer = node->CreateWriter<TfMsg>("/tf");
  using DescriptionMsg = automsgs::msgs::std_msgs::String;
  const std::string description_payload =
      PublishableDescription(request.name, request.robot_description);
  std::shared_ptr<autolink::Writer<DescriptionMsg>> robot_description_writer;
  if (!request.robot_description.empty()) {
    // Latched, like a transient-local /robot_description: a display that
    // subscribes after the first publish still receives the last URDF.
    autolink::proto::RoleAttributes description_attr;
    description_attr.set_channel_name("/robot_description");
    description_attr.mutable_qos_profile()->CopyFrom(
        autolink::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
    robot_description_writer =
        node->CreateWriter<DescriptionMsg>(description_attr);
  }
  auto publish_description = [&]() {
    if (!robot_description_writer || description_payload.empty()) {
      return;
    }
    auto message = std::make_shared<DescriptionMsg>();
    message->set_data(description_payload);
    robot_description_writer->Write(message);
  };
  publish_description();

  if (request.reference) {
    request.mpc->getSolverPtr()->setReferenceManager(request.reference);
  }
  MPC_MRT_Interface mrt(*request.mpc);
  mrt.initRollout(request.rollout);

  SystemObservation observation;
  observation.time = 0.0;
  observation.state = request.initial_state;
  observation.input = request.initial_input;
  observation.mode = request.initial_mode;
  vector_t target_state = request.initial_target;
  TargetTrajectories target =
      request.initial_trajectories.empty()
          ? TargetTrajectories({0.0}, {target_state}, {request.initial_input})
          : request.initial_trajectories;
  if (!target.stateTrajectory.empty()) {
    target_state = target.stateTrajectory.back();
  }
  std::mutex target_mu;

  auto apply_pose = [&](const std::shared_ptr<PoseStamped>& pose) {
    if (!pose) {
      return;
    }
    vector_t previous;
    scalar_t time = 0.0;
    {
      std::lock_guard<std::mutex> lock(target_mu);
      previous = request.command_from_state ? observation.state : target_state;
      time = observation.time;
    }
    target = request.on_target(*pose, previous, time);
    if (!target.stateTrajectory.empty()) {
      std::lock_guard<std::mutex> lock(target_mu);
      target_state = target.stateTrajectory.back();
    }
    mrt.getReferenceManager().setTargetTrajectories(target);
    AINFO << request.name << " target updated";
  };
  std::shared_ptr<autolink::Writer<TwistStampedMsg>> twist_writer;
  if (request.velocity) {
    twist_writer = node->CreateWriter<TwistStampedMsg>("/cmd_vel");
  }
  auto target_reader = node->CreateReader<PoseStamped>(prefix + "/target_pose", apply_pose);
  // autoviz Nav Goal publishes geometry_msgs/PoseStamped on /goal_pose.
  auto goal_reader = node->CreateReader<PoseStamped>("/goal_pose", apply_pose);
  (void)target_reader;
  (void)goal_reader;

  mrt.setCurrentObservation(observation);
  mrt.getReferenceManager().setTargetTrajectories(target);
  AINFO << request.name << " waiting for the initial policy";
  try {
    for (int attempt = 0; g_running.load() && !mrt.initialPolicyReceived() && attempt < 20;
         ++attempt) {
      mrt.advanceMpc();
      mrt.updatePolicy();
    }
  } catch (const std::exception& ex) {
    AERROR << request.name << " initial solve failed: " << ex.what();
    autolink::Clear();
    return 1;
  }
  if (!mrt.initialPolicyReceived()) {
    AERROR << request.name << " initial policy was not received";
    autolink::Clear();
    return 1;
  }
  AINFO << request.name << " initial policy received";

  double mpc_hz = request.mpc_settings.mpcDesiredFrequency_;
  double mrt_hz = request.mpc_settings.mrtDesiredFrequency_;
  if (!(mpc_hz > 0.0)) {
    mpc_hz = 50.0;
  }
  if (!(mrt_hz > 0.0)) {
    mrt_hz = 100.0;
  }

  // While set, the rollout loop holds the pose. A solve that throws, or a
  // state that has already run away, sets this so the feedforward is not
  // integrated on top of a diverged pose.
  std::atomic<bool> mpc_running{true};
  // While set, the rollout loop holds the pose. A solve that throws, or a
  // state that has already run away, sets this so the feedforward is not
  // integrated on top of a diverged pose.
  std::atomic<bool> hold_rollout{false};
  std::atomic<bool> reset_requested{false};
  // A failed solve used to reset, fly the new policy, and trip the same
  // guard again a few hundred milliseconds later.
  std::atomic<int64_t> recovery_not_before_ms{0};
  const auto now_ms = []() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  auto recover_mpc = [&]() {
    const int64_t now = now_ms();
    if (now < recovery_not_before_ms.load()) {
      return;
    }
    recovery_not_before_ms.store(now + 1000);
    hold_rollout.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    SystemObservation recovered;
    vector_t goal_state;
    {
      std::lock_guard<std::mutex> lock(target_mu);
      goal_state = target_state;
      RecoverState(request.name, &observation.state);
      if (request.name == "quadrotor" &&
          request.initial_input.size() == observation.input.size()) {
        observation.input = request.initial_input;
      } else {
        observation.input.setZero();
        RecoverState(request.name, &target_state);
      }
      recovered = observation;
    }
    const vector_t input = recovered.input;
    // A single sample at the current pose drops the walk. Keep a two-point
    // base command so the next solve can still trot toward the goal.
    TargetTrajectories hold({recovered.time}, {recovered.state}, {input});
    if (request.name == "legged_robot") {
      const TargetTrajectories commanded = mrt.getReferenceManager().getTargetTrajectories();
      if (commanded.timeTrajectory.size() >= 2 &&
          commanded.stateTrajectory.size() >= 2) {
        hold = commanded;
      }
    }
    try {
      // Stabilize in place first. The following warm start picks the goal back up.
      mrt.resetMpcNode(hold);
      mrt.getReferenceManager().setTargetTrajectories(hold);
      mrt.setCurrentObservation(recovered);
      mrt.advanceMpc();
      mrt.updatePolicy();
      if (request.name == "quadrotor" && goal_state.size() == recovered.state.size()) {
        goal_state(3) = 0.0;
        goal_state(4) = 0.0;
        if (goal_state.size() > 6) {
          goal_state.tail(goal_state.size() - 6).setZero();
        }
        const double distance = (goal_state.head<3>() - recovered.state.head<3>()).norm();
        const scalar_t arrival = recovered.time + std::max(0.5, distance / 2.0);
        const TargetTrajectories resumed({recovered.time, arrival}, {recovered.state, goal_state},
                                         {input, input});
        mrt.getReferenceManager().setTargetTrajectories(resumed);
        std::lock_guard<std::mutex> lock(target_mu);
        target_state = goal_state;
      } else {
        std::lock_guard<std::mutex> lock(target_mu);
        target_state = recovered.state;
      }
      AINFO << request.name << " recovered an upright pose";
      hold_rollout.store(false);
    } catch (const std::exception& ex) {
      AERROR << request.name << " mpc reset failed: " << ex.what();
    }
  };

  std::thread mpc_thread([&]() {
    while (mpc_running.load() && g_running.load()) {
      const bool retry_hold =
          hold_rollout.load() && now_ms() >= recovery_not_before_ms.load();
      if (reset_requested.exchange(false) || retry_hold) {
        if (now_ms() >= recovery_not_before_ms.load()) {
          recover_mpc();
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        continue;
      }
      if (hold_rollout.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      try {
        executeAndSleep([&]() { mrt.advanceMpc(); }, mpc_hz);
      } catch (const std::exception& ex) {
        AERROR << request.name << " mpc thread: " << ex.what();
        hold_rollout.store(true);
        recover_mpc();
      }
    }
  });
  setThreadPriority(request.thread_priority, mpc_thread);

  const double dt = 1.0 / mrt_hz;
  // Joint, marker, and TF traffic at the 400 Hz rollout rate makes Autoviz
  // rebuild the viewport continuously. The controller still steps at mrt_hz.
  const int viz_stride = std::max(1, static_cast<int>(mrt_hz / 30.0));
  int steps = 0;
  bool logged_plan_hold = false;
  try {
    while (g_running.load() && mpc_running.load() &&
           (request.max_steps <= 0 || steps < request.max_steps)) {
      executeAndSleep(
          [&]() {
            SystemObservation current;
            {
              std::lock_guard<std::mutex> lock(target_mu);
              current = observation;
            }
            if (now_ms() >= recovery_not_before_ms.load() &&
                StateNeedsRecovery(request.name, current.state)) {
              hold_rollout.store(true);
              reset_requested.store(true);
            }
            const bool holding = hold_rollout.load();
            if (!holding) {
              mrt.setCurrentObservation(current);
              mrt.updatePolicy();
            }
            SystemObservation next = current;
            const auto& policy = mrt.getPolicy();
            const bool plan_covers =
                !holding && !policy.timeTrajectory_.empty() &&
                current.time + dt <= policy.timeTrajectory_.back() + 1e-8;
            // The received plan is only the solution window (0.2 s). Integrating
            // the feedforward past that end prints a warning every MRT step and
            // tips the ballbot over.
            if (plan_covers) {
              logged_plan_hold = false;
              next.time = current.time + dt;
              try {
                mrt.rolloutPolicy(current.time, current.state, dt, next.state, next.input,
                                  next.mode);
              } catch (const std::exception& ex) {
                AERROR << request.name << " rollout: " << ex.what();
                next = current;
                hold_rollout.store(true);
                reset_requested.store(true);
              }
            } else if (!logged_plan_hold && !holding) {
              AINFO << request.name << " waiting for the next policy";
              logged_plan_hold = true;
            }
            if (now_ms() >= recovery_not_before_ms.load() &&
                StateNeedsRecovery(request.name, next.state)) {
              next = current;
              hold_rollout.store(true);
              reset_requested.store(true);
            }
            if (!hold_rollout.load()) {
              std::lock_guard<std::mutex> lock(target_mu);
              observation = next;
            }

            if (steps % viz_stride == 0) {
              MarkerArray markers;
              JointStateMsg joints;
              PathMsg path;
              TfMsg tf;
              StampHeader(path.mutable_header(), "map");
              request.publish(next, mrt.getPolicy(), mrt.getCommand(), &markers, &joints,
                              &path, &tf);
              joint_states_writer->Write(std::make_shared<JointStateMsg>(joints));
              // An empty protobuf serializes to 0 bytes. RawMessage rejects that
              // size, and autoviz then logs a parse failure on every cycle.
              if (markers.markers_size() > 0) {
                marker_writer->Write(std::make_shared<MarkerArray>(markers));
              }
              path_writer->Write(std::make_shared<PathMsg>(path));
              tf_writer->Write(std::make_shared<TfMsg>(tf));
              if (twist_writer) {
                TwistStampedMsg twist;
                request.velocity(next, &twist);
                twist_writer->Write(std::make_shared<TwistStampedMsg>(twist));
              }
            }
            if (steps % 100 == 0) {
              publish_description();
            }
            ++steps;
          },
          mrt_hz);
    }
  } catch (const std::exception& ex) {
    AERROR << request.name << " rollout: " << ex.what();
  }

  mpc_running = false;
  g_running = false;
  if (mpc_thread.joinable()) {
    mpc_thread.join();
  }
  AINFO << request.name << " stopped after " << steps << " steps";
  autolink::Clear();
  return 0;
}

}  // namespace examples
}  // namespace automanip
