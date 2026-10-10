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

#include <atomic>
#include <csignal>
#include <exception>
#include <mutex>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"

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

void HandleSignal(int) { g_running = false; }

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
  std::shared_ptr<autolink::Writer<DescriptionMsg>> robot_description_writer;
  if (!request.robot_description.empty()) {
    robot_description_writer =
        node->CreateWriter<DescriptionMsg>("/robot_description");
  }
  auto publish_description = [&]() {
    if (!robot_description_writer || request.robot_description.empty()) {
      return;
    }
    auto message = std::make_shared<DescriptionMsg>();
    message->set_data(request.robot_description);
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
  TargetTrajectories target({0.0}, {target_state}, {request.initial_input});
  std::mutex target_mu;

  auto target_reader = node->CreateReader<PoseStamped>(
      prefix + "/target_pose",
      [&](const std::shared_ptr<PoseStamped>& pose) {
        if (!pose) {
          return;
        }
        vector_t previous;
        scalar_t time = 0.0;
        {
          std::lock_guard<std::mutex> lock(target_mu);
          previous = target_state;
          time = observation.time;
        }
        target = request.on_target(*pose, previous, time);
        if (!target.stateTrajectory.empty()) {
          std::lock_guard<std::mutex> lock(target_mu);
          target_state = target.stateTrajectory.front();
        }
        mrt.getReferenceManager().setTargetTrajectories(target);
        AINFO << request.name << " target updated";
      });
  (void)target_reader;

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

  std::atomic<bool> mpc_running{true};
  std::thread mpc_thread([&]() {
    while (mpc_running.load() && g_running.load()) {
      try {
        executeAndSleep([&]() { mrt.advanceMpc(); }, mpc_hz);
      } catch (const std::exception& ex) {
        AERROR << request.name << " mpc thread: " << ex.what();
        mpc_running = false;
        g_running = false;
      }
    }
  });
  setThreadPriority(request.thread_priority, mpc_thread);

  const double dt = 1.0 / mrt_hz;
  int steps = 0;
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
            mrt.setCurrentObservation(current);
            mrt.updatePolicy();
            SystemObservation next;
            next.time = current.time + dt;
            mrt.rolloutPolicy(current.time, current.state, dt, next.state, next.input,
                              next.mode);
            {
              std::lock_guard<std::mutex> lock(target_mu);
              observation = next;
            }

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
