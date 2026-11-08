/*
 * Copyright 2026 The Navigation Authors (navigation)
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

#include "navigation/runner.hpp"

#include "autonomy/common/helper_functions/angle_utils.hpp"
#include "autonomy/common/math/math.hpp"
#ifdef NAVIGATION_HAS_AUTONOMY_TIME
#include "autonomy/common/time.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "navigation/planning/plugin_manager.hpp"
#include "navigation/planning/plugins/memory/pose_indexed_memory.hpp"

namespace navigation
{
namespace
{

std::uint64_t DecisionPeriodNs(const config::NavigationConfig& configuration)
{
    if (!configuration.has_policy())
    {
        return 200000000ULL;
    }
    if (configuration.policy().has_vntpa())
    {
        const double rate = configuration.policy().vntpa().decision_rate_hz();
        if (std::isfinite(rate) && rate > 0.0)
        {
            return static_cast<std::uint64_t>(1e9 / rate);
        }
    }
    return 200000000ULL;
}

}  // namespace

Runner::Runner(config::NavigationConfig configuration, Dependencies dependencies)
    : configuration_(std::move(configuration)), dependencies_(std::move(dependencies))
{
    decision_period_ns_ = DecisionPeriodNs(configuration_);
}

Runner::~Runner()
{
    Stop();
}

std::unique_ptr<Runner> Runner::Create(const config::NavigationConfig& configuration)
{
    planning::PluginManager& plugins = planning::PluginManager::Instance();

    Dependencies dependencies;
    dependencies.encoder = plugins.CreateEncoder("defm", configuration);
    if (dependencies.encoder == nullptr)
    {
        return nullptr;
    }

    const std::string memory_name = configuration.memory().name().empty()
                                        ? "pose_indexed"
                                        : configuration.memory().name();
    dependencies.memory =
        plugins.CreateMemory(memory_name, configuration.memory());
    if (dependencies.memory == nullptr &&
        memory_name == "pose_indexed" && configuration.memory().has_pose_indexed())
    {
        dependencies.memory = std::make_unique<planning::PoseIndexedMemory>(
            configuration.memory().pose_indexed());
    }
    if (dependencies.memory == nullptr)
    {
        return nullptr;
    }

    const std::string policy_name =
        configuration.policy().name().empty() ? "vntpa" : configuration.policy().name();
    dependencies.policy = plugins.CreatePolicy(policy_name, configuration);
    if (dependencies.policy == nullptr ||
        !dependencies.policy->Configure(configuration.policy()))
    {
        return nullptr;
    }

    planning::ChunkBlender::Options blender_options;
    if (configuration.policy().has_vntpa())
    {
        const auto& vntpa = configuration.policy().vntpa();
        blender_options.chunk_length =
            vntpa.action_chunk_length() == 0 ? 8 : vntpa.action_chunk_length();
        blender_options.execute_length =
            vntpa.execute_length() == 0 ? 2 : vntpa.execute_length();
        blender_options.action_dt_s =
            vntpa.action_dt_s() > 0.0 ? vntpa.action_dt_s() : 0.1;
    }
    dependencies.blender =
        std::make_unique<planning::ChunkBlender>(blender_options);

    auto scenario = std::make_unique<planning::ScenarioManager>();
    if (!scenario->Configure(configuration))
    {
        return nullptr;
    }
    dependencies.scenario = std::move(scenario);

    return std::make_unique<Runner>(configuration, std::move(dependencies));
}

bool Runner::Start()
{
    if (running_)
    {
        return true;
    }
    if (dependencies_.encoder == nullptr || dependencies_.memory == nullptr ||
        dependencies_.policy == nullptr || dependencies_.blender == nullptr)
    {
        return false;
    }
    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_stop_ = false;
        pending_job_.reset();
        pending_result_.reset();
        result_ready_ = false;
        worker_busy_ = false;
    }
    worker_ = common::JoinableThread([this]() { WorkerLoop(); });
    running_ = true;
    last_tick_ns_ = 0;
    last_decision_ns_ = 0;
    return true;
}

void Runner::Stop()
{
    if (!running_)
    {
        worker_.Join();
        return;
    }
    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_stop_ = true;
        pending_job_.reset();
    }
    worker_cv_.notify_all();
    worker_.Join();
    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_result_.reset();
        result_ready_ = false;
        worker_busy_ = false;
    }
    running_ = false;
    if (dependencies_.blender != nullptr)
    {
        dependencies_.blender->Reset();
    }
    if (dependencies_.scenario != nullptr)
    {
        dependencies_.scenario->Reset();
    }
}

std::optional<common::Twist2D> Runner::Tick(const observation::Observation& observation,
                                            const common::Pose2D& current_pose,
                                            const common::Twist2D& current_velocity)
{
    if (!running_ || !common::IsFinite(current_pose) || !common::IsFinite(current_velocity))
    {
        return std::nullopt;
    }

    {
        const std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!active_goal_.has_value())
        {
            if (dependencies_.blender != nullptr)
            {
                dependencies_.blender->Reset();
            }
            return std::nullopt;
        }
    }

    const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
    double dt_s = 1.0 / 15.0;
    if (last_tick_ns_ > 0 && now_ns > last_tick_ns_)
    {
#ifdef NAVIGATION_HAS_AUTONOMY_TIME
        dt_s = autonomy::common::ToSeconds(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::nanoseconds(now_ns - last_tick_ns_)));
#else
        dt_s = std::chrono::duration<double>(
                   std::chrono::nanoseconds(now_ns - last_tick_ns_))
                   .count();
#endif
        dt_s = autonomy::common::Clamp(dt_s, 1e-3, 0.5);
    }
    last_tick_ns_ = now_ns;

    observation::DepthFeature feature;
    if (!dependencies_.encoder->Encode(observation, &feature).Ok())
    {
        return common::Twist2D{};
    }

    memory::Keyframe keyframe;
    keyframe.pose = current_pose;
    keyframe.stamp_ns = feature.stamp_ns();
    keyframe.embedding_dim = static_cast<std::uint32_t>(
        std::min<std::size_t>(feature.pooled_size(), memory::kMaxEmbeddingDim));
    for (std::size_t i = 0; i < keyframe.embedding_dim; ++i)
    {
        keyframe.embedding[i] = feature.pooled(static_cast<int>(i));
    }
    if (keyframe.embedding_dim > 0)
    {
        (void)dependencies_.memory->Insert(keyframe);
    }

    UpdateGoalReached(current_pose);

    PolicyResult result;
    if (TryTakeResult(&result))
    {
        if (result.intent.Ok() && result.intent.Value().has_chunk())
        {
            (void)dependencies_.blender->Push(result.intent.Value().chunk(), now_ns);
        }
    }

    (void)PostJobIfDue(observation, feature, current_pose, now_ns, dt_s);

    common::Twist2D command;
    if (!dependencies_.blender->Consume(dt_s, &command))
    {
        return common::Twist2D{};
    }
    if (dependencies_.scenario != nullptr)
    {
        planning::NavigationFrame frame;
        frame.observation = &observation;
        frame.pose = current_pose;
        frame.velocity = current_velocity;
        frame.command = command;
        frame.now_ns = static_cast<std::uint64_t>(now_ns);
        frame.dt_s = dt_s;
        frame.intent.mutable_velocity()->set_x(command.x());
        frame.intent.mutable_velocity()->set_y(command.y());
        frame.intent.mutable_velocity()->set_theta(command.theta());
        const auto planned = dependencies_.scenario->Plan(frame);
        if (!planned.Ok())
        {
            return common::Twist2D{};
        }
        command = planned.Value();
    }
    return command;
}

void Runner::SetGoal(const GoalSpec& goal)
{
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ = goal;
    goal_reached_ = false;
    if (dependencies_.blender != nullptr)
    {
        dependencies_.blender->Reset();
    }
    if (dependencies_.scenario != nullptr)
    {
        (void)dependencies_.scenario->Update(goal);
    }
    last_decision_ns_ = 0;
}

bool Runner::HasActiveGoal() const
{
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    return active_goal_.has_value();
}

void Runner::ClearGoal()
{
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
    goal_reached_ = false;
    if (dependencies_.blender != nullptr)
    {
        dependencies_.blender->Reset();
    }
    if (dependencies_.scenario != nullptr)
    {
        dependencies_.scenario->Reset();
    }
}

bool Runner::IsGoalReached() const
{
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    return goal_reached_;
}

bool Runner::ResetMemory(bool keep_pose_index)
{
    if (dependencies_.memory == nullptr)
    {
        return false;
    }
    dependencies_.memory->Reset(keep_pose_index);
    return true;
}

std::size_t Runner::MemoryEntryCount() const
{
    return dependencies_.memory != nullptr ? dependencies_.memory->Size() : 0;
}

void Runner::WorkerLoop()
{
    while (true)
    {
        PolicyJob job;
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            worker_cv_.wait(lock, [this]() { return worker_stop_ || pending_job_.has_value(); });
            if (worker_stop_)
            {
                return;
            }
            job = std::move(*pending_job_);
            pending_job_.reset();
            worker_busy_ = true;
        }

        PolicyResult result;
        result.sequence = job.sequence;
        result.intent = dependencies_.policy->Query(job.observation, job.feature, job.goal,
                                                    job.context);

        {
            const std::lock_guard<std::mutex> lock(worker_mutex_);
            if (!worker_stop_)
            {
                pending_result_ = std::move(result);
                result_ready_ = true;
            }
            worker_busy_ = false;
        }
    }
}

bool Runner::TryTakeResult(PolicyResult* result)
{
    if (result == nullptr)
    {
        return false;
    }
    const std::lock_guard<std::mutex> lock(worker_mutex_);
    if (!result_ready_ || !pending_result_.has_value())
    {
        return false;
    }
    *result = std::move(*pending_result_);
    pending_result_.reset();
    result_ready_ = false;
    return true;
}

bool Runner::PostJobIfDue(const observation::Observation& observation,
                          const observation::DepthFeature& feature,
                          const common::Pose2D& pose, std::int64_t now_ns, double dt_s)
{
    (void)dt_s;
    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_busy_ || pending_job_.has_value())
        {
            return false;
        }
    }
    if (last_decision_ns_ > 0 &&
        now_ns - last_decision_ns_ < static_cast<std::int64_t>(decision_period_ns_))
    {
        return false;
    }

    GoalSpec goal;
    {
        const std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!active_goal_.has_value() || goal_reached_)
        {
            return false;
        }
        goal = *active_goal_;
    }

    PolicyJob job;
    job.observation = observation;
    job.feature = feature;
    job.goal = goal;
    job.context = dependencies_.memory->View(pose);
    job.sequence = next_sequence_++;

    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_job_ = std::move(job);
    }
    last_decision_ns_ = now_ns;
    worker_cv_.notify_one();
    return true;
}

void Runner::UpdateGoalReached(const common::Pose2D& pose)
{
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!active_goal_.has_value() || !active_goal_->has_pose())
    {
        return;
    }
    const double tolerance = active_goal_->position_tolerance() > 0.0
                                 ? active_goal_->position_tolerance()
                                 : 0.15;
    const double yaw_tolerance =
        active_goal_->yaw_tolerance_rad() > 0.0 ? active_goal_->yaw_tolerance_rad() : 0.15;
    const auto& target = active_goal_->pose();
    const auto& orientation = target.pose().orientation();
    const double target_yaw =
        std::atan2(2.0 * (orientation.w() * orientation.z() +
                          orientation.x() * orientation.y()),
                   1.0 - 2.0 * (orientation.y() * orientation.y() +
                                orientation.z() * orientation.z()));
    const double dx = pose.x() - target.pose().position().x();
    const double dy = pose.y() - target.pose().position().y();
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double yaw_error = std::abs(autonomy::common::helper_functions::wrap_angle(pose.theta() - target_yaw));
    goal_reached_ = distance <= tolerance && yaw_error <= yaw_tolerance;
}

}  // namespace navigation
