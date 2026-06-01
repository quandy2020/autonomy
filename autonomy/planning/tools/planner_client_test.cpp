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

/**
 * Integration test for PlannerServer autolink endpoints.
 *
 * Starts PlannerServer in-process, then exercises action clients
 * (compute_path_to_pose, compute_path_through_poses) and is_path_valid service.
 *
 * Example:
 *   autonomy_planner_client_test \
 *     --configuration_directory=/workspace/autonomy/src/autonomy/config
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "autolink/action/create_client.hpp"
#include "autolink/action/types.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"

DEFINE_bool(use_synthetic_map, true,
            "Use synthetic free-space costmap (no map YAML required).");
DEFINE_double(start_x, 0.5, "Start pose x [m] on synthetic map.");
DEFINE_double(start_y, 0.5, "Start pose y [m].");
DEFINE_double(goal_x, 19.0, "Goal pose x [m].");
DEFINE_double(goal_y, 19.0, "Goal pose y [m].");
DEFINE_double(via_x, 10.0, "Intermediate via pose x [m] for through-poses test.");
DEFINE_double(via_y, 10.0, "Intermediate via pose y [m].");
DEFINE_int32(server_wait_ms, 5000,
             "Max wait for autolink action/service discovery [ms].");

namespace autonomy {
namespace planning {
namespace tools {
namespace {

namespace task_proto = tasks::behavior_tree::task_proto;
using PathValidRequest = task_proto::IsPathValid_Request;
using PathValidResponse = task_proto::IsPathValid_Response;

constexpr auto kAcceptTimeout = std::chrono::seconds(30);
constexpr auto kResultTimeout = std::chrono::seconds(60);
constexpr auto kServiceTimeout = std::chrono::seconds(10);

proto::PlannerOptions PrepareTestOptions(proto::PlannerOptions options,
                                         bool use_synthetic_map) {
    options.set_auto_smooth_after_plan(false);
    if (use_synthetic_map) {
        options.mutable_costmap()->clear_plugins();
        options.mutable_costmap()->add_plugins("none");
    }
    return options;
}

void SetupSyntheticCostmap(map::costmap_2d::Costmap2D* costmap) {
    if (costmap == nullptr) {
        return;
    }
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);
}

void FinalizeOfflineCostmap(PlannerServer& server) {
    auto wrapper = server.GetCostmapWrapper();
    if (wrapper == nullptr) {
        return;
    }
    wrapper->updateMap();
    wrapper->Stop();
}

commsgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                             const std::string& frame_id) {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

template <typename ActionTraits>
bool WaitForActionServer(
    const std::shared_ptr<autolink::action::Client<ActionTraits>>& client,
    std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (autolink::OK() && !client->ActionServerIsReady()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return autolink::OK() && client->ActionServerIsReady();
}

template <typename ActionTraits>
std::optional<typename ActionTraits::Result> SendActionGoal(
    const std::shared_ptr<autolink::action::Client<ActionTraits>>& client,
    const typename ActionTraits::Goal& goal) {
    using GoalHandle = autolink::action::ClientGoalHandle<ActionTraits>;

    const auto accepted_future = client->AsyncSendGoal(goal);
    if (accepted_future.wait_for(kAcceptTimeout) != std::future_status::ready) {
        AERROR << "Timeout waiting for goal acceptance";
        return std::nullopt;
    }

    const std::shared_ptr<GoalHandle> handle = accepted_future.get();
    if (!handle) {
        AERROR << "Goal rejected by action server";
        return std::nullopt;
    }

    auto result_future = handle->AsyncGetResult();
    if (result_future.wait_for(kResultTimeout) != std::future_status::ready) {
        AERROR << "Timeout waiting for action result";
        return std::nullopt;
    }

    const typename GoalHandle::WrappedResult wrapped = result_future.get();
    if (wrapped.code != autolink::action::ResultCode::SUCCEEDED ||
        !wrapped.result) {
        AERROR << "Action failed: " << autolink::action::ToString(wrapped.code)
                   << (wrapped.result && !wrapped.result->error_msg().empty()
                           ? (" — " + wrapped.result->error_msg())
                           : "");
        return std::nullopt;
    }
    return *wrapped.result;
}

bool WaitForServiceClient(
    const std::shared_ptr<autolink::Client<PathValidRequest, PathValidResponse>>&
        client,
    std::chrono::milliseconds timeout) {
    return client->WaitForService(timeout);
}

struct TestContext {
    std::unique_ptr<PlannerServer> server;
    std::shared_ptr<autolink::Node> client_node;
    std::string frame_id;
    std::string planner_id;
    commsgs::planning_msgs::Path last_path;
};

bool TestComputePathToPose(TestContext& ctx) {
    auto client = autolink::action::CreateClient<
        tasks::behavior_tree::ComputePathToPoseActionTraits>(
        ctx.client_node, kComputePathToPoseActionName);
    if (!WaitForActionServer(client,
                             std::chrono::milliseconds(FLAGS_server_wait_ms))) {
        AERROR << "compute_path_to_pose server not ready";
        return false;
    }

    tasks::behavior_tree::ComputePathToPoseActionTraits::Goal goal;
    goal.set_use_start(true);
    goal.set_planner_id(ctx.planner_id);
    *goal.mutable_start() = commsgs::geometry_msgs::ToProto(
        MakePose(FLAGS_start_x, FLAGS_start_y, ctx.frame_id));
    *goal.mutable_goal() = commsgs::geometry_msgs::ToProto(
        MakePose(FLAGS_goal_x, FLAGS_goal_y, ctx.frame_id));

    const auto result = SendActionGoal<
        tasks::behavior_tree::ComputePathToPoseActionTraits>(client, goal);
    if (!result || result->path().poses_size() < 2) {
        AERROR << "compute_path_to_pose returned invalid path";
        return false;
    }
    if (result->error_code() != task_proto::COMPUTE_PATH_TO_POSE_NONE) {
        AERROR << "compute_path_to_pose error_code="
                   << result->error_code();
        return false;
    }

    ctx.last_path = commsgs::planning_msgs::FromProto(result->path());
    AINFO << "compute_path_to_pose: path poses="
              << ctx.last_path.poses.size();
    return true;
}

bool TestComputePathThroughPoses(TestContext& ctx) {
    auto client = autolink::action::CreateClient<
        tasks::behavior_tree::ComputePathThroughPosesActionTraits>(
        ctx.client_node, kComputePathThroughPosesActionName);
    if (!WaitForActionServer(client,
                             std::chrono::milliseconds(FLAGS_server_wait_ms))) {
        AERROR << "compute_path_through_poses server not ready";
        return false;
    }

    tasks::behavior_tree::ComputePathThroughPosesActionTraits::Goal goal;
    goal.set_use_start(true);
    goal.set_planner_id(ctx.planner_id);
    *goal.mutable_start() = commsgs::geometry_msgs::ToProto(
        MakePose(FLAGS_start_x, FLAGS_start_y, ctx.frame_id));
    *goal.mutable_goals()->add_goals() = commsgs::geometry_msgs::ToProto(
        MakePose(FLAGS_via_x, FLAGS_via_y, ctx.frame_id));
    *goal.mutable_goals()->add_goals() = commsgs::geometry_msgs::ToProto(
        MakePose(FLAGS_goal_x, FLAGS_goal_y, ctx.frame_id));

    const auto result = SendActionGoal<
        tasks::behavior_tree::ComputePathThroughPosesActionTraits>(client, goal);
    if (!result || result->path().poses_size() < 2) {
        AERROR << "compute_path_through_poses returned invalid path";
        return false;
    }
    if (result->error_code() != task_proto::COMPUTE_PATH_THROUGH_POSES_NONE) {
        AERROR << "compute_path_through_poses error_code="
                   << result->error_code();
        return false;
    }

    AINFO << "compute_path_through_poses: path poses="
              << result->path().poses_size();
    return true;
}

bool TestIsPathValid(TestContext& ctx,
                     const commsgs::planning_msgs::Path& valid_path) {
    auto client =
        ctx.client_node->CreateClient<PathValidRequest, PathValidResponse>(
            kIsPathValidServiceName);
    if (client == nullptr) {
        AERROR << "is_path_valid service not ready";
        return false;
    }

    auto send_with_retry =
        [&](const std::shared_ptr<PathValidRequest>& request)
        -> std::shared_ptr<PathValidResponse> {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(FLAGS_server_wait_ms);
        while (autolink::OK()) {
            if (const auto response = client->SendRequest(request, kServiceTimeout)) {
                return response;
            }
            if (std::chrono::steady_clock::now() > deadline) {
                return nullptr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return nullptr;
    };

    auto valid_req = std::make_shared<PathValidRequest>();
    *valid_req->mutable_path() = commsgs::planning_msgs::ToProto(valid_path);
    valid_req->set_max_cost(253);
    const auto valid_resp = send_with_retry(valid_req);
    if (valid_resp == nullptr || !valid_resp->is_valid()) {
        AERROR << "is_path_valid rejected a collision-free path";
        return false;
    }

    auto empty_req = std::make_shared<PathValidRequest>();
    const auto empty_resp = send_with_retry(empty_req);
    if (empty_resp == nullptr || empty_resp->is_valid()) {
        AERROR << "is_path_valid should reject empty path";
        return false;
    }

    AINFO << "is_path_valid: valid path accepted, empty path rejected";
    return true;
}

bool TestMetricsIncreased(const PlannerServer& server) {
    const auto& metrics = server.GetMetrics();
    if (metrics.plans_requested.load() < 2 ||
        metrics.plans_succeeded.load() < 2) {
        AERROR << "Planner metrics unexpected: requested="
                   << metrics.plans_requested.load()
                   << " succeeded=" << metrics.plans_succeeded.load();
        return false;
    }
    AINFO << "Planner metrics: requested="
              << metrics.plans_requested.load()
              << " succeeded=" << metrics.plans_succeeded.load()
              << " failed=" << metrics.plans_failed.load();
    return true;
}

bool RunAllTests() {
    if (::autonomy::common::FLAGS_configuration_directory.empty()) {
        AERROR << "configuration_directory is required";
        return false;
    }

    auto options = PrepareTestOptions(
        CreateOptions(::autonomy::common::FLAGS_configuration_directory),
        FLAGS_use_synthetic_map);

    TestContext ctx;
    ctx.server = std::make_unique<PlannerServer>(options);
    ctx.frame_id = options.costmap().frame_id().empty()
                       ? "map"
                       : options.costmap().frame_id();
    ctx.planner_id = options.default_planner_id().empty()
                           ? "navfn_planner"
                           : options.default_planner_id();

    auto* costmap_wrapper = ctx.server->GetCostmapWrapper().get();
    auto* costmap =
        costmap_wrapper != nullptr ? costmap_wrapper->getCostmap() : nullptr;
    if (costmap == nullptr) {
        AERROR << "Costmap unavailable after PlannerServer construction";
        return false;
    }

    if (FLAGS_use_synthetic_map) {
        SetupSyntheticCostmap(costmap);
        AINFO << "Using synthetic free-space costmap";
    } else {
        AERROR << "Only use_synthetic_map=true is supported in this test";
        return false;
    }
    FinalizeOfflineCostmap(*ctx.server);

    ctx.client_node = autolink::CreateNode("planner_client_test");
    if (!ctx.client_node) {
        AERROR << "Failed to create autolink client node";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int passed = 0;
    int failed = 0;
    auto run = [&](const char* name, const std::function<bool()>& fn) {
        AINFO << "=== " << name << " ===";
        if (fn()) {
            ++passed;
            AINFO << "PASS: " << name;
        } else {
            ++failed;
            AERROR << "FAIL: " << name;
        }
    };

    run("compute_path_to_pose",
        [&]() { return TestComputePathToPose(ctx); });

    run("compute_path_through_poses",
        [&]() { return TestComputePathThroughPoses(ctx); });

    run("is_path_valid", [&]() {
        if (ctx.last_path.poses.empty()) {
            AERROR << "No sample path for is_path_valid test";
            return false;
        }
        return TestIsPathValid(ctx, ctx.last_path);
    });

    run("planner_metrics", [&]() { return TestMetricsIncreased(*ctx.server); });

    AINFO << "Results: " << passed << " passed, " << failed << " failed";
    return failed == 0;
}

}  // namespace

int RunPlannerClientTest(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RunAllTests() ? 0 : 1;
}

}  // namespace tools
}  // namespace planning
}  // namespace autonomy

int main(int argc, char** argv) {
    google::SetUsageMessage(
        "Integration test for PlannerServer autolink action/service endpoints.");

    if (!autolink::Init(argv[0])) {
        std::cerr << "autolink::Init failed\n";
        return 1;
    }

    const int rc = autonomy::planning::tools::RunPlannerClientTest(argc, argv);
    autolink::Clear();
    return rc;
}
