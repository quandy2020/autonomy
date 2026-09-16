/*
 * Copyright 2026 The Openbot Authors
 *
 * Exploration task: orchestrates mapping + navigation waypoint loop via
 * ExplorationGoal (no separate Autolink Action surface).
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/std_msgs/bool.pb.h>
#include <automsgs/task/exploration.pb.h>
#include <automsgs/task/mapping.pb.h>
#include <automsgs/task/navigation.pb.h>

namespace autonomy {
namespace task {

/**
 * First-class exploration control surface on /autonomy/task/exploration/*.
 *
 * START may load a map and publish an area-centroid waypoint; PAUSE/RESUME/
 * CANCEL forward to NavigationGoal (and MAP_CMD_CANCEL when mapping is on).
 * Waypoints arriving on /exploration/waypoint are converted to NAV_CMD_START.
 */
class ExplorationTask
    : public TypedTaskAppBase<::autonomy::task::proto::ExplorationGoal,
                              ::autonomy::task::proto::ExplorationFeedback,
                              ::autonomy::task::proto::ExplorationResult>
{
public:
    static constexpr bool kUsesNavigationClient = false;

    using SubmitNavigation =
        std::function<bool(const ::autonomy::task::proto::NavigationGoal&)>;
    using SubmitMapping =
        std::function<bool(const ::autonomy::task::proto::MappingGoal&)>;
    using IsNavigationActive = std::function<bool()>;
    using GetNavigationResult =
        std::function<bool(::autonomy::task::proto::NavigationResult*)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationTask)

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetNode(std::shared_ptr<autolink::Node> node);
    void SetSubmitNavigation(SubmitNavigation submit);
    void SetSubmitMapping(SubmitMapping submit);
    void SetNavigationProbe(IsNavigationActive is_active,
                            GetNavigationResult get_result);

    void Shutdown() override;
    bool Cancel() override;
    bool Pause() override;
    bool Resume() override;

protected:
    bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;
    bool OnGoal(
        const ::autonomy::task::proto::ExplorationGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::ExplorationFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::ExplorationResult* result) const override;

private:
    bool PublishNavigation(
        ::autonomy::task::proto::NavigationCommand command);
    bool PublishMapping(::autonomy::task::proto::MapCommand command,
                        const std::string& map_name = {});
    bool PublishAreaWaypoint(
        const ::automsgs::msgs::geometry_msgs::Polygon& area);
    bool NavigateToWaypoint(
        const ::automsgs::msgs::geometry_msgs::PoseStamped& pose);
    void StopUnderlying();
    void StartMonitor(std::chrono::milliseconds period);
    void StopMonitor();
    void HandleWaypoint(
        const std::shared_ptr<::automsgs::msgs::geometry_msgs::PoseStamped>&
            pose);
    void HandleFinished(
        const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& message);

    std::shared_ptr<autolink::Node> node_;
    SubmitNavigation submit_navigation_;
    SubmitMapping submit_mapping_;
    IsNavigationActive is_navigation_active_;
    GetNavigationResult get_navigation_result_;

    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::geometry_msgs::PoseStamped>>
        waypoint_reader_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::std_msgs::Bool>>
        finished_reader_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::std_msgs::Bool>>
        waypoint_reached_writer_;

    std::thread monitor_thread_;
    std::atomic<bool> monitor_running_{false};
    std::atomic<bool> nav_pending_{false};
    std::atomic<bool> nav_was_active_{false};
    double last_waypoint_x_{0.0};
    double last_waypoint_y_{0.0};

    ::autonomy::task::proto::ExplorationStatus status_{
        ::autonomy::task::proto::EXPLORATION_STATUS_IDLE};
    std::string map_name_;
    bool mapping_enabled_{false};
    float coverage_target_{0.f};
};

}  // namespace task
}  // namespace autonomy
