/*
 * Copyright 2026 The Openbot Authors
 *
 * Behavior-tree task application base (Nav2 BtNavigator-style host).
 */

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/task/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/behavior_tree/bt_profile.hpp"
#include "autonomy/task/behavior_tree/bt_runner.hpp"
#include "autonomy/task/behavior_tree/bt_status_logger.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include <automsgs/task/task_options.pb.h>
#include "behaviortree_cpp/blackboard.h"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/nav_msgs/behavior_tree.pb.h>
#include <automsgs/msgs/time_utils.hpp>

namespace autonomy {
namespace task {

/**
 * Hosts a BtRunner for a typed Goal/Feedback/Result task.
 *
 * Derived classes implement domain hooks:
 *   OnTreeInitialize, PopulateBlackboard, OnTreeTick, OnGoal, Fill*.
 */
template <typename GoalType, typename FeedbackType, typename ResultType>
class BtTaskApp : public TypedTaskAppBase<GoalType, FeedbackType, ResultType>
{
public:
    // When false, TaskServer skips injecting the shared NavigationClient.
    static constexpr bool kUsesNavigationClient = true;

    ~BtTaskApp() override { StopTree(); }

    void SetNode(std::shared_ptr<autolink::Node> node) {
        node_ = std::move(node);
    }

    void SetNavigationClient(navigation::NavigationClient::Ptr client) {
        navigation_client_ = std::move(client);
    }

    [[nodiscard]] bool HasNavigationClient() const {
        return static_cast<bool>(navigation_client_);
    }

    // Compatibility aliases used by TaskServer wiring.
    void SetAutolinkNode(std::shared_ptr<autolink::Node> node) {
        SetNode(std::move(node));
    }

    bool Cancel() override {
        StopTree();
        return TypedTaskAppBase<GoalType, FeedbackType, ResultType>::Cancel();
    }

    void Shutdown() override {
        StopTree();
        TypedTaskAppBase<GoalType, FeedbackType, ResultType>::Shutdown();
    }

protected:
    bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override {
        config_directory_ = options.config_directory();
        profile_ = BtDefaults::ProfileFor(options, this->GetTaskType());
        if (!runner_.Configure(profile_)) {
            return false;
        }
        runner_.SetBlackboardSetup(
            [this](const BT::Blackboard::Ptr& blackboard) {
                PopulateBlackboard(blackboard);
            });
        runner_.SetTickCallback([this]() { OnTreeTick(); });
        EnsureBehaviorTreeLogWriter();
        runner_.SetStatusLogCallback(
            [this](const std::vector<BtStatusEvent>& events) {
                PublishBehaviorTreeLog(events);
            });
        return OnTreeInitialize(options);
    }

    virtual bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& /*options*/) {
        return true;
    }

    virtual void PopulateBlackboard(const BT::Blackboard::Ptr& /*blackboard*/) {}

    virtual void OnTreeTick() {}

    void EnsureBehaviorTreeLogWriter() {
        if (bt_log_writer_ || !node_) {
            return;
        }
        bt_log_writer_ =
            node_->template CreateWriter<::automsgs::msgs::nav_msgs::BehaviorTreeLog>(
                "/behavior_tree/log");
    }

    void PublishBehaviorTreeLog(const std::vector<BtStatusEvent>& events) {
        if (!bt_log_writer_ || events.empty()) {
            return;
        }
        ::automsgs::msgs::nav_msgs::BehaviorTreeLog log;
        const auto now = ::automsgs::msgs::builtin_interfaces::Now();
        *log.mutable_timestamp() = now;
        for (const BtStatusEvent& event : events) {
            auto* entry = log.add_event_log();
            if (event.timestamp_ns > 0) {
                entry->mutable_timestamp()->set_sec(
                    static_cast<int32_t>(event.timestamp_ns / 1000000000LL));
                entry->mutable_timestamp()->set_nanosec(
                    static_cast<uint32_t>(event.timestamp_ns % 1000000000LL));
            } else {
                *entry->mutable_timestamp() = now;
            }
            // Prefer instance / path; append registration id for Autoviz matching.
            if (!event.registration_id.empty() &&
                event.node_name.find(event.registration_id) == std::string::npos) {
                entry->set_node_name(event.node_name + "/" + event.registration_id);
            } else {
                entry->set_node_name(event.node_name);
            }
            entry->set_previous_status(event.previous_status);
            entry->set_current_status(event.current_status);
        }
        bt_log_writer_->Write(log);
    }

    [[nodiscard]] const std::shared_ptr<autolink::Node>& node() const {
        return node_;
    }

    [[nodiscard]] const navigation::NavigationClient::Ptr& navigation() const {
        return navigation_client_;
    }

    // Compatibility accessors for existing apps.
    [[nodiscard]] const std::shared_ptr<autolink::Node>& autolink_node() const {
        return node();
    }
    [[nodiscard]] const navigation::NavigationClient::Ptr&
    shared_navigation() const {
        return navigation();
    }

    [[nodiscard]] const BtProfile& profile() const { return profile_; }

    [[nodiscard]] const std::string& config_directory() const {
        return config_directory_;
    }

    BtRunner* runner() { return &runner_; }

    bool StartTree(const std::string& tree_xml_path = {}) {
        std::string path = tree_xml_path;
        if (path.empty()) {
            path = profile_.DefaultTreePath(config_directory_);
        } else if (!std::filesystem::exists(path)) {
            // ResolveTreeForGoal may already return config-relative paths.
            // Only join config_directory when the given path is missing.
            const std::string resolved =
                profile_.ResolvePath(config_directory_, path);
            if (std::filesystem::exists(resolved)) {
                path = resolved;
            }
        }
        if (path.empty() || !std::filesystem::exists(path)) {
            return false;
        }
        return runner_.Run(path);
    }

    void StopTree() {
        // Always cancel: after Succeeded/Failed IsRunning() is false but a new
        // goal must still reset runner state before StartTree.
        runner_.Cancel();
    }

    bool PauseTree() { return runner_.Pause(); }

    bool ResumeTree() { return runner_.Resume(); }

    [[nodiscard]] std::string PickTreePath(
        const std::string& override_path,
        const std::string& alternate_relative) const {
        if (!override_path.empty()) {
            return profile_.ResolvePath(config_directory_, override_path);
        }
        if (!alternate_relative.empty()) {
            return profile_.ResolvePath(config_directory_, alternate_relative);
        }
        return profile_.DefaultTreePath(config_directory_);
    }

private:
    BtProfile profile_;
    std::string config_directory_;
    std::shared_ptr<autolink::Node> node_;
    navigation::NavigationClient::Ptr navigation_client_;
    BtRunner runner_;
    std::shared_ptr<::autolink::Writer<::automsgs::msgs::nav_msgs::BehaviorTreeLog>>
        bt_log_writer_;
};

}  // namespace task
}  // namespace autonomy
