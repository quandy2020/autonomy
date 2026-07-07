/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/task/apps/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/apps/behavior_tree/bt_profile.hpp"
#include "autonomy/task/apps/behavior_tree/bt_runner.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"
#include "autonomy/task/apps/task_app_base.hpp"
#include "autonomy/task/proto/task_options.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

/**
 * Base class for task apps driven by a behavior tree.
 *
 * Resolves BtProfile from TaskServerOptions + GetTaskType(), then runs BtRunner.
 */
template <typename GoalT, typename FeedbackT, typename ResultT>
class BtTaskApp : public TypedTaskAppBase<GoalT, FeedbackT, ResultT>
{
public:
    ~BtTaskApp() override { StopTree(); }

    void SetAutolinkNode(std::shared_ptr<autolink::Node> node)
    {
        autolink_node_ = std::move(node);
    }

    void SetNavigationClient(navigation::NavigationClient::Ptr client)
    {
        navigation_client_ = std::move(client);
    }

    bool Cancel() override
    {
        StopTree();
        return TypedTaskAppBase<GoalT, FeedbackT, ResultT>::Cancel();
    }

    void Shutdown() override
    {
        StopTree();
        TypedTaskAppBase<GoalT, FeedbackT, ResultT>::Shutdown();
    }

protected:
    bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override
    {
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
        return OnTreeInitialize(options);
    }

    virtual bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& /*options*/)
    {
        return true;
    }

    virtual void PopulateBlackboard(const BT::Blackboard::Ptr& /*blackboard*/) {}

    virtual void OnTreeTick() {}

    const std::shared_ptr<autolink::Node>& autolink_node() const
    {
        return autolink_node_;
    }

    const navigation::NavigationClient::Ptr& shared_navigation() const
    {
        return navigation_client_;
    }

    const BtProfile& profile() const { return profile_; }

    const std::string& config_directory() const { return config_directory_; }

    BtRunner* runner() { return &runner_; }

    bool StartTree(const std::string& tree_xml_path = {})
    {
        std::string path = tree_xml_path;
        if (!path.empty() && path.front() != '/') {
            path = profile_.ResolvePath(config_directory_, path);
        }
        if (path.empty()) {
            path = profile_.DefaultTreePath(config_directory_);
        }
        if (path.empty()) {
            return false;
        }
        return runner_.Run(path);
    }

    void StopTree()
    {
        if (runner_.IsRunning()) {
            runner_.Cancel();
        }
    }

    bool PauseTree() { return runner_.Pause(); }

    bool ResumeTree() { return runner_.Resume(); }

    std::string PickTreePath(const std::string& override_path,
                             const std::string& alternate_relative) const
    {
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
    std::shared_ptr<autolink::Node> autolink_node_;
    navigation::NavigationClient::Ptr navigation_client_;
    BtRunner runner_;
};

}  // namespace task
}  // namespace autonomy
