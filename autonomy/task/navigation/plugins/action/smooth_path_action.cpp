/* Copyright 2026 The Openbot Authors */

#include <cmath>
#include <cstdint>
#include "autonomy/task/navigation/plugins/async_path_action_node.hpp"

namespace autonomy::task::plugins::navigation {
namespace actions = automsgs::actions;
class SmoothPathAction : public AsyncPathActionNode<actions::SmoothPathAction>
{
public:
    using AsyncPathActionNode::AsyncPathActionNode;
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("unsmoothed_path"),
            BT::OutputPort<automsgs::msgs::nav_msgs::Path>("smoothed_path"),
            BT::InputPort<std::string>("smoother_id", "simple_smoother", "plugin id"),
            BT::InputPort<double>("max_smoothing_duration", 1.0, "s"),
            BT::InputPort<bool>("check_for_collisions", false, "smoother collision check"),
            BT::InputPort<double>("max_path_reuse_age", 0.0, "use only this request's input fallback"),
            BT::OutputPort<double>("smoothing_duration"),
            BT::OutputPort<bool>("was_completed"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }
protected:
    Client& GetClient(NavClient& client) override { return client.smooth_path_client(); }
    bool BuildGoal(Goal& goal, Pose& endpoint) override
    {
        Path path;
        if (!getInput("unsmoothed_path", path) || path.poses().empty()) return false;
        endpoint = path.poses(path.poses_size()-1);
        if (endpoint.header().frame_id().empty()) {
            endpoint.mutable_header()->set_frame_id(path.header().frame_id());
        }
        *goal.mutable_path() = path;
        std::string smoother;
        double duration = 1.0;
        bool check = false;
        getInput("smoother_id", smoother);
        getInput("max_smoothing_duration", duration);
        getInput("check_for_collisions", check);
        if (!std::isfinite(duration) || duration <= 0.0 || duration > 60.0) return false;
        goal.set_smoother_id(smoother);
        *goal.mutable_max_smoothing_duration() = ToProtoDuration(duration);
        goal.set_check_for_collisions(check);
        return true;
    }
    Path InitialFallback(const Goal& goal) const override { return goal.path(); }
    bool UsePublishedPlan() const override { return false; }
    void ResetResultMetadata() override { smoothing_duration_ = 0.0; completed_ = false; }
    void RecordResult(const Result& result) override
    {
        smoothing_duration_ = result.smoothing_duration().sec() +
            1e-9*result.smoothing_duration().nanosec();
        completed_ = result.was_completed();
    }
    void OutputPath(const Path& path, bool computed) override
    {
        setOutput("smoothed_path", path);
        setOutput("smoothing_duration", computed ? smoothing_duration_ : 0.0);
        setOutput("was_completed", computed && completed_);
    }
private:
    double smoothing_duration_{0.0};
    bool completed_{false};
};
}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::SmoothPathAction>("SmoothPath");
}
