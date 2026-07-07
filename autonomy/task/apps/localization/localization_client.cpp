/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/localization/localization_client.hpp"

#include "autonomy/common/logging.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace localization {
namespace {

std::weak_ptr<LocalizationClient> g_shared_client;

}  // namespace

LocalizationClient::LocalizationClient(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node))
{
}

LocalizationClient::Ptr LocalizationClient::Create(
    std::shared_ptr<autolink::Node> node)
{
    if (!node) {
        return nullptr;
    }
    return std::make_shared<LocalizationClient>(std::move(node));
}

void LocalizationClient::SetShared(const Ptr& client)
{
    g_shared_client = client;
}

LocalizationClient::Ptr LocalizationClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kLocalizationClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

LocalizationClient::Ptr LocalizationClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
}

void LocalizationClient::ApplyGoal(
    const ::autonomy::task::proto::LocalizationGoal& goal)
{
    algorithm_ = goal.algorithm();
    configuration_directory_ = goal.configuration_directory();
    configuration_basename_ = goal.configuration_basename();
    ready_ = false;
    localization_quality_ = 0.f;
}

bool LocalizationClient::StartLocalization()
{
    running_ = true;
    ready_ = true;
    localization_quality_ = 1.f;
    AINFO << "LocalizationClient: started (algorithm=" << algorithm_ << ")";
    return true;
}

void LocalizationClient::StopLocalization()
{
    if (!running_) {
        return;
    }
    running_ = false;
    ready_ = false;
    localization_quality_ = 0.f;
    AINFO << "LocalizationClient: stopped";
}

}  // namespace localization
}  // namespace task
}  // namespace autonomy
