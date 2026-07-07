/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/task/proto/localization.pb.h"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace localization {

constexpr char kLocalizationClientBlackboardKey[] = "localization_client";

class LocalizationClient
{
public:
    using Ptr = std::shared_ptr<LocalizationClient>;

    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    void ApplyGoal(const ::autonomy::task::proto::LocalizationGoal& goal);

    bool StartLocalization();
    void StopLocalization();
    bool IsRunning() const { return running_; }
    bool IsReady() const { return ready_; }

    ::autonomy::task::proto::LocalizationAlgorithm algorithm() const
    {
        return algorithm_;
    }
    float localization_quality() const { return localization_quality_; }

    explicit LocalizationClient(std::shared_ptr<autolink::Node> node);

private:
    std::shared_ptr<autolink::Node> node_;
    ::autonomy::task::proto::LocalizationAlgorithm algorithm_{
        ::autonomy::task::proto::LOCALIZATION_ALGO_UNSPECIFIED};
    std::string configuration_directory_;
    std::string configuration_basename_;
    bool running_{false};
    bool ready_{false};
    float localization_quality_{0.f};
};

}  // namespace localization
}  // namespace task
}  // namespace autonomy
