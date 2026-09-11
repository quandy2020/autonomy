/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/latest_message_cache.hpp"
#include "autonomy/bridge/grpc/clients/map_stub.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include <automsgs/rpcs/localization.pb.h>
#include <automsgs/task/localization.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Localization facade: cached pose/feedback + initial-pose via MapStub.
 */
class LocalizationStub
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationStub)

    /**
     * @brief Construct localization readers.
     * @param[in] node Autolink node.
     * @param[in] map_stub Map stub used for SET_INITIAL_POSE.
     */
    LocalizationStub(std::shared_ptr<autolink::Node> node,
                     std::shared_ptr<MapStub> map_stub);

    /**
     * @brief Return the latest localized pose.
     * @param[in] request Optional map-frame override.
     */
    ::automsgs::rpcs::localization::GetPoseResponse GetPose(
        const ::automsgs::rpcs::localization::GetPoseRequest& request) const;

    /** @brief Return localization state / confidence. */
    ::automsgs::rpcs::localization::LocalizationStatus GetStatus() const;

    /**
     * @brief Seed AMCL / map initial pose through MapStub.
     * @param[in] request Initial pose request.
     */
    ::automsgs::rpcs::common::Status SetInitialPose(
        const ::automsgs::rpcs::localization::SetInitialPoseRequest& request);

private:
    using PoseMsg = ::automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped;
    using FeedbackMsg = ::autonomy::task::proto::LocalizationFeedback;

    void HandleFeedback(const FeedbackMsg& feedback);

    std::shared_ptr<MapStub> map_stub_;
    LatestMessageCache<PoseMsg> pose_cache_;
    LatestMessageCache<FeedbackMsg> feedback_cache_;

    mutable std::mutex mutex_;
    ::automsgs::rpcs::localization::LocalizationState state_{
        ::automsgs::rpcs::localization::LOCALIZATION_STATE_UNKNOWN};
    float quality_{0.f};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
