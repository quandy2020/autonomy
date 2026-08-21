/*
 * Copyright 2026 The Openbot Authors
 *
 * Feeds Autolink /tf (+ optional TransformStampeds) into process-local Buffer.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

namespace autonomy {
namespace transform {

class AutolinkTfListener
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(AutolinkTfListener)

    AutolinkTfListener() = default;
    ~AutolinkTfListener();

    /**
     * @param transform_message_topic TFMessage channel (Foxglove/autosim: /tf)
     * @param transform_stampeds_topic optional TransformStampeds channel
     * @param static_transform_message_topic optional static TFMessage (/tf_static)
     */
    bool Start(const std::shared_ptr<autolink::Node>& node,
               const std::string& transform_message_topic = "/tf",
               const std::string& transform_stampeds_topic = "tf",
               const std::string& static_transform_message_topic =
                   "/tf_static");

    void Stop();

private:
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::tf2_msgs::TFMessage>>
        transform_message_reader_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::tf2_msgs::TFMessage>>
        static_transform_message_reader_;
    std::shared_ptr<
        autolink::Reader<automsgs::msgs::geometry_msgs::TransformStampeds>>
        stampeds_reader_;
};

}  // namespace transform
}  // namespace autonomy
