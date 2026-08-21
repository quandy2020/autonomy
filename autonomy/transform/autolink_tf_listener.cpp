/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/transform/autolink_tf_listener.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy {
namespace transform {
namespace {

namespace tf2_pb = ::automsgs::msgs::tf2_msgs;

}  // namespace

AutolinkTfListener::~AutolinkTfListener() { Stop(); }

bool AutolinkTfListener::Start(const std::shared_ptr<autolink::Node>& node,
                               const std::string& transform_message_topic,
                               const std::string& transform_stampeds_topic,
                               const std::string& static_transform_message_topic)
{
    Stop();
    if (!node) {
        return false;
    }

    auto* buffer = Buffer::Instance();
    if (buffer->Init() != 0) {
        AWARN << "AutolinkTfListener: Buffer::Init returned non-zero";
    }
    // autosim (and typical stacks) use identity map→odom until localization
    // publishes a real transform. Seed so costmap canTransform(map, base_*)
    // works before the first /tf_static arrives.
    SeedIdentityMapToOdom(buffer, "autolink_tf_seed");

    node_ = node;

    auto bind_tf_message =
        [buffer](const std::string& topic, bool is_static,
                 std::shared_ptr<autolink::Reader<tf2_pb::TFMessage>>* out,
                 const std::shared_ptr<autolink::Node>& node_ref) {
            if (topic.empty() || out == nullptr || !node_ref) {
                return;
            }
            *out = node_ref->CreateReader<tf2_pb::TFMessage>(
                topic,
                [buffer, is_static](const std::shared_ptr<tf2_pb::TFMessage>& msg) {
                    if (!msg) {
                        return;
                    }
                    ApplyTfMessageToBuffer(buffer, *msg, "autolink_tf",
                                           is_static);
                });
            if (!*out) {
                AWARN << "AutolinkTfListener: failed to subscribe " << topic;
            } else {
                AINFO << "AutolinkTfListener: TFMessage on " << topic
                      << (is_static ? " (static)" : "");
            }
        };

    bind_tf_message(transform_message_topic, false, &transform_message_reader_,
                    node_);
    bind_tf_message(static_transform_message_topic, true,
                    &static_transform_message_reader_, node_);

    if (!transform_stampeds_topic.empty()) {
        stampeds_reader_ = node_->CreateReader<
            automsgs::msgs::geometry_msgs::TransformStampeds>(
            transform_stampeds_topic,
            [buffer](const std::shared_ptr<
                     automsgs::msgs::geometry_msgs::TransformStampeds>& msg) {
                if (!msg) {
                    return;
                }
                for (const auto& trans : msg->transforms()) {
                    ApplyTransformStampedToBuffer(buffer, trans, "autolink_tf",
                                                  false);
                }
            });
        if (!stampeds_reader_) {
            AWARN << "AutolinkTfListener: TransformStampeds subscribe failed: "
                  << transform_stampeds_topic;
        } else {
            AINFO << "AutolinkTfListener: TransformStampeds on "
                  << transform_stampeds_topic;
        }
    }

    return static_cast<bool>(transform_message_reader_ ||
                             static_transform_message_reader_ ||
                             stampeds_reader_);
}

void AutolinkTfListener::Stop()
{
    transform_message_reader_.reset();
    static_transform_message_reader_.reset();
    stampeds_reader_.reset();
    node_.reset();
}

}  // namespace transform
}  // namespace autonomy
