/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/task_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace task_msgs {

proto::task_msgs::SpeedLimit ToProto(const SpeedLimit& data) {
    proto::task_msgs::SpeedLimit proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_percentage(data.percentage);
    proto.set_speed_limit(data.speed_limit);
    return proto;
}

SpeedLimit FromProto(const proto::task_msgs::SpeedLimit& proto) {
    return {std_msgs::FromProto(proto.header()), proto.percentage(),
            proto.speed_limit()};
}

}  // namespace task_msgs
}  // namespace commsgs
}  // namespace autonomy
