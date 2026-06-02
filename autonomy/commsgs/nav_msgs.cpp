/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/nav_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace nav_msgs {

proto::nav_msgs::SpeedLimit ToProto(const SpeedLimit& data) {
    proto::nav_msgs::SpeedLimit proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_percentage(data.percentage);
    proto.set_speed_limit(data.speed_limit);
    return proto;
}

SpeedLimit FromProto(const proto::nav_msgs::SpeedLimit& proto) {
    SpeedLimit data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.percentage = proto.percentage();
    data.speed_limit = proto.speed_limit();
    return data;
}

}  // namespace nav_msgs
}  // namespace commsgs
}  // namespace autonomy
