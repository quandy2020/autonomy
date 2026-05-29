/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace planning_msgs {

proto::planning_msgs::Path ToProto(const Path& data) {
    proto::planning_msgs::Path proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& pose : data.poses) {
        *proto.add_poses() = geometry_msgs::ToProto(pose);
    }
    return proto;
}

Path FromProto(const proto::planning_msgs::Path& proto) {
    Path data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.poses.reserve(static_cast<size_t>(proto.poses_size()));
    for (const auto& pose : proto.poses()) {
        data.poses.push_back(geometry_msgs::FromProto(pose));
    }
    return data;
}

}  // namespace planning_msgs
}  // namespace commsgs
}  // namespace autonomy
