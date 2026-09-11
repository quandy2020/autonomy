/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/latest_message_cache.hpp"
#include "autonomy/bridge/grpc/clients/map_stub.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/rpcs/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief MapService facade: MapStub session RPCs + in-memory map catalog.
 */
class MapServiceStub
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(MapServiceStub)

    /**
     * @brief Construct map catalog and live-map cache.
     * @param[in] node Autolink node.
     * @param[in] map_stub Underlying map command stub.
     */
    MapServiceStub(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<MapStub> map_stub);

    ::automsgs::rpcs::common::Status StartMapping(
        const ::automsgs::rpcs::mapping::StartMappingRequest& request);
    ::automsgs::rpcs::mapping::FinishMappingResponse FinishMapping(
        const ::automsgs::rpcs::mapping::FinishMappingRequest& request);
    ::automsgs::rpcs::common::Status CancelMapping(
        const ::automsgs::rpcs::mapping::CancelMappingRequest& request);
    ::automsgs::rpcs::mapping::MappingStatus GetMappingStatus() const;

    ::automsgs::rpcs::mapping::ListMapsResponse ListMaps() const;
    ::automsgs::rpcs::mapping::GetMapResponse GetMap(
        const ::automsgs::rpcs::mapping::GetMapRequest& request) const;
    ::automsgs::rpcs::mapping::GetMapMetadataResponse GetMapMetadata(
        const ::automsgs::rpcs::mapping::GetMapMetadataRequest& request) const;
    ::automsgs::rpcs::mapping::SaveMapResponse SaveMap(
        const ::automsgs::rpcs::mapping::SaveMapRequest& request);
    ::automsgs::rpcs::common::Status DeleteMap(
        const ::automsgs::rpcs::mapping::DeleteMapRequest& request);
    ::automsgs::rpcs::common::Status SetCurrentMap(
        const ::automsgs::rpcs::mapping::SetCurrentMapRequest& request);

private:
    using GridMsg = ::automsgs::msgs::map_msgs::OccupancyGrid;

    std::shared_ptr<MapStub> map_stub_;
    LatestMessageCache<GridMsg> live_map_cache_;

    mutable std::mutex mutex_;
    ::automsgs::rpcs::mapping::MappingState session_state_{
        ::automsgs::rpcs::mapping::MAPPING_STATE_IDLE};
    std::string session_goal_id_;
    std::string session_map_name_;
    std::string current_map_id_;
    std::unordered_map<std::string, GridMsg> catalog_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
