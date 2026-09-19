/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_map_handlers.hpp
 * @brief MapService RpcHandlers: mapping lifecycle + map CRUD / metadata.
 *
 * @details
 * All handlers are unary and forward to Context::map_service() (MapServiceStub).
 * Live occupancy grids come from @c kMapChannel (`/map`); GoalChannel load /
 * set-current is delegated inside MapServiceStub → MappingStub
 * (@c kMappingGoal / @c kMappingFeedback). Handlers never touch Autolink.
 *
 * Generated types (SMART_PTR via macros):
 * - RpcStartMappingHandler / RpcCancelMappingHandler / RpcDeleteMapHandler /
 *   RpcSetCurrentMapHandler — BRIDGE_STATUS
 * - RpcFinishMappingHandler / RpcGetMapHandler / RpcGetMapMetadataHandler /
 *   RpcSaveMapHandler — BRIDGE_UNARY
 * - RpcGetMappingStatusHandler / RpcListMapsHandler — BRIDGE_GET
 *
 * @par Invariants
 * - Start / Cancel / Delete / SetCurrent return Status from MapServiceStub.
 * - Finish / Get / Save return stub response protobufs.
 * - ListMaps / GetMappingStatus are parameterless GetHandler paths.
 * - Ownership: per-RPC handlers; MapServiceStub owned by Context.
 * - Threading: gRPC completion queue; stub serializes on its mutex_.
 *
 * @see MapServiceStub
 * @see MappingStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief MapService/StartMapping — begin session (MAPPING_BUSY if active). */
BRIDGE_STATUS(RpcStartMappingHandler,
                  ::automsgs::rpcs::mapping::StartMappingRequest,
                  "/automsgs.rpcs.mapping.MapService/StartMapping",
                  &Context::map_service, &clients::MapServiceStub::StartMapping);

/** @brief MapService/FinishMapping — end session; optional persist to catalog. */
BRIDGE_UNARY(RpcFinishMappingHandler,
                 ::automsgs::rpcs::mapping::FinishMappingRequest,
                 ::automsgs::rpcs::mapping::FinishMappingResponse,
                 "/automsgs.rpcs.mapping.MapService/FinishMapping",
                 &Context::map_service, &clients::MapServiceStub::FinishMapping);

/** @brief MapService/CancelMapping — force session IDLE. */
BRIDGE_STATUS(RpcCancelMappingHandler,
                  ::automsgs::rpcs::mapping::CancelMappingRequest,
                  "/automsgs.rpcs.mapping.MapService/CancelMapping",
                  &Context::map_service, &clients::MapServiceStub::CancelMapping);

/** @brief MapService/GetMappingStatus — session snapshot. */
BRIDGE_GET(
    RpcGetMappingStatusHandler, ::automsgs::rpcs::mapping::GetMappingStatusRequest,
    ::automsgs::rpcs::mapping::MappingStatus,
    "/automsgs.rpcs.mapping.MapService/GetMappingStatus", &Context::map_service,
    &clients::MapServiceStub::GetMappingStatus);

/** @brief MapService/ListMaps — in-memory catalog summaries. */
BRIDGE_GET(RpcListMapsHandler, ::automsgs::rpcs::mapping::ListMapsRequest,
                       ::automsgs::rpcs::mapping::ListMapsResponse,
                       "/automsgs.rpcs.mapping.MapService/ListMaps",
                       &Context::map_service, &clients::MapServiceStub::ListMaps);

/** @brief MapService/GetMap — catalog or live `/map` OccupancyGrid. */
BRIDGE_UNARY(RpcGetMapHandler, ::automsgs::rpcs::mapping::GetMapRequest,
                 ::automsgs::rpcs::mapping::GetMapResponse,
                 "/automsgs.rpcs.mapping.MapService/GetMap", &Context::map_service,
                 &clients::MapServiceStub::GetMap);

/** @brief MapService/GetMapMetadata — OccupancyGrid.info for a map key. */
BRIDGE_UNARY(RpcGetMapMetadataHandler,
                 ::automsgs::rpcs::mapping::GetMapMetadataRequest,
                 ::automsgs::rpcs::mapping::GetMapMetadataResponse,
                 "/automsgs.rpcs.mapping.MapService/GetMapMetadata",
                 &Context::map_service, &clients::MapServiceStub::GetMapMetadata);

/** @brief MapService/SaveMap — insert into catalog (+ optional MappingStub load). */
BRIDGE_UNARY(RpcSaveMapHandler, ::automsgs::rpcs::mapping::SaveMapRequest,
                 ::automsgs::rpcs::mapping::SaveMapResponse,
                 "/automsgs.rpcs.mapping.MapService/SaveMap", &Context::map_service,
                 &clients::MapServiceStub::SaveMap);

/** @brief MapService/DeleteMap — erase catalog entry by identifier. */
BRIDGE_STATUS(RpcDeleteMapHandler, ::automsgs::rpcs::mapping::DeleteMapRequest,
                  "/automsgs.rpcs.mapping.MapService/DeleteMap",
                  &Context::map_service, &clients::MapServiceStub::DeleteMap);

/** @brief MapService/SetCurrentMap — select catalog entry and reload. */
BRIDGE_STATUS(RpcSetCurrentMapHandler,
                  ::automsgs::rpcs::mapping::SetCurrentMapRequest,
                  "/automsgs.rpcs.mapping.MapService/SetCurrentMap",
                  &Context::map_service, &clients::MapServiceStub::SetCurrentMap);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
