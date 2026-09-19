/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file map_service_stub.hpp
 * @brief MapServiceStub: MapService unary facade over MappingStub + in-memory catalog.
 *
 * @details
 * Not a GoalChannel client itself. Owns session state (MAPPING_STATE_*) and an
 * in-memory OccupancyGrid catalog keyed by map_identifier / map_name. Live grid
 * samples are cached from @c kMapChannel (`/map` in bridge/constants.hpp) via
 * LatestMessageCache. GoalChannel writes for load / set-current are delegated
 * to MappingStub (::HandleStart), which uses @c kMappingGoal /
 * @c kMappingFeedback.
 *
 * RPC surface (all unary):
 * - StartMapping / FinishMapping / CancelMapping — session lifecycle
 * - GetMappingStatus / ListMaps / GetMap / GetMapMetadata — reads
 * - SaveMap / DeleteMap / SetCurrentMap — catalog mutations (+ optional load)
 *
 * @par Invariants
 * - StartMapping rejects when session_state_ is already MAPPING (MAPPING_BUSY).
 * - FinishMapping optionally persists live grid into catalog_ when persist().
 * - GetMap prefers catalog key, then falls back to live_map_cache_.
 * - mutex_ guards session_* / catalog_ / current_map_id_; live cache is separate.
 *
 * @par Ownership
 * DomainBundle UniquePtr; holds non-owning MappingStub* (Bundle owns MappingStub).
 *
 * @par Threading
 * public methods take mutex_ (except live cache internals); do not
 * call MappingStub::HandleStart while holding locks that MappingStub may need
 * beyond the short critical sections used here (HandleStart is invoked under
 * mutex_ today — keep MappingStub work non-blocking).
 *
 * @see MappingStub
 * @see rpc_map_handlers.hpp
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/latest_message_cache.hpp"
#include "autonomy/bridge/grpc/clients/mapping_stub.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/rpcs/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief MapService unary facade (session + in-memory catalog). Not GoalChannel.
 *
 * @details
 * Implements MapService semantics for cloud / app clients: start a
 * mapping session, finish/cancel it, CRUD maps in an in-process catalog, and
 * read live `/map` when a catalog miss occurs. Persist paths copy from
 * live_map_cache_ rather than reading disk.
 *
 * @par Threading
 * mutex_ serializes session and catalog; LatestMessageCache is
 * updated on the Autolink reader thread.
 *
 * @par Ownership
 * UniquePtr held by DomainBundle; non-owning MappingStub* for GoalChannel.
 *
 * @note Catalog is process-local and ephemeral — not a substitute for map DB.
 * @warning Concurrent StartMapping while MAPPING returns MAPPING_BUSY.
 * @see MappingStub
 * @see rpc_map_handlers.hpp
 */
class MapServiceStub
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MapServiceStub)

    /**
     * @brief Bind live `/map` cache and non-owning MappingStub for GoalChannel.
     *
     * @param[in] node         Autolink node for @c kMapChannel reader.
     * @param[in] mapping_stub Non-owning MappingStub (may be null; Start/Save degrade).
     *                         Must outlive this MapServiceStub (DomainBundle order).
     */
    MapServiceStub(std::shared_ptr<autolink::Node> node,
                   MappingStub* mapping_stub);

    /**
     * @brief Begin a mapping session and optionally notify MappingStub.
     *
     * @param[in] request StartMappingRequest (goal_id, map_name).
     * @return            OkStatus on start; MAPPING_BUSY if already mapping.
     */
    ::automsgs::rpcs::common::Status StartMapping(
        const ::automsgs::rpcs::mapping::StartMappingRequest& request);

    /**
     * @brief End an active mapping session; optionally persist live grid.
     *
     * @param[in] request FinishMappingRequest (persist flag, goal_id).
     * @return            FinishMappingResponse with map_identifier and status.
     */
    ::automsgs::rpcs::mapping::FinishMappingResponse FinishMapping(
        const ::automsgs::rpcs::mapping::FinishMappingRequest& request);

    /**
     * @brief Force session to IDLE (does not erase catalog entries).
     *
     * @param[in] request CancelMappingRequest (fields unused today).
     * @return            OkStatus("mapping cancelled").
     */
    ::automsgs::rpcs::common::Status CancelMapping(
        const ::automsgs::rpcs::mapping::CancelMappingRequest& request);

    /**
     * @brief Snapshot of session state / goal / map identifiers.
     *
     * @return MappingStatus under mutex_.
     */
    ::automsgs::rpcs::mapping::MappingStatus GetMappingStatus() const;

    /**
     * @brief Enumerate in-memory catalog summaries (resolution / size / current).
     *
     * @return ListMapsResponse with one summary per catalog_ entry.
     */
    ::automsgs::rpcs::mapping::ListMapsResponse ListMaps() const;

    /**
     * @brief Fetch a named catalog map or the live `/map` sample.
     *
     * @param[in] request GetMapRequest (map_identifier and/or map_name).
     * @return            GetMapResponse with OccupancyGrid or NOT_FOUND.
     */
    ::automsgs::rpcs::mapping::GetMapResponse GetMap(
        const ::automsgs::rpcs::mapping::GetMapRequest& request) const;

    /**
     * @brief Metadata-only view of GetMap (info / map_identifier).
     *
     * @param[in] request GetMapMetadataRequest.
     * @return            GetMapMetadataResponse; status mirrors underlying GetMap.
     */
    ::automsgs::rpcs::mapping::GetMapMetadataResponse GetMapMetadata(
        const ::automsgs::rpcs::mapping::GetMapMetadataRequest& request) const;

    /**
     * @brief Insert request.map() or live grid into catalog_ and set current.
     *
     * Also triggers MappingStub::HandleStart with the saved map name when the
     * stub is present.
     *
     * @param[in] request SaveMapRequest (identifier / name / optional map).
     * @return            SaveMapResponse with map_identifier or INVALID_ARGUMENT.
     */
    ::automsgs::rpcs::mapping::SaveMapResponse SaveMap(
        const ::automsgs::rpcs::mapping::SaveMapRequest& request);

    /**
     * @brief Erase a catalog entry by map_identifier.
     *
     * @param[in] request DeleteMapRequest (map_identifier required).
     * @return            OkStatus or NOT_FOUND; clears current_map_id_ if it matched.
     */
    ::automsgs::rpcs::common::Status DeleteMap(
        const ::automsgs::rpcs::mapping::DeleteMapRequest& request);

    /**
     * @brief Mark an existing catalog entry as current and reload via MappingStub.
     *
     * @param[in] request SetCurrentMapRequest (map_identifier must exist).
     * @return            OkStatus or NOT_FOUND.
     */
    ::automsgs::rpcs::common::Status SetCurrentMap(
        const ::automsgs::rpcs::mapping::SetCurrentMapRequest& request);

private:
    /**
     * @brief OccupancyGrid message type for live `/map` and catalog entries.
     */
    using GridMsg = ::automsgs::msgs::map_msgs::OccupancyGrid;

    /**
     * @brief Non-owning MappingStub for GoalChannel load / set-current.
     *
     * @details DomainBundle owns MappingStub and constructs MapServiceStub
     * with `mapping_.get()`. May be null (Start/Save degrade gracefully).
     * Must outlive this MapServiceStub.
     */
    MappingStub* mapping_stub_{nullptr};

    /**
     * @brief Latest OccupancyGrid sample from @c kMapChannel (`/map`).
     *
     * @details Internally synchronized; used by GetMap fallback and
     * FinishMapping / SaveMap persist paths.
     */
    LatestMessageCache<GridMsg> live_map_cache_;

    /**
     * @brief Serializes session_* / catalog_ / current_map_id_ accessors.
     *
     * @details Does not cover live_map_cache_ internals.
     */
    mutable std::mutex mutex_;

    /**
     * @brief Current mapping session state (IDLE / MAPPING / …).
     */
    ::automsgs::rpcs::mapping::MappingState session_state_{
        ::automsgs::rpcs::mapping::MAPPING_STATE_IDLE};

    /**
     * @brief goal_id of the active StartMapping session (if any).
     */
    std::string session_goal_id_;

    /**
     * @brief map_name associated with the active mapping session.
     */
    std::string session_map_name_;

    /**
     * @brief Catalog key of the map marked current (may be empty).
     */
    std::string current_map_id_;

    /**
     * @brief In-memory OccupancyGrid catalog keyed by map_identifier.
     *
     * @details Process-local and ephemeral — not a durable map database.
     */
    std::unordered_map<std::string, GridMsg> catalog_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
