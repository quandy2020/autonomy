/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcStartMappingSignature, ::automsgs::rpcs::mapping::StartMappingRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.mapping.MapService/StartMapping")

class RpcStartMappingHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcStartMappingSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::StartMappingRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcFinishMappingSignature, ::automsgs::rpcs::mapping::FinishMappingRequest,
    ::automsgs::rpcs::mapping::FinishMappingResponse,
    "/automsgs.rpcs.mapping.MapService/FinishMapping")

class RpcFinishMappingHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcFinishMappingSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::FinishMappingRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcCancelMappingSignature, ::automsgs::rpcs::mapping::CancelMappingRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.mapping.MapService/CancelMapping")

class RpcCancelMappingHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcCancelMappingSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::CancelMappingRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcGetMappingStatusSignature, ::automsgs::rpcs::mapping::GetMappingStatusRequest,
    ::automsgs::rpcs::mapping::MappingStatus,
    "/automsgs.rpcs.mapping.MapService/GetMappingStatus")

class RpcGetMappingStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcGetMappingStatusSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::GetMappingStatusRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcListMapsSignature, ::automsgs::rpcs::mapping::ListMapsRequest,
    ::automsgs::rpcs::mapping::ListMapsResponse,
    "/automsgs.rpcs.mapping.MapService/ListMaps")

class RpcListMapsHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcListMapsSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::ListMapsRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcGetMapSignature, ::automsgs::rpcs::mapping::GetMapRequest,
    ::automsgs::rpcs::mapping::GetMapResponse,
    "/automsgs.rpcs.mapping.MapService/GetMap")

class RpcGetMapHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcGetMapSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::GetMapRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcGetMapMetadataSignature, ::automsgs::rpcs::mapping::GetMapMetadataRequest,
    ::automsgs::rpcs::mapping::GetMapMetadataResponse,
    "/automsgs.rpcs.mapping.MapService/GetMapMetadata")

class RpcGetMapMetadataHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcGetMapMetadataSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::GetMapMetadataRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSaveMapSignature, ::automsgs::rpcs::mapping::SaveMapRequest,
    ::automsgs::rpcs::mapping::SaveMapResponse,
    "/automsgs.rpcs.mapping.MapService/SaveMap")

class RpcSaveMapHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSaveMapSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::SaveMapRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcDeleteMapSignature, ::automsgs::rpcs::mapping::DeleteMapRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.mapping.MapService/DeleteMap")

class RpcDeleteMapHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcDeleteMapSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::DeleteMapRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSetCurrentMapSignature, ::automsgs::rpcs::mapping::SetCurrentMapRequest,
    ::automsgs::rpcs::common::Status,
    "/automsgs.rpcs.mapping.MapService/SetCurrentMap")

class RpcSetCurrentMapHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSetCurrentMapSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::mapping::SetCurrentMapRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

