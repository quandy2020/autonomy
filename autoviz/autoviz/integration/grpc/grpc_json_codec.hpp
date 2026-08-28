/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <google/protobuf/message.h>

namespace autoviz {
namespace integration {
namespace grpc_client {

/** Parse JSON into a DynamicMessage matching |descriptor|.
 *  On failure returns nullptr and writes a message into |error| if non-null. */
std::unique_ptr<google::protobuf::Message> JsonToDynamicMessage(
    const std::string& json, const google::protobuf::Descriptor* descriptor,
    std::string* error);

/** Serialize |message| to JSON. When |include_defaults| is true, always print
 *  primitive fields (via protobuf_json_compat). */
bool DynamicMessageToJson(const google::protobuf::Message& message,
                          bool include_defaults, std::string* json_out,
                          std::string* error);

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
