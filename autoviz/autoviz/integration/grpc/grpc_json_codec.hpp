/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>

namespace autoviz {
namespace integration {
namespace grpc_client {

/** Parse JSON into a DynamicMessage matching |descriptor| using |factory|.
 *  Prefer store.factory() (Task 5+) so prototypes stay tied to the store pool.
 *  On failure returns nullptr and writes a message into |error| if non-null. */
std::unique_ptr<google::protobuf::Message> JsonToDynamicMessage(
    const std::string& json, const google::protobuf::Descriptor* descriptor,
    google::protobuf::DynamicMessageFactory& factory, std::string* error);

/** Convenience overload using a process-lifetime static factory (unit tests). */
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
