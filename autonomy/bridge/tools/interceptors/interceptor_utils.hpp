/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file interceptor_utils.hpp
 * @brief Shared helpers for Bridge server interceptors.
 */

#pragma once

#include <string>
#include <unordered_map>

#include "grpcpp/support/interceptor.h"
#include "grpcpp/support/string_ref.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Convert gRPC recv metadata multimap to lower-case key→value map.
 *
 * Only the first value per key is kept.
 */
inline std::unordered_map<std::string, std::string> FlattenMetadata(
    const std::multimap<::grpc::string_ref, ::grpc::string_ref>* metadata) {
    std::unordered_map<std::string, std::string> out;
    if (metadata == nullptr) {
        return out;
    }
    for (const auto& kv : *metadata) {
        std::string key(kv.first.data(), kv.first.size());
        for (char& c : key) {
            if (c >= 'A' && c <= 'Z') {
                c = static_cast<char>(c - 'A' + 'a');
            }
        }
        if (out.find(key) == out.end()) {
            out.emplace(std::move(key),
                        std::string(kv.second.data(), kv.second.size()));
        }
    }
    return out;
}

/**
 * @brief Lookup a key in a flattened metadata map (already lower-case keys).
 */
inline std::string LookupMetadata(
    const std::unordered_map<std::string, std::string>& metadata,
    const char* key) {
    const auto it = metadata.find(key);
    return it == metadata.end() ? std::string{} : it->second;
}

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
