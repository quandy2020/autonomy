/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_SERIALIZATION_SESSION_WRITER_HPP_
#define ATLAS_MAP_SERIALIZATION_SESSION_WRITER_HPP_

#include <string>

#include "map/serialization/session_data.hpp"

namespace atlas {
namespace map {
namespace serialization {

/** @brief Writes one pose and body-frame cloud per scan. */
class SessionWriter {
 public:
  bool Write(const std::string& base_path, const SessionData& session,
             std::string* error) const;
};

}  // namespace serialization
}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_SERIALIZATION_SESSION_WRITER_HPP_
