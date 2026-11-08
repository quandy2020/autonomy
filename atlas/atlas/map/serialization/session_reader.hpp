/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_SERIALIZATION_SESSION_READER_HPP_
#define ATLAS_MAP_SERIALIZATION_SESSION_READER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "map/serialization/voxel_map_serializer.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {
namespace serialization {

/** @brief Reads pose and cloud entries from the current session directory
 * format. */
class SessionReader {
 public:
  bool ReadPoses(const std::string& base_path, const std::string& session_name,
                 std::vector<SessionPoseEntry>* poses,
                 std::string* error) const;
  bool ReadCloud(const std::string& base_path, const std::string& session_name,
                 std::size_t scan_index, sensor::PointCloudXYZI* cloud,
                 std::string* error) const;
};

}  // namespace serialization
}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_SERIALIZATION_SESSION_READER_HPP_
