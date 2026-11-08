/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_SERIALIZATION_SESSION_DATA_HPP_
#define ATLAS_MAP_SERIALIZATION_SESSION_DATA_HPP_

#include <string>
#include <vector>

#include "map/serialization/voxel_map_serializer.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {
namespace serialization {

/** @brief Owned values needed to persist one session directory. */
struct SessionData {
  std::string name;
  std::vector<SessionPoseEntry> poses;
  std::vector<sensor::PointCloudXYZI> scan_clouds;
};

}  // namespace serialization
}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_SERIALIZATION_SESSION_DATA_HPP_
