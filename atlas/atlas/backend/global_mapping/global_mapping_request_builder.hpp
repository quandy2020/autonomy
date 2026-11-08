/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_BUILDER_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_BUILDER_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "backend/global_mapping/global_mapping_request.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

/** @brief Builds immutable HBA requests from active keyframe storage. */
class GlobalMappingRequestBuilder {
 public:
  GlobalMappingRequest BuildLocalWindowRequest(
      GlobalMappingRequest request, std::size_t first_keyframe_index,
      std::size_t maximum_keyframe_count,
      const std::vector<std::unique_ptr<map::Keyframe>>& keyframes) const;

  GlobalMappingRequest BuildSubmapRequest(
      GlobalMappingRequest request,
      const std::vector<std::unique_ptr<map::Keyframe>>& submaps) const;

 private:
  static void AddKeyframe(const map::Keyframe& keyframe,
                          std::size_t keyframe_index,
                          GlobalMappingRequest* request);
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_REQUEST_BUILDER_HPP_
