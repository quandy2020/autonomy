/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_OPTIMIZER_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_GLOBAL_MAPPING_OPTIMIZER_HPP_

#include <vector>

#include "backend/global_mapping/global_mapping_request.hpp"
#include "backend/global_mapping/global_mapping_result.hpp"
#include "map/hierarchical_bundle_adjustment.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

/** @brief Runs HBA computations over immutable backend requests. */
class GlobalMappingOptimizer {
 public:
  explicit GlobalMappingOptimizer(
      map::GlobalBundleAdjustmentConfiguration local_window_configuration,
      map::GlobalBundleAdjustmentConfiguration submap_configuration);

  GlobalMappingResult OptimizeLocalWindow(
      const GlobalMappingRequest& request,
      const std::vector<int>& session_ids);
  GlobalMappingResult OptimizeSubmaps(const GlobalMappingRequest& request,
                                      const std::vector<int>& session_ids);

 private:
  GlobalMappingResult OptimizeWindow(
      const GlobalMappingRequest& request,
      const std::vector<int>& session_ids,
      const map::GlobalBundleAdjustmentConfiguration& configuration);

  map::HierarchicalBundleAdjustment hierarchical_optimizer_;
  map::GlobalBundleAdjustmentConfiguration local_window_configuration_;
  map::GlobalBundleAdjustmentConfiguration submap_configuration_;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif
