/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/pick_registry.hpp"

namespace autoviz {
namespace common {

void PickRegistry::clear() {
  allocator_.reset();
  records_.clear();
}

PickHandle PickRegistry::registerPick(const PickRecord& record) {
  const PickHandle handle = allocator_.allocate();
  records_[handle] = record;
  return handle;
}

const PickRecord* PickRegistry::lookup(PickHandle handle) const {
  if (handle == kInvalidPickHandle) {
    return nullptr;
  }
  const auto it = records_.find(handle);
  return it == records_.end() ? nullptr : &it->second;
}

PickHandle PickRegistry::lookupByDisplayAndPointIndex(
    const std::string& display_name, int point_index) const {
  for (const auto& pair : records_) {
    if (pair.second.display_name == display_name &&
        pair.second.point_index == point_index) {
      return pair.first;
    }
  }
  return kInvalidPickHandle;
}

}  // namespace common
}  // namespace autoviz
