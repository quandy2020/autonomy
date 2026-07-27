/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {

struct PickRecord {
  std::string display_name;
  std::string display_type;
  QVector3D position;
  int point_index = -1;
};

/** Maps pick handles to world-space metadata (cleared each frame). */
class PickRegistry {
 public:
  void clear();

  PickHandle registerPick(const PickRecord& record);
  const PickRecord* lookup(PickHandle handle) const;
  /** Resolve per-point handle after Ogre Pick1 pass (display + point index). */
  PickHandle lookupByDisplayAndPointIndex(const std::string& display_name,
                                          int point_index) const;

 private:
  PickHandleAllocator allocator_;
  std::unordered_map<PickHandle, PickRecord> records_;
};

}  // namespace common
}  // namespace autoviz
