/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pick_registry.hpp
 * @brief Per-frame map from @ref PickHandle to world-space pick metadata.
 *
 * Displays register picks while drawing; tools and selection code look them
 * up after a GPU/CPU pick. Cleared at the start of each frame.
 *
 * @see PickHandle
 * @see SelectionManager
 * @see ToolContext
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {

/**
 * @struct PickRecord
 * @brief World-space metadata associated with one pickable object/point.
 */
struct PickRecord {
  /** Owning display instance name (UI / selection label). */
  std::string display_name;

  /** Display type id (e.g. @c "PointCloud2"). */
  std::string display_type;

  /** World position of the pick in the fixed frame. */
  QVector3D position;

  /**
   * Optional point index within a cloud/list (−1 if not applicable).
   * Used by @ref PickRegistry::lookupByDisplayAndPointIndex after Ogre Pick1.
   */
  int point_index = -1;
};

/**
 * @class PickRegistry
 * @brief Maps pick handles to @ref PickRecord for the current render frame.
 *
 * @note Call @ref clear() each frame before displays re-register picks.
 */
class PickRegistry {
 public:
  /**
   * @brief Removes all records and resets the handle allocator.
   */
  void clear();

  /**
   * @brief Registers a pick and returns its new handle.
   *
   * @param record Metadata to store for this pick.
   * @return Freshly allocated @ref PickHandle.
   */
  PickHandle registerPick(const PickRecord& record);

  /**
   * @brief Looks up metadata for a handle.
   *
   * @param handle Handle from a pick buffer or CPU pick.
   * @return Pointer to the stored record, or @c nullptr if unknown.
   */
  const PickRecord* lookup(PickHandle handle) const;

  /**
   * @brief Resolves a per-point handle after an Ogre Pick1 pass.
   *
   * Matches @p display_name and @p point_index against registered records.
   *
   * @param display_name Display that owns the point.
   * @param point_index Index within that display's point list.
   * @return Matching handle, or @ref kInvalidPickHandle if not found.
   */
  PickHandle lookupByDisplayAndPointIndex(const std::string& display_name,
                                          int point_index) const;

 private:
  /** Allocator providing unique handles for this frame. */
  PickHandleAllocator allocator_;

  /** Handle → record map (cleared each frame). */
  std::unordered_map<PickHandle, PickRecord> records_;
};

}  // namespace common
}  // namespace autoviz
