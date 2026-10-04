/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file selection_handler.hpp
 * @brief Per-object selection handlers and handle→handler registry.
 *
 * Mirrors @c rviz_common::interaction::SelectionHandler without Ogre
 * dependencies: displays attach handlers to @ref PickHandle values so tools
 * can query properties and fire select/deselect hooks.
 *
 * @see PickRegistry
 * @see SelectionManager
 * @see SelectionEntry
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/selection.hpp"

namespace autoviz {
namespace common {

/**
 * @class SelectionHandler
 * @brief Base class for objects that participate in picking / selection.
 *
 * Subclasses override @ref properties() to populate the Selection panel and
 * optionally @ref onSelect() / @ref onDeselect() for visual feedback.
 */
class SelectionHandler : public std::enable_shared_from_this<SelectionHandler> {
 public:
  virtual ~SelectionHandler() = default;

  /**
   * @brief Owning display instance name.
   * @return Display name string.
   */
  const std::string& displayName() const { return display_name_; }

  /**
   * @brief Owning display type id.
   * @return Type string.
   */
  const std::string& displayType() const { return display_type_; }

  /**
   * @brief Pick handle currently associated with this handler.
   * @return Handle, or @ref kInvalidPickHandle if unset.
   */
  PickHandle handle() const { return handle_; }

  /**
   * @brief Sets display identity used when building @ref SelectionEntry.
   *
   * @param name Display instance name.
   * @param type Display type id.
   */
  void setDisplayInfo(std::string name, std::string type);

  /**
   * @brief Associates this handler with a pick handle for the current frame.
   *
   * @param handle Handle from @ref PickRegistry::registerPick.
   */
  void setHandle(PickHandle handle) { handle_ = handle; }

  /**
   * @brief Called when this object becomes selected.
   */
  virtual void onSelect() {}

  /**
   * @brief Called when this object is removed from the selection.
   */
  virtual void onDeselect() {}

  /**
   * @brief Returns key/value pairs for the Selection panel.
   *
   * @return Property list (may be empty).
   */
  virtual std::vector<std::pair<std::string, std::string>> properties() const;

  /**
   * @brief Builds a @ref SelectionEntry from this handler plus a world point.
   *
   * @param position World position of the pick.
   * @param point_index Optional point index (−1 if N/A).
   * @return Entry suitable for @ref SelectionManager::select.
   */
  SelectionEntry toSelectionEntry(const QVector3D& position,
                                  int point_index = -1) const;

 protected:
  /** Display instance name. */
  std::string display_name_;

  /** Display type id. */
  std::string display_type_;

  /** Associated pick handle for this frame. */
  PickHandle handle_ = kInvalidPickHandle;
};

/**
 * @class PointCloudSelectionHandler
 * @brief Selection handler specialized for a single point in a point cloud.
 */
class PointCloudSelectionHandler : public SelectionHandler {
 public:
  /**
   * @brief Sets the cloud point index represented by this handler.
   * @param index Point index (≥ 0), or −1 if unset.
   */
  void setPointIndex(int index) { point_index_ = index; }

  /**
   * @brief Returns the cloud point index.
   * @return Stored index.
   */
  int pointIndex() const { return point_index_; }

  /**
   * @brief Includes the point index in the property list.
   * @return Key/value pairs for the Selection panel.
   */
  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  int point_index_ = -1;
};

/**
 * @class DisplayPointSelectionHandler
 * @brief Generic per-point handler for displays using @c addPoint/@c addPickPoint.
 */
class DisplayPointSelectionHandler : public SelectionHandler {
 public:
  /**
   * @brief Sets the point index within the owning display.
   * @param index Point index (≥ 0), or −1 if unset.
   */
  void setPointIndex(int index) { point_index_ = index; }

  /**
   * @brief Includes the point index in the property list.
   * @return Key/value pairs for the Selection panel.
   */
  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  int point_index_ = -1;
};

/**
 * @class MarkerSelectionHandler
 * @brief Selection handler for a visualization marker (namespace + id).
 */
class MarkerSelectionHandler : public SelectionHandler {
 public:
  /**
   * @brief Sets marker identity for property display.
   *
   * @param ns Marker namespace.
   * @param id Marker id within the namespace.
   */
  void setMarkerInfo(std::string ns, int id) {
    marker_ns_ = std::move(ns);
    marker_id_ = id;
  }

  /**
   * @brief Includes marker namespace and id in the property list.
   * @return Key/value pairs for the Selection panel.
   */
  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  std::string marker_ns_;
  int marker_id_ = -1;
};

/**
 * @class HandlerManager
 * @brief Maps pick handles to @ref SelectionHandler instances for the frame.
 *
 * Stores weak pointers so handlers owned by displays can expire safely.
 */
class HandlerManager {
 public:
  /**
   * @brief Removes all handle→handler associations.
   */
  void clear();

  /**
   * @brief Registers a handler under a pick handle.
   *
   * @param handle Pick handle for this frame.
   * @param handler Shared ownership of the handler (stored as weak_ptr).
   */
  void registerHandler(PickHandle handle,
                       const std::shared_ptr<SelectionHandler>& handler);

  /**
   * @brief Looks up a live handler by handle.
   *
   * @param handle Pick handle.
   * @return Raw pointer if still alive, else @c nullptr.
   */
  SelectionHandler* lookup(PickHandle handle) const;

  /**
   * @brief Invokes @ref SelectionHandler::onSelect for the given handle.
   * @param handle Pick handle that became selected.
   */
  void notifySelected(PickHandle handle);

  /**
   * @brief Invokes @ref SelectionHandler::onDeselect for the given handle.
   * @param handle Pick handle that was deselected.
   */
  void notifyDeselected(PickHandle handle);

 private:
  /** Handle → weak handler map (cleared / rebuilt each frame typically). */
  std::unordered_map<PickHandle, std::weak_ptr<SelectionHandler>> handlers_;
};

/**
 * @brief Factory helper: @c std::make_shared with perfect forwarding.
 *
 * @tparam T Handler type derived from @ref SelectionHandler.
 * @tparam Args Constructor argument types.
 * @param args Forwarded to @c T's constructor.
 * @return Shared pointer to the new handler.
 */
template <typename T, typename... Args>
std::shared_ptr<T> CreateSelectionHandler(Args&&... args) {
  return std::make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace common
}  // namespace autoviz
