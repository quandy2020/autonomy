/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** rviz_common::interaction::SelectionHandler (CPU/GPU pick, no Ogre). */
class SelectionHandler : public std::enable_shared_from_this<SelectionHandler> {
 public:
  virtual ~SelectionHandler() = default;

  const std::string& displayName() const { return display_name_; }
  const std::string& displayType() const { return display_type_; }
  PickHandle handle() const { return handle_; }

  void setDisplayInfo(std::string name, std::string type);
  void setHandle(PickHandle handle) { handle_ = handle; }

  virtual void onSelect() {}
  virtual void onDeselect() {}
  virtual std::vector<std::pair<std::string, std::string>> properties() const;

  SelectionEntry toSelectionEntry(const QVector3D& position,
                                  int point_index = -1) const;

 protected:
  std::string display_name_;
  std::string display_type_;
  PickHandle handle_ = kInvalidPickHandle;
};

class PointCloudSelectionHandler : public SelectionHandler {
 public:
  void setPointIndex(int index) { point_index_ = index; }
  int pointIndex() const { return point_index_; }

  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  int point_index_ = -1;
};

/** Generic per-point handler for displays using addPoint/addPickPoint. */
class DisplayPointSelectionHandler : public SelectionHandler {
 public:
  void setPointIndex(int index) { point_index_ = index; }

  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  int point_index_ = -1;
};

class MarkerSelectionHandler : public SelectionHandler {
 public:
  void setMarkerInfo(std::string ns, int id) {
    marker_ns_ = std::move(ns);
    marker_id_ = id;
  }

  std::vector<std::pair<std::string, std::string>> properties() const override;

 private:
  std::string marker_ns_;
  int marker_id_ = -1;
};

/** Maps pick handles to SelectionHandler instances for the current frame. */
class HandlerManager {
 public:
  void clear();

  void registerHandler(PickHandle handle,
                       const std::shared_ptr<SelectionHandler>& handler);
  SelectionHandler* lookup(PickHandle handle) const;

  void notifySelected(PickHandle handle);
  void notifyDeselected(PickHandle handle);

 private:
  std::unordered_map<PickHandle, std::weak_ptr<SelectionHandler>> handlers_;
};

template <typename T, typename... Args>
std::shared_ptr<T> CreateSelectionHandler(Args&&... args) {
  return std::make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace common
}  // namespace autoviz
