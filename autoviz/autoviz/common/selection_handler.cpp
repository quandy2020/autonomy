/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/selection_handler.hpp"

namespace autoviz {
namespace common {

void SelectionHandler::setDisplayInfo(std::string name, std::string type) {
  display_name_ = std::move(name);
  display_type_ = std::move(type);
}

std::vector<std::pair<std::string, std::string>>
SelectionHandler::properties() const {
  return {};
}

SelectionEntry SelectionHandler::toSelectionEntry(
    const QVector3D& position, int point_index) const {
  SelectionEntry entry;
  entry.position = position;
  entry.display_name = display_name_;
  entry.display_type = display_type_;
  entry.pick_handle = handle_;
  entry.point_index = point_index;
  entry.properties = properties();
  return entry;
}

std::vector<std::pair<std::string, std::string>>
PointCloudSelectionHandler::properties() const {
  std::vector<std::pair<std::string, std::string>> props;
  if (point_index_ >= 0) {
    props.emplace_back("Point Index", std::to_string(point_index_));
  }
  return props;
}

std::vector<std::pair<std::string, std::string>>
DisplayPointSelectionHandler::properties() const {
  std::vector<std::pair<std::string, std::string>> props;
  if (point_index_ >= 0) {
    props.emplace_back("Point Index", std::to_string(point_index_));
  }
  if (!display_name_.empty()) {
    props.emplace_back("Display", display_name_);
  }
  return props;
}

std::vector<std::pair<std::string, std::string>>
MarkerSelectionHandler::properties() const {
  std::vector<std::pair<std::string, std::string>> props;
  if (!marker_ns_.empty()) {
    props.emplace_back("Marker NS", marker_ns_);
  }
  if (marker_id_ >= 0) {
    props.emplace_back("Marker ID", std::to_string(marker_id_));
  }
  return props;
}

void HandlerManager::clear() { handlers_.clear(); }

void HandlerManager::registerHandler(
    PickHandle handle, const std::shared_ptr<SelectionHandler>& handler) {
  if (handle == kInvalidPickHandle || handler == nullptr) {
    return;
  }
  handler->setHandle(handle);
  handlers_[handle] = handler;
}

SelectionHandler* HandlerManager::lookup(PickHandle handle) const {
  const auto it = handlers_.find(handle);
  if (it == handlers_.end()) {
    return nullptr;
  }
  return it->second.lock().get();
}

void HandlerManager::notifySelected(PickHandle handle) {
  if (SelectionHandler* handler = lookup(handle)) {
    handler->onSelect();
  }
}

void HandlerManager::notifyDeselected(PickHandle handle) {
  if (SelectionHandler* handler = lookup(handle)) {
    handler->onDeselect();
  }
}

}  // namespace common
}  // namespace autoviz
