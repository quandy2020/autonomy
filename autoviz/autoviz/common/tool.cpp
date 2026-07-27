/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/tool.hpp"

#include <QMouseEvent>
#include <QWheelEvent>

namespace autoviz {
namespace common {

bool Tool::mousePressEvent(QMouseEvent* /*event*/) { return false; }
bool Tool::mouseMoveEvent(QMouseEvent* /*event*/) { return false; }
bool Tool::mouseReleaseEvent(QMouseEvent* /*event*/) { return false; }
bool Tool::wheelEvent(QWheelEvent* /*event*/) { return false; }

void Tool::setProperties(const DisplayPropertyMap& properties) {
  properties_ = properties;
}

std::string Tool::propertyValue(const std::string& key,
                                const std::string& default_value) const {
  const auto it = properties_.find(key);
  if (it == properties_.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}

void Tool::setPropertyValue(const std::string& key, const std::string& value) {
  properties_[key] = value;
}

}  // namespace common
}  // namespace autoviz
