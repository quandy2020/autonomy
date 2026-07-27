/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {

struct SelectionEntry {
  QVector3D position;
  std::string display_name;
  std::string display_type;
  PickHandle pick_handle = kInvalidPickHandle;
  int point_index = -1;
  std::vector<std::pair<std::string, std::string>> properties;
};

}  // namespace common
}  // namespace autoviz
