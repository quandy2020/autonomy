/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/view_picker.hpp"

#include "autoviz/common/tool.hpp"
#include "autoviz/rendering/pick_utils.hpp"

namespace autoviz {
namespace common {

bool ViewPicker::get3DPoint(int pixel_x, int pixel_y, QVector3D* point) const {
  if (context_ == nullptr || point == nullptr) {
    return false;
  }
  const rendering::PickResult pick =
      rendering::pickAtToolContext(*context_, pixel_x, pixel_y);
  if (!pick.hit) {
    return false;
  }
  *point = pick.position();
  return true;
}

}  // namespace common
}  // namespace autoviz
