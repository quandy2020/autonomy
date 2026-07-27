/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

namespace autoviz {
namespace common {
class DisplayContext;
}

namespace display {

bool projectWorldToScreen(const common::DisplayContext* context, const QVector3D& world,
                          float* out_x, float* out_y);

bool unprojectScreenToWorld(const common::DisplayContext* context, float screen_x,
                              float screen_y, float eye_depth, QVector3D* out_world);

/** BICMap LabelBubble：锚点投影到屏幕后叠加像素偏移，再反投影回世界坐标。 */
QVector3D applyScreenOffset(const common::DisplayContext* context, const QVector3D& anchor,
                            float offset_x, float offset_y);

}  // namespace display
}  // namespace autoviz
