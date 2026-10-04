/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file screen_projection.hpp
 * @brief World ↔ screen projections using DisplayContext view matrices.
 *
 * Requires @c DisplayContext::has_view_matrices and a positive viewport size.
 * Used by label offsets, screen-space annotations, and tools that place
 * geometry relative to projected anchors.
 *
 * @see projectWorldToScreen()
 * @see unprojectScreenToWorld()
 * @see applyScreenOffset()
 * @see common::DisplayContext
 */

#pragma once

#include <QVector3D>

namespace autoviz {
namespace common {
class DisplayContext;
}

namespace display {

/**
 * @brief Projects a world point to pixel coordinates (origin top-left).
 *
 * @param context Display context with view/projection matrices.
 * @param world World / fixed-frame point.
 * @param out_x Output pixel X; must be non-null.
 * @param out_y Output pixel Y; must be non-null.
 * @return @c true when the point is in front of the camera (@c w > ε).
 */
bool projectWorldToScreen(const common::DisplayContext* context, const QVector3D& world,
                          float* out_x, float* out_y);

/**
 * @brief Unprojects a screen pixel at a given eye-space depth to world.
 *
 * @param context Display context with view/projection matrices.
 * @param screen_x Pixel X (top-left origin).
 * @param screen_y Pixel Y (top-left origin).
 * @param eye_depth Positive depth in eye space (−Z of the view transform).
 * @param out_world Output world point; must be non-null.
 * @return @c true on success.
 */
bool unprojectScreenToWorld(const common::DisplayContext* context, float screen_x,
                              float screen_y, float eye_depth, QVector3D* out_world);

/**
 * @brief Projects @p anchor, applies a pixel offset, then unprojects to world.
 *
 * Keeps roughly constant screen-space offset for labels / HUD attachments
 * that must track a 3D anchor.
 *
 * @param context Display context with view/projection matrices.
 * @param anchor World-space anchor point.
 * @param offset_x Pixel offset in X (right positive).
 * @param offset_y Pixel offset in Y (down positive, matching screen Y).
 * @return Adjusted world position; returns @p anchor if projection fails.
 */
QVector3D applyScreenOffset(const common::DisplayContext* context, const QVector3D& anchor,
                            float offset_x, float offset_y);

}  // namespace display
}  // namespace autoviz
