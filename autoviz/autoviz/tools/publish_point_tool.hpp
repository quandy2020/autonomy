/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_point_tool.hpp
 * @brief Publish Point tool — pick a 3D point and publish PointStamped.
 *
 * RViz @c PublishPointTool equivalent. Left-click ray-picks the ground / scene,
 * publishes @c geometry_msgs/PointStamped on the configured topic (default
 * @c /clicked_point), and draws a transient hover/last-click marker.
 *
 * @see PublishMessage
 * @see FillHeader
 * @see common::Tool
 */

#pragma once

#include <optional>

#include <QCursor>
#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class PublishPointTool
 * @brief One-shot tool that publishes a clicked world point on an Autolink
 *        channel.
 *
 * ## Properties
 *
 * | Key     | Label | Default          |
 * |---------|-------|------------------|
 * | topic   | Topic | /clicked_point   |
 *
 * ## Interaction
 *
 * - **Move:** update hover cursor / hover marker when a pick succeeds.
 * - **Press:** publish PointStamped and store @c last_point_.
 *
 * @note Activate swaps to a crosshair-style hit cursor when the pointer is
 *       over a valid pick; deactivate restores the standard cursor.
 */
class PublishPointTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id for the registry / session config.
   * @return @c "PublishPoint".
   */
  std::string id() const override { return "PublishPoint"; }

  /**
   * @brief Human-readable toolbar label.
   * @return Localized @c "Publish Point".
   */
  QString label() const override { return QStringLiteral("Publish Point"); }

  /**
   * @brief Declares the editable Topic property.
   * @return Spec list with default @c /clicked_point.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"topic", "Topic", "/clicked_point"}};
  }

  /**
   * @brief Publishes a PointStamped at the picked world position.
   *
   * @param event Mouse press in viewport coordinates.
   * @return @c true when a point was picked and publish was attempted.
   */
  bool mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates hover pick and status text as the cursor moves.
   *
   * @param event Mouse move in viewport coordinates.
   * @return @c true when the hover pick was handled.
   */
  bool mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Caches cursors and prepares for picking.
   *
   * @param context Active tool context (viewport, node, fixed frame).
   */
  void activate(common::ToolContext* context) override;

  /**
   * @brief Clears hover state and restores the default cursor.
   */
  void deactivate() override;

  /**
   * @brief Draws last-click and hover markers into the overlay.
   *
   * @param scene Active viewport scene overlay.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Status text for the last published / hovered point.
   * @return Coordinate summary, or an empty / instructional string.
   */
  QString statusText() const override;

 private:
  /**
   * @brief Ray-picks a world point at viewport pixel (@p x, @p y).
   *
   * @param x Viewport X in pixels.
   * @param y Viewport Y in pixels.
   * @param[out] hit World hit position on success.
   * @return @c true if a hit was found.
   */
  bool pickPoint(int x, int y, QVector3D* hit) const;

  /**
   * @brief Formats and pushes status text via @c ToolContext::set_status.
   *
   * @param point World position to describe.
   * @param prefix Leading label (e.g. hover vs published).
   */
  void updateStatusFromPoint(const QVector3D& point, const QString& prefix) const;

  /**
   * @brief Resolves the publish channel from the @c topic property.
   * @return Channel name, defaulting to @c /clicked_point.
   */
  std::string publishChannel() const {
    return propertyValue("topic", "/clicked_point");
  }

  /** Last successfully published world point. */
  std::optional<QVector3D> last_point_;

  /** Current hover pick while the tool is active. */
  std::optional<QVector3D> hover_point_;

  /** Cursor shown when a valid pick is under the pointer. */
  QCursor hit_cursor_;

  /** Cursor restored when no valid pick / on deactivate. */
  QCursor std_cursor_;
};

}  // namespace tools
}  // namespace autoviz
