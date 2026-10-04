/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file goal_pose_tool.hpp
 * @brief Abstract RViz PoseTool — click-drag yaw on the ground plane.
 *
 * Base class for @ref NavGoalTool and @ref PoseEstimateTool. Interaction:
 * press sets position, drag sets yaw arrow, release commits via
 * @ref onPoseSet(). Subclasses supply id/label/channel/color and publish logic.
 *
 * @see NavGoalTool
 * @see PoseEstimateTool
 * @see common::Tool
 */

#pragma once

#include <optional>

#include <QColor>
#include <QMouseEvent>
#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace rendering {
class SceneOverlay;
}

namespace tools {

/**
 * @class GoalPoseTool
 * @brief Shared click-drag-release pose interaction for 2D goal / estimate tools.
 *
 * ## State machine
 *
 * @code
 *   idle ──press──► kPosition ──drag──► kOrientation ──release──► onPoseSet()
 * @endcode
 *
 * The arrow is drawn into Ogre / SceneOverlay while orientation is being set,
 * and may remain as a committed visual until the next press clears it.
 *
 * @note Subclasses must implement the protected pure virtuals; public mouse /
 *       draw APIs are final interaction plumbing.
 */
class GoalPoseTool : public common::Tool {
 public:
  /**
   * @brief Resets interaction state and attaches @p context.
   * @param context Active viewport / Autolink tool context.
   */
  void activate(common::ToolContext* context) override;

  /**
   * @brief Clears arrow overlay state and detaches context.
   */
  void deactivate() override;

  /**
   * @brief Starts a new pose: ground pick → @c kPosition, shows arrow.
   *
   * @param event Mouse press in viewport coordinates.
   * @return @c true if a ground hit was obtained.
   */
  bool mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates yaw from cursor while in @c kOrientation (or after press drag).
   *
   * @param event Mouse move in viewport coordinates.
   * @return @c true while orientation is being edited.
   */
  bool mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Commits pose: calls @ref onPoseSet() with position + yaw.
   *
   * @param event Mouse release in viewport coordinates.
   * @return @c true if a pose was committed.
   */
  bool mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Draws the live / committed yaw arrow into the overlay.
   * @param scene Active viewport scene overlay.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Status text describing position and yaw while interacting.
   * @return Coordinate / angle summary for the frame status bar.
   */
  QString statusText() const override;

 protected:
  /**
   * @brief Stable subclass tool id (e.g. @c "NavGoal").
   * @return Registry id string.
   */
  virtual std::string toolId() const = 0;

  /**
   * @brief Human-readable subclass label.
   * @return Toolbar label (e.g. @c "2D Goal Pose").
   */
  virtual QString toolLabel() const = 0;

  /**
   * @brief Autolink channel / topic the subclass will publish to.
   * @return Fully-qualified channel name from properties or default.
   */
  virtual std::string publishChannel() const = 0;

  /**
   * @brief Arrow tint for the live pose visual.
   * @return Qt color used by Ogre / overlay drawing.
   */
  virtual QColor arrowColor() const = 0;

  /**
   * @brief Subclass hook invoked when the user finishes click-drag-release.
   *
   * @param position Ground-plane world position (fixed frame).
   * @param yaw Yaw about +Z in radians.
   */
  virtual void onPoseSet(const QVector3D& position, float yaw) = 0;

 private:
  /**
   * @enum State
   * @brief Internal interaction phase for the pose gesture.
   */
  enum class State {
    kPosition,     /**< Waiting for / just placed the anchor position. */
    kOrientation,  /**< Dragging to set yaw relative to the anchor. */
  };

  /**
   * @brief Picks the ground plane at viewport pixel (@p x, @p y).
   *
   * @param x Viewport X in pixels.
   * @param y Viewport Y in pixels.
   * @param[out] hit World hit on the ground plane.
   * @return @c true on a successful pick.
   */
  bool pickGround(int x, int y, QVector3D* hit) const;

  /**
   * @brief Computes yaw from @p anchor toward @p cursor in the XY plane.
   *
   * @param cursor Current ground pick under the mouse.
   * @param anchor Pose anchor from the press event.
   * @return Yaw in radians.
   */
  static float calculateAngle(const QVector3D& cursor, const QVector3D& anchor);

  /** Removes any Ogre-hosted arrow entity for this tool. */
  void clearOgreOverlay() const;

  /** Rebuilds the Ogre arrow from @c arrow_position_ / @c angle_. */
  void refreshArrowVisual() const;

  /**
   * @brief Draws the arrow into @p scene (Qt overlay path).
   * @param scene Overlay to draw into; may be @c nullptr for Ogre-only refresh.
   */
  void drawArrowVisual(rendering::SceneOverlay* scene) const;

  /** Hides the live arrow without clearing committed pose state. */
  void hideArrowVisual() const;

  /** Current gesture phase. */
  State state_ = State::kPosition;

  /** Live yaw while dragging (radians). */
  float angle_ = 0.f;

  /** Whether the live arrow should be drawn. */
  bool arrow_visible_ = false;

  /** Anchor / tip position for the live arrow. */
  std::optional<QVector3D> arrow_position_;

  /** Last committed position (shown until next press). */
  std::optional<QVector3D> committed_position_;

  /** Last committed yaw (radians). */
  float committed_angle_ = 0.f;
};

}  // namespace tools
}  // namespace autoviz
