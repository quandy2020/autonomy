/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file viewport_hud.hpp
 * @brief Soft glass HUD overlay for 3D viewports (speed / yaw rate / odometer).
 *
 * Subscribes to @c nav_msgs/Odometry (or equivalent) via
 * @ref common::VisualizationManager and paints dials on the top-left of the
 * viewport. Hidden state fully detaches from the scene (no paint, no channel
 * fan-out).
 *
 * @see FrameViewport::installViewportHudOverlay()
 * @see ViewportFloatingToolbar
 * @see AppUiPreferences::viewport_hud_visible
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include <QString>
#include <QWidget>

class QHideEvent;
class QPaintEvent;
class QResizeEvent;
class QShowEvent;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class ViewportHudOverlay
 * @brief Soft glass HUD on the top-left of a 3D viewport.
 *
 * ## Metrics
 *
 * - Linear speed (m/s) with auto-scaling max
 * - Angular rate (rad/s)
 * - Integrated path distance (odometer) from consecutive poses
 *
 * ## Lifecycle
 *
 * @ref setHudVisible(false) stops timers and unsubscribes. Show/hide events
 * call @ref syncActiveState() so embedding in a layout does not leak
 * subscriptions when the parent viewport is hidden.
 *
 * @note Payload parsing runs off the UI thread into @c snapshot_ under
 *       @c mutex_; @ref applyLatestToUi() copies onto the GUI thread.
 *
 * @see glass::OverlayTokens
 * @see FrameViewport
 */
class ViewportHudOverlay : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the HUD and prepares refresh / resolve timers.
   *
   * Does not subscribe until visible and a channel is set (or auto-resolved).
   *
   * @param manager Non-owning visualization manager for channel subscription.
   *        May be @c nullptr (HUD stays idle).
   * @param parent Typically the viewport host widget.
   */
  explicit ViewportHudOverlay(common::VisualizationManager* manager,
                              QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes and stops timers.
   */
  ~ViewportHudOverlay() override;

  /**
   * @brief Sets the odometry channel to subscribe to.
   *
   * Triggers resubscribe when the HUD is active.
   *
   * @param channel Channel name (empty clears / waits for auto-resolve).
   * @see channel()
   * @see resolveChannelIfNeeded()
   */
  void setChannel(const QString& channel);

  /**
   * @brief Returns the configured odometry channel name.
   * @return Channel string (may be empty before resolve).
   */
  QString channel() const;

  /**
   * @brief Shows or hides the HUD and syncs subscription state.
   *
   * @param visible @c true to show and (re)subscribe; @c false to detach.
   * @see isHudVisible()
   * @see syncActiveState()
   */
  void setHudVisible(bool visible);

  /**
   * @brief @c true when the HUD is logically visible (may still be hidden by
   *        parent).
   */
  bool isHudVisible() const { return hud_visible_; }

 protected:
  /**
   * @brief Paints frosted dials from the latest @ref Snapshot.
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Re-activates subscription when the widget becomes visible.
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Detaches subscription when the widget is hidden.
   * @param event Hide event.
   */
  void hideEvent(QHideEvent* event) override;

  /**
   * @brief Relayout hint for dial geometry (invalidates paint).
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

 private:
  /**
   * @struct Snapshot
   * @brief Thread-safe copy of the latest odometry-derived metrics.
   */
  struct Snapshot {
    bool has_data = false;           /**< At least one payload received. */
    double speed_mps = 0.0;          /**< Linear speed (m/s). */
    double angular_rad_s = 0.0;      /**< Yaw rate (rad/s). */
    double distance_m = 0.0;         /**< Integrated path length (m). */
    double max_speed_mps = 5.0;      /**< Dial full-scale speed. */
    double max_angular_rad_s = 1.5;  /**< Dial full-scale yaw rate. */
  };

  /**
   * @brief Picks a default odometry channel when @c channel_ is empty.
   */
  void resolveChannelIfNeeded();

  /**
   * @brief Subscribes to @c channel_ (unsubscribes any previous id first).
   */
  void resubscribe();

  /**
   * @brief Drops the current subscription if any.
   */
  void unsubscribe();

  /**
   * @brief Channel callback: parses payload into @c snapshot_ under lock.
   * @param payload Serialized message bytes.
   */
  void onPayload(const std::string& payload);

  /**
   * @brief Timer slot: copies snapshot and schedules repaint on the GUI thread.
   */
  void applyLatestToUi();

  /**
   * @brief Subscribes when visible + @c hud_visible_; otherwise unsubscribes.
   */
  void syncActiveState();

  /**
   * @brief Locked copy of @c snapshot_.
   * @return Snapshot value for painting.
   */
  Snapshot readSnapshot() const;

  /** Non-owning manager for subscribe / channel discovery. */
  common::VisualizationManager* manager_ = nullptr;

  /** Periodic UI refresh from the latest snapshot. */
  QTimer* refresh_timer_ = nullptr;

  /** Deferred channel auto-resolve while waiting for graph discovery. */
  QTimer* resolve_timer_ = nullptr;

  /** Configured odometry channel name. */
  QString channel_;

  /** Active subscription handle (0 = none). */
  std::uint64_t subscription_id_ = 0;

  /** Logical HUD visibility (independent of QWidget::isVisible). */
  bool hud_visible_ = true;

  /** Guards @c snapshot_ and pose integration fields. */
  mutable std::mutex mutex_;

  /** Latest metrics published by @ref onPayload(). */
  Snapshot snapshot_;

  /** Whether @c last_x_ / @c last_y_ are valid for distance integration. */
  bool has_last_pose_ = false;

  /** Previous pose X for odometer integration. */
  double last_x_ = 0.0;

  /** Previous pose Y for odometer integration. */
  double last_y_ = 0.0;

  /** Accumulated path length (m). */
  double distance_m_ = 0.0;

  /** Running max speed used for dial full-scale. */
  double max_speed_mps_ = 5.0;

  /** Running max angular rate used for dial full-scale. */
  double max_angular_rad_s_ = 1.5;
};

}  // namespace autoviz
