/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Adapted from rviz_common/splash_screen (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file splash.hpp
 * @brief RViz2-style startup splash screen with AViz branding.
 *
 * Shows status messages during manager / UI construction and enforces a
 * minimum visible time so the splash does not flash past unreadably.
 *
 * @see InstallAppTranslations()
 * @see VisualizationFrame
 */

#pragma once

#include <memory>

#include <QElapsedTimer>
#include <QSplashScreen>

class QPixmap;

namespace autoviz {

/**
 * @class SplashScreen
 * @brief RViz2-style startup splash (AViz branding).
 *
 * ## Timing
 *
 * - @ref showStatusFor() holds long enough to read each message
 * - @ref finish() waits until @p min_total_ms since the first status, then
 *   shows a final ready message for @p ready_hold_ms
 *
 * Processes Qt events while waiting so the splash paints and stays responsive.
 *
 * @note Adapted from @c rviz_common/splash_screen (BSD-3-Clause).
 *
 * @see create()
 */
class SplashScreen : public QSplashScreen {
  Q_OBJECT

 public:
  /**
   * @brief Factory: loads @p image_path and returns a heap splash, or empty on
   *        failure.
   *
   * @param image_path Path to the splash pixmap (resource or filesystem).
   * @return Owned splash, or @c nullptr / empty @c unique_ptr if load failed.
   */
  static std::unique_ptr<SplashScreen> create(const QString& image_path);

  /**
   * @brief Constructs a splash from an already-loaded @p pixmap.
   * @param pixmap Branding image.
   */
  explicit SplashScreen(const QPixmap& pixmap);

  /**
   * @brief Updates the status message drawn on the splash.
   * @param message Localized status text.
   * @see showStatusFor()
   */
  void showStatus(const QString& message);

  /**
   * @brief Show status and hold long enough to read (processes Qt events).
   *
   * @param message Status text.
   * @param hold_ms Minimum time to keep the message visible.
   */
  void showStatusFor(const QString& message, int hold_ms = 550);

  /**
   * @brief Final message + minimum total visible time since first status.
   *
   * @param min_total_ms Minimum wall time the splash should have been visible.
   * @param ready_hold_ms Hold time for the final “ready” message.
   */
  void finish(int min_total_ms = 3200, int ready_hold_ms = 900);

 public slots:
  /**
   * @brief Qt slot alias for @ref showStatus() (signal/slot convenience).
   * @param message Status text.
   */
  void showMessage(const QString& message) { showStatus(message); }

 private:
  /**
   * @brief Starts @c visible_timer_ on the first status update.
   */
  void ensureVisibleTimerStarted();

  /**
   * @brief Blocks for @p milliseconds while processing Qt events.
   * @param milliseconds Wait duration.
   */
  void waitMs(int milliseconds);

  /** Wall clock since first status (for @ref finish()). */
  QElapsedTimer visible_timer_;

  /** Whether @c visible_timer_ has been started. */
  bool visible_timer_started_ = false;
};

}  // namespace autoviz
