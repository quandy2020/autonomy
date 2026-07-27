/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Adapted from rviz_common/splash_screen (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#include <memory>

#include <QElapsedTimer>
#include <QSplashScreen>

class QPixmap;

namespace autoviz {

/** RViz2-style startup splash (AViz branding). */
class SplashScreen : public QSplashScreen {
  Q_OBJECT

 public:
  static std::unique_ptr<SplashScreen> create(const QString& image_path);

  explicit SplashScreen(const QPixmap& pixmap);

  void showStatus(const QString& message);
  /** Show status and hold long enough to read (processes Qt events). */
  void showStatusFor(const QString& message, int hold_ms = 550);
  /** Final message + minimum total visible time since first status. */
  void finish(int min_total_ms = 3200, int ready_hold_ms = 900);

 public slots:
  void showMessage(const QString& message) { showStatus(message); }

 private:
  void ensureVisibleTimerStarted();
  void waitMs(int milliseconds);

  QElapsedTimer visible_timer_;
  bool visible_timer_started_ = false;
};

}  // namespace autoviz
