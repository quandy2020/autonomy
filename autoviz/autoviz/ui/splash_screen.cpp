/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Adapted from rviz_common/splash_screen (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/ui/splash_screen.hpp"

#include <QCoreApplication>
#include <QEventLoop>
#include <QPainter>
#include <QPixmap>
#include <QThread>

namespace autoviz {
namespace {

// Match rviz_common splash.png (400x260) + bottom status bar.
constexpr int kSplashWidth = 400;
constexpr int kSplashHeight = 260;
constexpr int kBottomBorder = 27;

QPixmap buildSplashPixmap(const QPixmap& pixmap) {
  QPixmap scaled = pixmap.scaled(
      kSplashWidth, kSplashHeight, Qt::IgnoreAspectRatio,
      Qt::SmoothTransformation);

  QPixmap splash(kSplashWidth, kSplashHeight + kBottomBorder);
  splash.fill(QColor(0, 0, 0));

  QPainter painter(&splash);
  painter.drawPixmap(QPoint(0, 0), scaled);
  return splash;
}

}  // namespace

std::unique_ptr<SplashScreen> SplashScreen::create(const QString& image_path) {
  QPixmap pixmap(image_path);
  if (pixmap.isNull()) {
    return nullptr;
  }
  return std::make_unique<SplashScreen>(pixmap);
}

SplashScreen::SplashScreen(const QPixmap& pixmap) : QSplashScreen(buildSplashPixmap(pixmap)) {}

void SplashScreen::ensureVisibleTimerStarted() {
  if (!visible_timer_started_) {
    visible_timer_.start();
    visible_timer_started_ = true;
  }
}

void SplashScreen::waitMs(int milliseconds) {
  if (milliseconds <= 0) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    return;
  }
  QElapsedTimer wait;
  wait.start();
  while (wait.elapsed() < milliseconds) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    QThread::msleep(16);
  }
}

void SplashScreen::showStatus(const QString& message) {
  ensureVisibleTimerStarted();
  QSplashScreen::showMessage(message, Qt::AlignLeft | Qt::AlignBottom, Qt::white);
  QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
}

void SplashScreen::showStatusFor(const QString& message, int hold_ms) {
  showStatus(message);
  waitMs(hold_ms);
}

void SplashScreen::finish(int min_total_ms, int ready_hold_ms) {
  showStatus(QStringLiteral("AViz is ready."));
  waitMs(ready_hold_ms);
  const int remaining =
      min_total_ms - static_cast<int>(visible_timer_.elapsed());
  waitMs(remaining);
}

}  // namespace autoviz
