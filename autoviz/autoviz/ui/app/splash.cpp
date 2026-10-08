/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Adapted from rviz_common/splash_screen (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/ui/app/splash.hpp"

#include <QCoreApplication>
#include <QEventLoop>
#include <QFile>
#include <QGuiApplication>
#include <QPainter>
#include <QPixmap>
#include <QScreen>
#include <QSvgRenderer>
#include <QThread>

namespace autoviz {
namespace {

// ~1.2× rviz_common splash (400×260) + status bar.
constexpr int kSplashWidth = 480;
constexpr int kSplashHeight = 312;
constexpr int kBottomBorder = 32;

QPixmap loadSplashSource(const QString& image_path) {
  if (image_path.endsWith(QLatin1String(".svg"), Qt::CaseInsensitive)) {
    if (!QFile::exists(image_path)) {
      return {};
    }
    QFile file(image_path);
    if (!file.open(QIODevice::ReadOnly)) {
      return {};
    }
    const QByteArray svg_bytes = file.readAll();
    QSvgRenderer renderer(svg_bytes);
    if (renderer.isValid()) {
      const QSize default_size = renderer.defaultSize();
      const int w =
          default_size.width() > 0 ? default_size.width() : kSplashWidth;
      const int h =
          default_size.height() > 0 ? default_size.height() : kSplashHeight;
      QPixmap pixmap(w, h);
      pixmap.fill(Qt::transparent);
      QPainter painter(&pixmap);
      renderer.render(&painter, QRectF(0, 0, w, h));
      painter.end();
      if (!pixmap.isNull()) {
        return pixmap;
      }
    }
    // Splash SVGs may embed the original PNG as a data URI (QtSvg may skip
    // <image>). Fall back to decoding the embedded raster.
    const QByteArray marker("data:image/png;base64,");
    const int start = svg_bytes.indexOf(marker);
    if (start >= 0) {
      int end = svg_bytes.indexOf('"', start);
      if (end < 0) {
        end = svg_bytes.indexOf('\'', start);
      }
      if (end > start) {
        const QByteArray b64 =
            svg_bytes.mid(start + marker.size(), end - start - marker.size());
        QPixmap pixmap;
        pixmap.loadFromData(QByteArray::fromBase64(b64), "PNG");
        return pixmap;
      }
    }
    return {};
  }

  // Prefer QFile + loadFromData so :/ Qt resources are reliable on macOS.
  QFile file(image_path);
  if (file.open(QIODevice::ReadOnly)) {
    QPixmap pixmap;
    if (pixmap.loadFromData(file.readAll())) {
      return pixmap;
    }
  }
  return QPixmap(image_path);
}

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
  // QSplashScreen::setPixmap requires a live QScreen; skip when none yet.
  if (QGuiApplication::primaryScreen() == nullptr) {
    return nullptr;
  }
  QPixmap pixmap = loadSplashSource(image_path);
  if (pixmap.isNull()) {
    return nullptr;
  }
  return std::make_unique<SplashScreen>(pixmap);
}

SplashScreen::SplashScreen(const QPixmap& pixmap)
    : QSplashScreen(
          buildSplashPixmap(pixmap),
          // macOS (Ventura+): Qt::SplashScreen is invisible when launched from
          // Terminal until the app activates; Dialog+Frameless works instead.
          // Linux: keep Qt::SplashScreen so the WM does not treat the splash as
          // a second dock / taskbar application next to the main window.
#if defined(Q_OS_MACOS) || defined(Q_OS_MAC)
          Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint
#else
          Qt::SplashScreen | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint
#endif
      ) {
#if defined(Q_OS_LINUX)
  setAttribute(Qt::WA_X11DoNotAcceptFocus);
#endif
  setWindowFlag(Qt::WindowDoesNotAcceptFocus, true);
}

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
