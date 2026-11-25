/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_hud_overlay.hpp
 * @brief Circular always-on-top Teleop stick. Drag anywhere except the knob.
 *
 * Independent Tool window (no mosaic snap, no parent clip). Floats above
 * Autoviz panels and follows the main window into the background. Esc or the
 * panel HUD button docks it back.
 *
 * @see TeleopPanel
 * @see TeleopControlWidget
 */

#pragma once

#include <QPoint>
#include <QRect>
#include <QWidget>

class QCloseEvent;
class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QShowEvent;
class QVBoxLayout;
class QWheelEvent;

namespace autoviz {
namespace teleop {

/**
 * @class TeleopHudOverlay
 * @brief Frameless circular HUD that hosts the teleop stick.
 *
 * Left-drag the knob to drive. Left-drag anywhere else (or right-drag) to
 * move the overlay. Mouse wheel changes opacity.
 */
class TeleopHudOverlay : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopHudOverlay(QWidget* parent = nullptr);

  void setControl(QWidget* control);
  QWidget* takeControl();
  QWidget* control() const { return control_; }

  void setOpacity(double opacity);
  double opacity() const { return opacity_; }

  void placeDefault(QWidget* relative);
  bool applyGeometryIfOnScreen(const QRect& geometry);
  static bool geometryOnScreen(const QRect& geometry);

 signals:
  void closeRequested();
  void opacityChanged(double opacity);
  void geometryChanged();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void closeEvent(QCloseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void showEvent(QShowEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void beginDrag(const QPoint& global_pos);
  void endDrag();

  QWidget* host_ = nullptr;
  QVBoxLayout* host_layout_ = nullptr;
  QWidget* control_ = nullptr;

  double opacity_ = 0.92;
  bool dragging_ = false;
  QPoint drag_offset_;
};

}  // namespace teleop
}  // namespace autoviz
