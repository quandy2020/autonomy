/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

namespace autoviz {
namespace teleop {

/** Circular joystick with normalized output in [-1, 1]. */
class TeleopJoystickWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopJoystickWidget(const QString& label, QWidget* parent = nullptr);

  QPointF normalizedValue() const;
  void reset();
  void resetVisual();

 signals:
  void valueChanged(double x, double y);
  void released();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  void updateFromPosition(const QPointF& local_pos);
  QPointF clampToUnitDisk(const QPointF& vector) const;
  QRectF outerRect() const;
  QPointF center() const;
  double radius() const;

  QString label_;
  QPointF knob_{0.0, 0.0};
  bool dragging_ = false;
};

}  // namespace teleop
}  // namespace autoviz
