/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPointF>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QResizeEvent;

namespace autoviz::widget {

struct StepItem {
  QString title;
  bool enabled = true;
};

// Horizontal progress/navigation indicator for multi-stage workflows.
class StepIndicator final : public QWidget {
  Q_OBJECT

 public:
  explicit StepIndicator(QWidget* parent = nullptr);

  const QVector<StepItem>& steps() const noexcept { return steps_; }
  void SetSteps(QVector<StepItem> steps);
  int current_index() const noexcept { return current_index_; }
  void SetCurrentIndex(int index);
  bool is_interactive() const noexcept { return interactive_; }
  void SetInteractive(bool interactive);
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void CurrentIndexChanged(int index);
  void StepActivated(int index);

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  void Activate(int index);
  void MoveCurrent(int direction);
  void RebuildGeometry();
  void UpdateAccessibility();
  int HitTest(const QPointF& position) const;
  bool IsSelectable(int index) const;

  QVector<StepItem> steps_;
  QVector<QPointF> centers_;
  style::Tone tone_ = style::Tone::Shell;
  int current_index_ = -1;
  bool interactive_ = false;
};

}  // namespace autoviz::widget
