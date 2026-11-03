/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QHideEvent;
class QPaintEvent;
class QPropertyAnimation;
class QShowEvent;

namespace autoviz::widget {

// Keyboard-accessible binary switch with a lightweight thumb animation.
class ToggleSwitch final : public QAbstractButton {
  Q_OBJECT
  Q_PROPERTY(qreal thumbPosition READ thumbPosition WRITE setThumbPosition)

 public:
  explicit ToggleSwitch(QWidget* parent = nullptr);

  qreal thumbPosition() const noexcept { return thumb_position_; }
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 public slots:
  void setThumbPosition(qreal position);

 protected:
  void changeEvent(QEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  bool hitButton(const QPoint& position) const override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;

 private:
  void AnimateToCheckedState(bool checked);
  void StopAnimationAtCurrentState();

  QPropertyAnimation* animation_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  qreal thumb_position_ = 0.0;
};

}  // namespace autoviz::widget
