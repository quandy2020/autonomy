/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QBasicTimer>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

// Lightweight loading placeholder. It owns one basic timer and stops it while
// hidden or disabled so off-screen views have no animation cost.
class Skeleton final : public QWidget {
 public:
  explicit Skeleton(QWidget* parent = nullptr);

  bool is_active() const noexcept { return active_; }
  void SetActive(bool active);
  int corner_radius() const noexcept { return corner_radius_; }
  void SetCornerRadius(int radius);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;

 protected:
  void changeEvent(QEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void timerEvent(QTimerEvent* event) override;

 private:
  void SyncAnimation();

  QBasicTimer timer_;
  style::Tone tone_ = style::Tone::Shell;
  int phase_ = 0;
  int corner_radius_ = 6;
  bool active_ = true;
};

}  // namespace autoviz::widget
