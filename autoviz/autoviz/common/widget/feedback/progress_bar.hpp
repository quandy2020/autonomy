/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QProgressBar>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QEvent;
class QPaintEvent;

namespace autoviz::widget {

// Linear progress indicator that keeps native QProgressBar accessibility.
class ProgressBar final : public QProgressBar {
 public:
  enum class State { kNormal, kSuccess, kWarning, kError };

  explicit ProgressBar(QWidget* parent = nullptr);

  State state() const noexcept { return state_; }
  void SetState(State state);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void changeEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  void ApplyPalette();

  State state_ = State::kNormal;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace autoviz::widget
