/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QKeyEvent;

namespace autoviz::widget {

class Surface : public QFrame {
  Q_OBJECT

 public:
  enum class Elevation { kFlat, kRaised, kOverlay };

  explicit Surface(QWidget* parent = nullptr);

  Elevation elevation() const noexcept { return elevation_; }
  void SetElevation(Elevation elevation);
  bool is_interactive() const noexcept { return interactive_; }
  void SetInteractive(bool interactive);
  bool is_selected() const noexcept { return selected_; }
  void SetSelected(bool selected);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 signals:
  void Clicked();

 protected:
  void changeEvent(QEvent* event) override;
  void enterEvent(QEnterEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void leaveEvent(QEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  Elevation elevation_ = Elevation::kFlat;
  style::Tone tone_ = style::Tone::Shell;
  bool interactive_ = false;
  bool selected_ = false;
  bool hovered_ = false;
  bool pressed_ = false;
};

}  // namespace autoviz::widget
