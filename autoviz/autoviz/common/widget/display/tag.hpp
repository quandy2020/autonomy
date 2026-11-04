/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;

namespace autoviz::widget {

// Compact label/filter chip. Checkability uses QAbstractButton semantics;
// removal is an explicit request so the owner controls object lifetime.
class Tag final : public QAbstractButton {
  Q_OBJECT

 public:
  enum class Kind { kNeutral, kAccent, kSuccess, kWarning, kDanger };
  Q_ENUM(Kind)

  explicit Tag(QWidget* parent = nullptr);
  explicit Tag(const QString& text, QWidget* parent = nullptr);

  Kind kind() const noexcept { return kind_; }
  void SetKind(Kind kind);
  bool removable() const noexcept { return removable_; }
  void SetRemovable(bool removable);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void RemoveRequested();

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  QRect RemoveRect() const;
  void UpdateAccessibility();

  Kind kind_ = Kind::kNeutral;
  style::Tone tone_ = style::Tone::Shell;
  bool removable_ = false;
};

}  // namespace autoviz::widget
