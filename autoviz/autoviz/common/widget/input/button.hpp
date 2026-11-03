/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

namespace autoviz::widget {

class Button : public QAbstractButton {
 public:
  enum class Variant { kPrimary, kSecondary, kSubtle, kDanger };
  enum class Size { kSmall, kMedium, kLarge };

  explicit Button(QWidget* parent = nullptr);
  explicit Button(const QString& text, QWidget* parent = nullptr);

  Variant variant() const noexcept { return variant_; }
  void SetVariant(Variant variant);
  Size button_size() const noexcept { return size_; }
  void SetButtonSize(Size size);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void enterEvent(QEnterEvent* event) override;
  void leaveEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  int ControlHeight(const ThemeTokens& tokens) const noexcept;

  Variant variant_ = Variant::kSecondary;
  Size size_ = Size::kMedium;
  style::Tone tone_ = style::Tone::Shell;
};

class IconButton final : public Button {
 public:
  explicit IconButton(QWidget* parent = nullptr);
  explicit IconButton(const QIcon& icon, const QString& accessible_name,
                      QWidget* parent = nullptr);

  QSize sizeHint() const override;
};

}  // namespace autoviz::widget
