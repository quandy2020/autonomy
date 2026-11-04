/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QPaintEvent;

namespace autoviz::widget {

// Circular identity image with deterministic initials fallback.
class Avatar final : public QWidget {
 public:
  explicit Avatar(QWidget* parent = nullptr);

  QString name() const { return name_; }
  void SetName(const QString& name);
  QImage image() const { return image_; }
  void SetImage(const QImage& image);
  int diameter() const noexcept { return diameter_; }
  void SetDiameter(int diameter);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QString Initials() const;

  QString name_;
  QImage image_;
  style::Tone tone_ = style::Tone::Shell;
  int diameter_ = 36;
};

}  // namespace autoviz::widget
