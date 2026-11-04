/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/display/avatar.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QStringList>
#include <QVariant>

namespace autoviz::widget {

Avatar::Avatar(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizAvatar"));
  setProperty("autovizRole", QStringLiteral("avatar"));
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAccessibleName(tr("Avatar"));
}

void Avatar::SetName(const QString& name) {
  if (name_ == name) {
    return;
  }
  name_ = name;
  setAccessibleName(name_.isEmpty() ? tr("Avatar") : name_);
  setToolTip(name_);
  update();
}

void Avatar::SetImage(const QImage& image) {
  if (image_.cacheKey() == image.cacheKey()) {
    return;
  }
  image_ = image;
  update();
}

void Avatar::SetDiameter(int diameter) {
  const int bounded = qBound(16, diameter, 256);
  if (diameter_ == bounded) {
    return;
  }
  diameter_ = bounded;
  updateGeometry();
  update();
}

void Avatar::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Avatar::sizeHint() const { return QSize(diameter_, diameter_); }

QSize Avatar::minimumSizeHint() const { return QSize(16, 16); }

void Avatar::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QRectF bounds = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  QPainterPath clip;
  clip.addEllipse(bounds);

  if (!image_.isNull()) {
    painter.save();
    painter.setClipPath(clip);
    painter.drawImage(bounds, image_, QRectF(image_.rect()));
    painter.restore();
  } else {
    painter.fillPath(clip, isEnabled() ? tokens.accent : tokens.disabled);
    QFont initials_font = font();
    initials_font.setBold(true);
    initials_font.setPixelSize(qMax(9, qRound(bounds.height() * 0.38)));
    painter.setFont(initials_font);
    painter.setPen(tokens.on_accent);
    painter.drawText(bounds, Qt::AlignCenter, Initials());
  }

  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(tokens.border, 1));
  painter.drawEllipse(bounds);
}

QString Avatar::Initials() const {
  const QStringList words =
      name_.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
  if (words.isEmpty()) {
    return QStringLiteral("?");
  }
  QString initials = words.front().left(1);
  if (words.size() > 1) {
    initials += words.back().left(1);
  }
  return initials.toUpper();
}

}  // namespace autoviz::widget
