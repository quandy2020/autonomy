/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QIcon>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPixmap>
#include <QPolygonF>

namespace autoviz::widget::internal {
enum class SemanticStatusIcon { kInformation, kSuccess, kWarning, kError };

namespace semantic_icon_internal {

enum class Glyph {
  kClose,
  kVisibility,
  kVisibilityOff,
  kChevronDown,
  kChevronRight,
  kInformation,
  kSuccess,
  kWarning,
  kError,
};

inline QPixmap DrawGlyph(Glyph glyph, const QColor& color, int scale) {
  constexpr int kExtent = 18;
  QPixmap pixmap(kExtent * scale, kExtent * scale);
  pixmap.setDevicePixelRatio(scale);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(color, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.setBrush(Qt::NoBrush);

  if (glyph == Glyph::kClose) {
    painter.drawLine(QPointF(5.0, 5.0), QPointF(13.0, 13.0));
    painter.drawLine(QPointF(13.0, 5.0), QPointF(5.0, 13.0));
    return pixmap;
  }

  if (glyph == Glyph::kChevronDown) {
    painter.drawPolyline(QPolygonF{QPointF(5.5, 6.75), QPointF(9.0, 10.25),
                                   QPointF(12.5, 6.75)});
    return pixmap;
  }

  if (glyph == Glyph::kChevronRight) {
    painter.drawPolyline(QPolygonF{QPointF(6.75, 5.5), QPointF(10.25, 9.0),
                                   QPointF(6.75, 12.5)});
    return pixmap;
  }

  if (glyph == Glyph::kWarning) {
    QPainterPath triangle;
    triangle.moveTo(9.0, 2.3);
    triangle.lineTo(16.1, 14.8);
    triangle.quadTo(16.5, 15.6, 15.5, 15.6);
    triangle.lineTo(2.5, 15.6);
    triangle.quadTo(1.5, 15.6, 1.9, 14.8);
    triangle.closeSubpath();
    painter.drawPath(triangle);
    painter.drawLine(QPointF(9.0, 6.1), QPointF(9.0, 10.7));
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    painter.drawEllipse(QPointF(9.0, 13.2), 1.0, 1.0);
    return pixmap;
  }

  if (glyph == Glyph::kInformation || glyph == Glyph::kSuccess ||
      glyph == Glyph::kError) {
    painter.drawEllipse(QRectF(2.3, 2.3, 13.4, 13.4));
    if (glyph == Glyph::kInformation) {
      painter.drawLine(QPointF(9.0, 7.7), QPointF(9.0, 12.7));
      painter.setPen(Qt::NoPen);
      painter.setBrush(color);
      painter.drawEllipse(QPointF(9.0, 5.3), 1.0, 1.0);
    } else if (glyph == Glyph::kSuccess) {
      QPainterPath check;
      check.moveTo(5.2, 9.1);
      check.lineTo(7.8, 11.6);
      check.lineTo(12.9, 6.4);
      painter.drawPath(check);
    } else {
      painter.drawLine(QPointF(6.2, 6.2), QPointF(11.8, 11.8));
      painter.drawLine(QPointF(11.8, 6.2), QPointF(6.2, 11.8));
    }
    return pixmap;
  }

  QPainterPath eye;
  eye.moveTo(2.0, 9.0);
  eye.cubicTo(4.7, 4.8, 13.3, 4.8, 16.0, 9.0);
  eye.cubicTo(13.3, 13.2, 4.7, 13.2, 2.0, 9.0);
  painter.drawPath(eye);
  painter.setBrush(color);
  painter.drawEllipse(QPointF(9.0, 9.0), 2.1, 2.1);
  if (glyph == Glyph::kVisibilityOff) {
    painter.setBrush(Qt::NoBrush);
    painter.drawLine(QPointF(3.0, 3.0), QPointF(15.0, 15.0));
  }
  return pixmap;
}

inline void AddModePixmaps(QIcon* icon, Glyph glyph, const QColor& color,
                           QIcon::Mode mode) {
  // Store native 1x/2x/3x rasters so Qt never has to upscale a small glyph on
  // high-DPI displays. Painting still uses logical coordinates.
  for (int scale = 1; scale <= 3; ++scale) {
    icon->addPixmap(DrawGlyph(glyph, color, scale), mode);
  }
}

inline QIcon BuildIcon(const QPalette& palette, QPalette::ColorRole role,
                       Glyph glyph) {
  QIcon icon;
  const QColor active = palette.color(QPalette::Active, role);
  AddModePixmaps(&icon, glyph, active, QIcon::Normal);
  AddModePixmaps(&icon, glyph, active, QIcon::Active);
  AddModePixmaps(&icon, glyph, palette.color(QPalette::Disabled, role),
                 QIcon::Disabled);
  AddModePixmaps(&icon, glyph, active, QIcon::Selected);
  return icon;
}

inline QIcon BuildTintedIcon(const QPalette& palette, const QColor& color,
                             Glyph glyph) {
  QIcon icon;
  AddModePixmaps(&icon, glyph, color, QIcon::Normal);
  AddModePixmaps(&icon, glyph, color, QIcon::Active);
  AddModePixmaps(&icon, glyph,
                 palette.color(QPalette::Disabled, QPalette::ButtonText),
                 QIcon::Disabled);
  AddModePixmaps(&icon, glyph, color, QIcon::Selected);
  return icon;
}

}  // namespace semantic_icon_internal

inline QIcon MakePaletteCloseIcon(const QPalette& palette) {
  return semantic_icon_internal::BuildIcon(
      palette, QPalette::ButtonText, semantic_icon_internal::Glyph::kClose);
}

inline QIcon MakePaletteVisibilityIcon(const QPalette& palette,
                                       bool password_visible) {
  return semantic_icon_internal::BuildIcon(
      palette, QPalette::Text,
      password_visible ? semantic_icon_internal::Glyph::kVisibilityOff
                       : semantic_icon_internal::Glyph::kVisibility);
}

inline QIcon MakePaletteChevronIcon(const QPalette& palette, bool expanded) {
  return semantic_icon_internal::BuildIcon(
      palette, QPalette::ButtonText,
      expanded ? semantic_icon_internal::Glyph::kChevronDown
               : semantic_icon_internal::Glyph::kChevronRight);
}

inline QIcon MakePaletteStatusIcon(const QPalette& palette, const QColor& color,
                                   SemanticStatusIcon status) {
  using semantic_icon_internal::Glyph;
  Glyph glyph = Glyph::kInformation;
  switch (status) {
    case SemanticStatusIcon::kInformation:
      glyph = Glyph::kInformation;
      break;
    case SemanticStatusIcon::kSuccess:
      glyph = Glyph::kSuccess;
      break;
    case SemanticStatusIcon::kWarning:
      glyph = Glyph::kWarning;
      break;
    case SemanticStatusIcon::kError:
      glyph = Glyph::kError;
      break;
  }
  return semantic_icon_internal::BuildTintedIcon(palette, color, glyph);
}

}  // namespace autoviz::widget::internal
