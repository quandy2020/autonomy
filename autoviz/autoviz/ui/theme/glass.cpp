/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/glass.hpp"

#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

namespace autoviz {
namespace glass {

OverlayTokens Overlay() { return OverlayTokens{}; }

ShellTokens Shell() { return ShellTokens{}; }

QString Hex(const QColor& c) { return c.name(QColor::HexRgb); }

QString Rgba(const QColor& c) {
  return QStringLiteral("rgba(%1,%2,%3,%4)")
      .arg(c.red())
      .arg(c.green())
      .arg(c.blue())
      .arg(c.alpha());
}

QString ShellGlassBarGradientCss() {
  const ShellTokens t = Shell();
  return QStringLiteral(
             "qlineargradient(x1:0,y1:0,x2:0,y2:1,"
             "stop:0 %1, stop:0.35 %2, stop:1 %3)")
      .arg(Rgba(QColor(255, 255, 255, 245)), Rgba(t.glass_fill_deep),
           Rgba(t.glass_fill));
}

QString ShellWindowWashCss() {
  // Milky wash (乳白) with a whisper of teal — avoid sky-blue #CDEBFA.
  return QStringLiteral(
      "qlineargradient(x1:0,y1:0,x2:1,y2:1,"
      "stop:0 #EEF6F4, stop:0.45 #F5FAF8, stop:1 #FAFCFB)");
}

QString ShellCssBg() { return Rgba(Shell().glass_fill); }

QString ShellCssSurface() { return Rgba(Shell().glass_fill_deep); }

QString ShellCssBorder() { return Rgba(Shell().glass_rim); }

QString ShellCssText() { return Hex(Shell().text); }

QString ShellCssMuted() { return Hex(Shell().secondary_text); }

QString ShellCssAccent() { return Hex(Shell().accent); }

void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius) {
  const ShellTokens t = Shell();
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF card = rect.adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, radius, radius);

  // Base: dense milky white.
  painter.fillPath(path, t.glass_fill_deep);

  // Soft white body wash (reads as frosted milk, not tinted glass).
  QLinearGradient body(card.topLeft(), card.bottomLeft());
  body.setColorAt(0.0, t.glass_fill);
  body.setColorAt(1.0, QColor(255, 255, 255, 120));
  painter.fillPath(path, body);

  // Barely-there cool whisper, then strong top sheen.
  painter.fillPath(path, t.glass_cool);

  QColor sheen_end = t.glass_sheen;
  sheen_end.setAlpha(0);
  QLinearGradient sheen(card.topLeft(),
                        QPointF(card.left(), card.top() + card.height() * 0.45));
  sheen.setColorAt(0.0, t.glass_sheen);
  sheen.setColorAt(1.0, sheen_end);
  painter.fillPath(path, sheen);

  painter.setPen(QPen(t.glass_rim, 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
}

}  // namespace glass
}  // namespace autoviz
