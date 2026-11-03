/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/foundation/widget_theme.hpp"

#include <QList>
#include <QVariant>
#include <QWidget>

namespace autoviz::widget {
namespace {

QColor Blend(const QColor& foreground, const QColor& background, int alpha) {
  alpha = qBound(0, alpha, 255);
  const int inverse = 255 - alpha;
  return QColor(
      (foreground.red() * alpha + background.red() * inverse) / 255,
      (foreground.green() * alpha + background.green() * inverse) / 255,
      (foreground.blue() * alpha + background.blue() * inverse) / 255,
      (foreground.alpha() * alpha + background.alpha() * inverse) / 255);
}

void ApplyPalette(QWidget* widget, const QPalette& palette, style::Tone tone) {
  widget->setPalette(palette);
  widget->setProperty("autovizTone", tone == style::Tone::Frost
                                         ? QStringLiteral("frost")
                                         : QStringLiteral("shell"));
}

}  // namespace

QPalette BuildWidgetPalette(style::Tone tone, const QPalette& base) {
  const ThemeTokens& tokens = GetThemeTokens(tone);
  QPalette palette = base;

  palette.setColor(QPalette::Window, tokens.window);
  palette.setColor(QPalette::WindowText, tokens.text);
  palette.setColor(QPalette::Base, tokens.surface);
  palette.setColor(QPalette::AlternateBase, tokens.surface_raised);
  palette.setColor(QPalette::ToolTipBase, tokens.surface_raised);
  palette.setColor(QPalette::ToolTipText, tokens.text);
  palette.setColor(QPalette::Text, tokens.text);
  palette.setColor(QPalette::Button, tokens.surface_raised);
  palette.setColor(QPalette::ButtonText, tokens.text);
  palette.setColor(QPalette::BrightText, tokens.danger);
  palette.setColor(QPalette::Link, tokens.accent);
  palette.setColor(QPalette::Highlight, tokens.accent);
  palette.setColor(QPalette::HighlightedText, tokens.on_accent);
  palette.setColor(QPalette::PlaceholderText, tokens.text_muted);
  palette.setColor(QPalette::Light, Blend(tokens.surface, tokens.text, 224));
  palette.setColor(QPalette::Midlight, Blend(tokens.surface, tokens.text, 208));
  palette.setColor(QPalette::Mid, tokens.border);
  palette.setColor(QPalette::Dark, Blend(tokens.border, tokens.text, 168));

  const QColor disabled_surface = Blend(tokens.disabled, tokens.surface, 24);
  palette.setColor(QPalette::Disabled, QPalette::WindowText, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::Text, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::PlaceholderText,
                   tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::Base, disabled_surface);
  palette.setColor(QPalette::Disabled, QPalette::Button, disabled_surface);
  return palette;
}

void ApplyWidgetTheme(QWidget* widget, style::Tone tone, bool recursive) {
  if (widget == nullptr) {
    return;
  }

  const QPalette palette = BuildWidgetPalette(tone, widget->palette());
  ApplyPalette(widget, palette, tone);
  if (!recursive) {
    return;
  }

  const QList<QWidget*> descendants = widget->findChildren<QWidget*>();
  for (QWidget* descendant : descendants) {
    ApplyPalette(descendant, palette, tone);
  }
}

}  // namespace autoviz::widget
