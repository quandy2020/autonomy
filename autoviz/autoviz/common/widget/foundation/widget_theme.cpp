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

ThemeAppearance ResolveAppearance(ThemeAppearance appearance,
                                  const QPalette& palette) {
  if (appearance != ThemeAppearance::kSystem) {
    return appearance;
  }
  return qGray(palette.color(QPalette::Window).rgb()) < 128
             ? ThemeAppearance::kDark
             : ThemeAppearance::kLight;
}

void ApplyPalette(QWidget* widget, const QPalette& palette, style::Tone tone,
                  const ThemeTokens& tokens) {
  widget->setPalette(palette);
  widget->setProperty("autovizTone", tone == style::Tone::Frost
                                         ? QStringLiteral("frost")
                                         : QStringLiteral("shell"));
  widget->setProperty("autovizAppearance", tokens.is_dark
                                               ? QStringLiteral("dark")
                                               : QStringLiteral("light"));
}

}  // namespace

QPalette BuildWidgetPalette(style::Tone tone, const QPalette& base,
                            ThemeAppearance appearance) {
  const ThemeAppearance resolved = ResolveAppearance(appearance, base);
  const ThemeTokens& tokens = GetThemeTokens(tone, resolved);
  QPalette palette = base;

  palette.setColor(QPalette::Window, tokens.window);
  palette.setColor(QPalette::WindowText, tokens.text);
  palette.setColor(QPalette::Base, tokens.surface);
  palette.setColor(QPalette::AlternateBase, tokens.surface_sunken);
  palette.setColor(QPalette::ToolTipBase, tokens.surface_material);
  palette.setColor(QPalette::ToolTipText, tokens.text);
  palette.setColor(QPalette::Text, tokens.text);
  palette.setColor(QPalette::Button, tokens.surface_material);
  palette.setColor(QPalette::ButtonText, tokens.text);
  palette.setColor(QPalette::BrightText, tokens.danger);
  palette.setColor(QPalette::Link, tokens.accent_text);
  palette.setColor(QPalette::LinkVisited, tokens.accent_text);
  palette.setColor(QPalette::Highlight, tokens.accent);
  palette.setColor(QPalette::HighlightedText, tokens.on_accent);
  palette.setColor(QPalette::PlaceholderText, tokens.placeholder);
  palette.setColor(QPalette::Light, Blend(tokens.surface, tokens.text, 224));
  palette.setColor(QPalette::Midlight, Blend(tokens.surface, tokens.text, 208));
  palette.setColor(QPalette::Mid, tokens.separator);
  palette.setColor(QPalette::Dark, tokens.border_strong);
  palette.setColor(QPalette::Shadow, tokens.shadow_ambient);

  palette.setColor(QPalette::Inactive, QPalette::WindowText,
                   tokens.text_secondary);
  palette.setColor(QPalette::Inactive, QPalette::Text, tokens.text_secondary);
  palette.setColor(QPalette::Inactive, QPalette::ButtonText,
                   tokens.text_secondary);
  palette.setColor(QPalette::Inactive, QPalette::Highlight,
                   tokens.surface_selected);
  palette.setColor(QPalette::Inactive, QPalette::HighlightedText, tokens.text);
  palette.setColor(QPalette::Inactive, QPalette::Link, tokens.accent_text);

  palette.setColor(QPalette::Disabled, QPalette::WindowText, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::Text, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::PlaceholderText,
                   tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::Base, tokens.surface_sunken);
  palette.setColor(QPalette::Disabled, QPalette::Button, tokens.surface_sunken);
  palette.setColor(QPalette::Disabled, QPalette::Highlight,
                   tokens.border_subtle);
  palette.setColor(QPalette::Disabled, QPalette::HighlightedText,
                   tokens.disabled);
  palette.setColor(QPalette::Disabled, QPalette::Link, tokens.disabled);
  return palette;
}

void ApplyWidgetTheme(QWidget* widget, style::Tone tone, bool recursive,
                      ThemeAppearance appearance) {
  if (widget == nullptr) {
    return;
  }

  const ThemeAppearance resolved =
      ResolveAppearance(appearance, widget->palette());
  const ThemeTokens& tokens = GetThemeTokens(tone, resolved);
  const QPalette palette =
      BuildWidgetPalette(tone, widget->palette(), resolved);
  ApplyPalette(widget, palette, tone, tokens);
  if (!recursive) {
    return;
  }

  const QList<QWidget*> descendants = widget->findChildren<QWidget*>();
  for (QWidget* descendant : descendants) {
    ApplyPalette(descendant, palette, tone, tokens);
  }
}

}  // namespace autoviz::widget
