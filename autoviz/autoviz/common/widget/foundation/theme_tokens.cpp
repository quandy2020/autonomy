/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

#include <QHash>
#include <QString>
#include <QStringList>

namespace autoviz::widget {
namespace {

QColor ParseCssColor(QString value, const QColor& fallback) {
  value = value.trimmed();
  const bool is_rgb = value.startsWith(QLatin1String("rgb("));
  const bool is_rgba = value.startsWith(QLatin1String("rgba("));
  if (is_rgb || is_rgba) {
    const qsizetype open = value.indexOf(QLatin1Char('('));
    const qsizetype close = value.lastIndexOf(QLatin1Char(')'));
    if (open >= 0 && close > open) {
      const QStringList channels =
          value.mid(open + 1, close - open - 1).split(QLatin1Char(','));
      const qsizetype expected_channels = is_rgba ? 4 : 3;
      if (channels.size() == expected_channels) {
        bool red_ok = false;
        bool green_ok = false;
        bool blue_ok = false;
        const int red = channels[0].trimmed().toInt(&red_ok);
        const int green = channels[1].trimmed().toInt(&green_ok);
        const int blue = channels[2].trimmed().toInt(&blue_ok);
        if (red_ok && green_ok && blue_ok) {
          int alpha = 255;
          if (is_rgba) {
            bool alpha_ok = false;
            const QString alpha_text = channels[3].trimmed();
            const double parsed_alpha = alpha_text.toDouble(&alpha_ok);
            if (!alpha_ok) {
              return fallback;
            }
            alpha = parsed_alpha <= 1.0 ? qRound(parsed_alpha * 255.0)
                                        : qRound(parsed_alpha);
          }
          return QColor(qBound(0, red, 255), qBound(0, green, 255),
                        qBound(0, blue, 255), qBound(0, alpha, 255));
        }
      }
    }
    return fallback;
  }

  const QColor color(value);
  return color.isValid() ? color : fallback;
}

QColor ReadColor(const QHash<QString, QString>& values, const char* key,
                 const QColor& fallback) {
  return ParseCssColor(values.value(QString::fromLatin1(key)), fallback);
}

QColor Blend(const QColor& foreground, const QColor& background, int alpha) {
  const int inverse = 255 - alpha;
  return QColor(
      (foreground.red() * alpha + background.red() * inverse) / 255,
      (foreground.green() * alpha + background.green() * inverse) / 255,
      (foreground.blue() * alpha + background.blue() * inverse) / 255);
}

ThemeTokens BuildTokens(style::Tone tone) {
  const QHash<QString, QString> values = style::tokens(tone);
  ThemeTokens result;
  result.window = ReadColor(values, "{{window}}", QColor(244, 247, 251));
  result.surface = ReadColor(values, "{{card}}", QColor(255, 255, 255));
  result.surface_raised =
      ReadColor(values, "{{panel-surface-raised}}", result.surface);
  result.text = ReadColor(values, "{{text}}", QColor(30, 41, 59));
  result.text_muted = ReadColor(values, "{{muted}}", QColor(100, 116, 139));
  result.accent = ReadColor(values, "{{accent}}", QColor(8, 145, 178));
  result.accent_pressed =
      ReadColor(values, "{{accent-pressed}}", result.accent.darker(112));
  result.accent_hovered = Blend(result.accent, result.surface, 224);
  result.on_accent = ReadColor(values, "{{sel-text}}", QColor(Qt::white));
  result.border = ReadColor(values, "{{border}}", QColor(203, 213, 225));
  result.focus_ring = result.accent;
  result.success = QColor(5, 150, 105);
  result.warning = QColor(217, 119, 6);
  result.danger = QColor(220, 38, 38);
  result.disabled = ReadColor(values, "{{disabled}}", QColor(148, 163, 184));
  result.surface_hovered = Blend(result.accent, result.surface, 24);

  if (tone == style::Tone::Frost) {
    result.surface = ReadColor(values, "{{panel-surface}}", result.surface);
    result.surface_raised =
        ReadColor(values, "{{panel-surface-raised}}", result.surface_raised);
    result.text = ReadColor(values, "{{panel-text}}", result.text);
    result.text_muted = ReadColor(values, "{{panel-muted}}", result.text_muted);
    result.accent = ReadColor(values, "{{panel-accent}}", result.accent);
    result.accent_pressed =
        ReadColor(values, "{{panel-accent-pressed}}", result.accent_pressed);
    result.border = ReadColor(values, "{{panel-border}}", result.border);
    result.focus_ring = result.accent;
    result.accent_hovered = Blend(result.accent, result.surface, 224);
    result.surface_hovered = Blend(result.accent, result.surface, 24);
  }
  return result;
}

struct ThemeTokenCache {
  ThemeTokens shell;
  ThemeTokens frost;
  bool valid = false;
};

ThemeTokenCache& Cache() {
  static ThemeTokenCache cache;
  return cache;
}

}  // namespace

const ThemeTokens& GetThemeTokens(style::Tone tone) {
  ThemeTokenCache& cache = Cache();
  if (!cache.valid) {
    cache.shell = BuildTokens(style::Tone::Shell);
    cache.frost = BuildTokens(style::Tone::Frost);
    cache.valid = true;
  }
  return tone == style::Tone::Frost ? cache.frost : cache.shell;
}

void InvalidateThemeTokens() noexcept { Cache().valid = false; }

}  // namespace autoviz::widget
