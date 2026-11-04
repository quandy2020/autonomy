/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/style.hpp"

#include <QFile>
#include <QRegularExpression>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {
namespace style {
namespace {

QString ReadFile(const QString& relative) {
  QFile file(QStringLiteral(":/autoviz/styles/") + relative);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return {};
  }
  return QString::fromUtf8(file.readAll());
}

/** Drop unresolved {{tokens}} so Qt never fails to parse the stylesheet. */
QString ScrubUnresolved(QString css) {
  static const QRegularExpression kToken(QStringLiteral(R"(\{\{[a-zA-Z0-9_-]+\}\})"));
  return css.replace(kToken, QStringLiteral("transparent"));
}

QString Apply(QString css, const QHash<QString, QString>& map) {
  for (auto it = map.constBegin(); it != map.constEnd(); ++it) {
    css.replace(it.key(), it.value());
  }
  return ScrubUnresolved(std::move(css));
}

QString Section(const QString& css, const QString& name) {
  const QString begin = QStringLiteral("/* @section %1 */").arg(name);
  const int start = css.indexOf(begin);
  if (start < 0) {
    return {};
  }
  const int body = start + begin.size();
  const int stop = css.indexOf(QStringLiteral("/* @end */"), body);
  if (stop < 0) {
    return css.mid(body).trimmed() + QLatin1Char('\n');
  }
  return css.mid(body, stop - body).trimmed() + QLatin1Char('\n');
}

/** Join every @section whose name equals @p prefix or starts with "prefix.".
 *  Property-only bodies (no `{`) are skipped — they are leaf-widget marks and
 *  would invalidate a multi-rule stylesheet if concatenated. */
QString SectionsPrefixed(const QString& css, const QString& prefix) {
  QString out;
  const QString marker = QStringLiteral("/* @section ");
  int pos = 0;
  while (true) {
    const int start = css.indexOf(marker, pos);
    if (start < 0) {
      break;
    }
    const int name_begin = start + marker.size();
    const int name_end = css.indexOf(QStringLiteral(" */"), name_begin);
    if (name_end < 0) {
      break;
    }
    const QString name = css.mid(name_begin, name_end - name_begin).trimmed();
    pos = name_end;
    if (name != prefix && !name.startsWith(prefix + QLatin1Char('.'))) {
      continue;
    }
    const QString body = Section(css, name);
    if (!body.contains(QLatin1Char('{'))) {
      continue;  // property-only — load via "panel/part" on the leaf widget
    }
    out += body;
    if (!out.isEmpty() && !out.endsWith(QLatin1Char('\n'))) {
      out += QLatin1Char('\n');
    }
  }
  return out;
}

bool IsCoreModule(const QString& name) {
  return name == QLatin1String("theme") || name == QLatin1String("menu") ||
         name == QLatin1String("chrome") || name == QLatin1String("widget") ||
         name == QLatin1String("panel");
}

}  // namespace

QHash<QString, QString> tokens(Tone tone) {
  const glass::ShellTokens& s = glass::Shell();
  QHash<QString, QString> t;
  t.insert(QStringLiteral("{{window}}"), glass::Hex(s.window));
  t.insert(QStringLiteral("{{wash}}"), glass::ShellWindowWashCss());
  t.insert(QStringLiteral("{{base}}"), glass::Hex(s.base));
  t.insert(QStringLiteral("{{card}}"), glass::Hex(s.card));
  t.insert(QStringLiteral("{{alt}}"), glass::Hex(s.alternate_base));
  t.insert(QStringLiteral("{{text}}"), glass::Hex(s.text));
  t.insert(QStringLiteral("{{muted}}"), glass::Hex(s.secondary_text));
  t.insert(QStringLiteral("{{button}}"), glass::Hex(s.button));
  t.insert(QStringLiteral("{{button-text}}"), glass::Hex(s.button_text));
  t.insert(QStringLiteral("{{accent}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{accent-pressed}}"), glass::Hex(s.accent_pressed));
  t.insert(QStringLiteral("{{accent-soft}}"), glass::Hex(s.accent_soft));
  // Translucent accent washes for QSS that used to hardcode teal/cyan rgba.
  auto accent_a = [&](int alpha) {
    QColor c = s.accent;
    c.setAlpha(qBound(0, alpha, 255));
    return glass::Rgba(c);
  };
  t.insert(QStringLiteral("{{accent-a08}}"), accent_a(20));
  t.insert(QStringLiteral("{{accent-a10}}"), accent_a(26));
  t.insert(QStringLiteral("{{accent-a14}}"), accent_a(36));
  t.insert(QStringLiteral("{{accent-a16}}"), accent_a(41));
  t.insert(QStringLiteral("{{accent-a18}}"), accent_a(46));
  t.insert(QStringLiteral("{{accent-a20}}"), accent_a(51));
  t.insert(QStringLiteral("{{accent-a22}}"), accent_a(56));
  t.insert(QStringLiteral("{{accent-a24}}"), accent_a(61));
  t.insert(QStringLiteral("{{accent-a26}}"), accent_a(66));
  t.insert(QStringLiteral("{{accent-a28}}"), accent_a(71));
  t.insert(QStringLiteral("{{accent-a32}}"), accent_a(82));
  t.insert(QStringLiteral("{{accent-a35}}"), accent_a(89));
  t.insert(QStringLiteral("{{accent-a40}}"), accent_a(102));
  t.insert(QStringLiteral("{{accent-a55}}"), accent_a(55));
  t.insert(QStringLiteral("{{accent-a70}}"), accent_a(70));
  t.insert(QStringLiteral("{{accent-a90}}"), accent_a(230));
  t.insert(QStringLiteral("{{sel-bg}}"), glass::Hex(s.selection_bg));
  t.insert(QStringLiteral("{{sel-text}}"), glass::Hex(s.selection_text));
  t.insert(QStringLiteral("{{border}}"), glass::Hex(s.border));
  t.insert(QStringLiteral("{{border-light}}"), glass::Hex(s.border_light));
  t.insert(QStringLiteral("{{disabled}}"), glass::Hex(s.disabled_text));
  t.insert(QStringLiteral("{{toolbar}}"), glass::Hex(s.toolbar_bg));
  t.insert(QStringLiteral("{{hover}}"), glass::Hex(s.hover_bg));
  t.insert(QStringLiteral("{{dock-title}}"), glass::Hex(s.dock_title));
  t.insert(QStringLiteral("{{status}}"), glass::Hex(s.status_bar));
  t.insert(QStringLiteral("{{glass-bar}}"), glass::ShellGlassBarGradientCss());
  t.insert(QStringLiteral("{{glass-fill}}"), glass::Rgba(s.glass_fill));
  t.insert(QStringLiteral("{{glass-deep}}"), glass::Rgba(s.glass_fill_deep));
  t.insert(QStringLiteral("{{glass-rim}}"), glass::Rgba(s.glass_rim));
  t.insert(QStringLiteral("{{glass-cool}}"), glass::Rgba(s.glass_cool));
  t.insert(QStringLiteral("{{r-panel}}"), QString::number(s.panel_radius));
  t.insert(QStringLiteral("{{r-title}}"), QString::number(s.title_radius));
  t.insert(QStringLiteral("{{r-ctrl}}"), QString::number(s.control_radius));
  t.insert(QStringLiteral("{{id-content}}"),
           QString::fromLatin1(AppThemeIds::kPanelContent));
  t.insert(QStringLiteral("{{id-toolbar}}"),
           QString::fromLatin1(AppThemeIds::kPanelToolbar));
  t.insert(QStringLiteral("{{id-footer}}"),
           QString::fromLatin1(AppThemeIds::kPanelFooter));
  t.insert(QStringLiteral("{{id-dock-title}}"),
           QString::fromLatin1(AppThemeIds::kDockTitleBar));
  t.insert(QStringLiteral("{{id-title-tools}}"),
           QString::fromLatin1(AppThemeIds::kPanelTitleTools));
  t.insert(QStringLiteral("{{id-hint}}"),
           QString::fromLatin1(AppThemeIds::kHintLabel));
  t.insert(QStringLiteral("{{id-section}}"),
           QString::fromLatin1(AppThemeIds::kSectionTitle));
  t.insert(QStringLiteral("{{id-pi-title}}"),
           QString::fromLatin1(AppThemeIds::kPropertyInspectorTitle));
  t.insert(QStringLiteral("{{id-tree}}"),
           QString::fromLatin1(AppThemeIds::kPanelTree));
  t.insert(QStringLiteral("{{id-segment}}"),
           QString::fromLatin1(AppThemeIds::kSegmentedToggle));
  t.insert(QStringLiteral("{{id-settings}}"),
           QString::fromLatin1(AppThemeIds::kSettingsScroll));
  t.insert(QStringLiteral("{{panels-menu}}"), QString());

  // Panel-family sheets — always derive from Shell so Liquid / Dark / Light
  // stay in sync (hardcoded milky cyan washed out Liquid Glass checkboxes).
  QColor panel_bg = s.glass_fill;
  QColor panel_surface = s.glass_fill_deep;
  QColor panel_raised = s.base;
  if (glass::ShellIsLiquid()) {
    panel_bg.setAlpha(qMax(panel_bg.alpha(), 235));
    panel_surface.setAlpha(qMax(panel_surface.alpha(), 245));
    panel_raised.setAlpha(255);
  } else if (glass::ShellIsDark()) {
    panel_bg.setAlpha(qMax(panel_bg.alpha(), 230));
    panel_surface.setAlpha(qMax(panel_surface.alpha(), 240));
    panel_raised = s.card;
    panel_raised.setAlpha(255);
  } else {
    panel_bg.setAlpha(qMax(panel_bg.alpha(), 200));
    panel_surface.setAlpha(qMax(panel_surface.alpha(), 220));
    panel_raised.setAlpha(255);
  }
  t.insert(QStringLiteral("{{panel-bg}}"), glass::Rgba(panel_bg));
  t.insert(QStringLiteral("{{panel-surface}}"), glass::Rgba(panel_surface));
  t.insert(QStringLiteral("{{panel-border}}"), glass::Rgba(s.glass_rim));
  t.insert(QStringLiteral("{{panel-text}}"), glass::Hex(s.text));
  t.insert(QStringLiteral("{{panel-muted}}"), glass::Hex(s.secondary_text));
  t.insert(QStringLiteral("{{panel-accent}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{panel-accent-pressed}}"),
           glass::Hex(s.accent_pressed));
  t.insert(QStringLiteral("{{panel-surface-raised}}"),
           glass::Rgba(panel_raised));
  // Legacy {{accent-teal}} now follows Shell accent so Liquid Glass (system
  // blue) and Light/Dark stay one palette — dual teal/blue chrome is gone.
  t.insert(QStringLiteral("{{accent-teal}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{accent-color}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{on-accent}}"), QStringLiteral("#FFFFFF"));
  QColor header = s.glass_fill_deep;
  header.setAlpha(qMax(header.alpha(), 220));
  t.insert(QStringLiteral("{{header-start}}"), glass::Rgba(header));
  QColor editor = s.base;
  editor.setAlpha(255);
  t.insert(QStringLiteral("{{editor-bg}}"), glass::Rgba(editor));
  t.insert(QStringLiteral("{{btn-bg}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{btn-hover}}"), glass::Hex(s.accent_pressed));
  t.insert(QStringLiteral("{{color}}"), glass::Hex(s.accent));
  t.insert(QStringLiteral("{{name}}"), QStringLiteral("AutovizPanelPart"));

  Q_UNUSED(tone);
  return t;
}

QString sheet(const QString& path) {
  return sheet(path, tokens(Tone::Shell));
}

QString sheet(const QString& path, Tone tone) {
  return sheet(path, tokens(tone));
}

QString sheet(const QString& path, const QHash<QString, QString>& extra) {
  const QStringList parts = path.split(QLatin1Char('/'), Qt::SkipEmptyParts);
  QString css;

  if (parts.size() == 1) {
    const QString name = parts[0];
    if (IsCoreModule(name)) {
      css = ReadFile(name + QStringLiteral(".qss"));
    } else {
      // Panel family: join all "name" / "name.*" sections from panel.qss
      css = SectionsPrefixed(ReadFile(QStringLiteral("panel.qss")), name);
    }
  } else if (parts.size() == 2) {
    if (parts[0] == QLatin1String("chrome") ||
        parts[0] == QLatin1String("widget")) {
      css = Section(ReadFile(parts[0] + QStringLiteral(".qss")), parts[1]);
    } else if (parts[0] == QLatin1String("panel")) {
      css = SectionsPrefixed(ReadFile(QStringLiteral("panel.qss")), parts[1]);
    } else {
      // panel / part  →  @section panel.part
      css = Section(ReadFile(QStringLiteral("panel.qss")),
                    parts[0] + QLatin1Char('.') + parts[1]);
    }
  } else {
    QString file = path;
    if (!file.endsWith(QLatin1String(".qss"))) {
      file += QStringLiteral(".qss");
    }
    css = ReadFile(file);
  }

  return Apply(std::move(css), extra);
}

QString type(Role role, int size_px, int weight, bool italic, const QString& extra) {
  QString color;
  switch (role) {
    case Role::Muted:
      color = tokens(Tone::Shell).value(QStringLiteral("{{muted}}"));
      break;
    case Role::Body:
      color = tokens(Tone::Shell).value(QStringLiteral("{{text}}"));
      break;
    case Role::Accent:
      color = tokens(Tone::Shell).value(QStringLiteral("{{accent}}"));
      break;
    case Role::PanelMuted:
      color = tokens(Tone::Frost).value(QStringLiteral("{{panel-muted}}"));
      break;
    case Role::PanelBody:
      color = tokens(Tone::Frost).value(QStringLiteral("{{panel-text}}"));
      break;
    case Role::Ok:
      color = QStringLiteral("#059669");
      break;
    case Role::Danger:
      color = QStringLiteral("#dc2626");
      break;
  }
  QString css = QStringLiteral("color: %1; font-size: %2px; font-weight: %3;")
                    .arg(color, QString::number(size_px), QString::number(weight));
  if (italic) {
    css += QStringLiteral(" font-style: italic;");
  }
  if (!extra.isEmpty()) {
    if (!extra.startsWith(QLatin1Char(' '))) {
      css += QLatin1Char(' ');
    }
    css += extra;
  }
  return css;
}

QString mark(Mark kind, const QString& arg, int radius) {
  switch (kind) {
    case Mark::Clear:
      return QStringLiteral("background: transparent;");
    case Mark::Rule:
      return QStringLiteral(
          "background: rgba(0,0,0,0.08); border: none; max-height: 1px;");
    case Mark::Invalid:
      return QStringLiteral("border: 1px solid #c62828;");
    case Mark::Swatch:
      return QStringLiteral(
                 "background: %1; border: 1px solid palette(mid); "
                 "border-radius: %2px;")
          .arg(arg, QString::number(radius));
    case Mark::Preview:
      return QStringLiteral(
          "border: 1px solid rgba(0,0,0,0.15); border-radius: 4px;");
    case Mark::Fill: {
      const QStringList rgb = arg.split(QLatin1Char(','));
      if (rgb.size() != 3) {
        return mark(Mark::Preview);
      }
      return QStringLiteral(
                 "background: rgb(%1,%2,%3); border: 1px solid rgba(0,0,0,0.15); "
                 "border-radius: 4px;")
          .arg(rgb[0].trimmed(), rgb[1].trimmed(), rgb[2].trimmed());
    }
    case Mark::VLine:
      return QStringLiteral("color: %1; margin: 3px 1px;")
          .arg(tokens(Tone::Frost).value(QStringLiteral("{{panel-border}}")));
  }
  return {};
}

}  // namespace style
}  // namespace autoviz
