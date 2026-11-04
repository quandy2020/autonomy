/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/proxy.hpp"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QFontMetrics>
#include <QLinearGradient>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygon>
#include <QScrollBar>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleOption>
#include <QStyleOptionMenuItem>
#include <QToolButton>
#include <QWidget>

#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {
namespace {

/** Floating Liquid Glass scrollbar thickness (visual pill is inset further). */
constexpr int kScrollBarExtent = 12;
constexpr int kRegularControlHeight = 44;
constexpr int kHeaderHeight = 32;
constexpr int kIndicatorExtent = 20;
constexpr qreal kControlRadius = 10.0;

// Horizontal row (shortcuts are not shown):
// | pad | icon | gap | check | gap | text | [submenu] | pad |
constexpr int kPadL = 12;
constexpr int kIcon = 16;
constexpr int kGapIconCheck = 8;
constexpr int kCheck = 14;
constexpr int kGapCheckText = 10;
constexpr int kSubmenuW = 12;
constexpr int kPadR = 12;
constexpr int kRowH = 44;
constexpr int kHoverInset = 4;
constexpr int kSepInset = 14;

QColor BlendColor(const QColor& foreground, const QColor& background,
                  int alpha) {
  const int bounded = qBound(0, alpha, 255);
  const int inverse = 255 - bounded;
  return QColor(
      (foreground.red() * bounded + background.red() * inverse) / 255,
      (foreground.green() * bounded + background.green() * inverse) / 255,
      (foreground.blue() * bounded + background.blue() * inverse) / 255,
      (foreground.alpha() * bounded + background.alpha() * inverse) / 255);
}

QColor WithAlpha(QColor color, int alpha) {
  color.setAlpha(qBound(0, alpha, 255));
  return color;
}

QPalette::ColorGroup PaletteGroup(const QStyleOption& option) {
  return option.state.testFlag(QStyle::State_Enabled) ? QPalette::Active
                                                      : QPalette::Disabled;
}

QColor PaletteColor(const QStyleOption& option, QPalette::ColorRole role) {
  return option.palette.color(PaletteGroup(option), role);
}

QColor PaletteColorOr(const QStyleOption& option, QPalette::ColorRole role,
                      const QColor& fallback) {
  const QColor color = PaletteColor(option, role);
  return color.isValid() ? color : fallback;
}

void PaintRoundedControlFrame(QPainter* painter, const QStyleOption& option,
                              const QRect& rect) {
  if (painter == nullptr || rect.isEmpty()) {
    return;
  }
  QColor fill = PaletteColor(option, QPalette::Base);
  const QColor text = PaletteColor(option, QPalette::Text);
  if (option.state.testFlag(QStyle::State_Sunken)) {
    fill = BlendColor(text, fill, 28);
  } else if (option.state.testFlag(QStyle::State_MouseOver)) {
    fill = BlendColor(text, fill, 14);
  }
  const bool focused = option.state.testFlag(QStyle::State_HasFocus);
  const QColor border = focused ? PaletteColor(option, QPalette::Highlight)
                                : PaletteColor(option, QPalette::Mid);
  const qreal width = focused ? 2.0 : 1.0;
  const qreal inset = width * 0.5;

  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setBrush(fill);
  painter->setPen(
      QPen(border, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter->drawRoundedRect(QRectF(rect).adjusted(inset, inset, -inset, -inset),
                           kControlRadius, kControlRadius);
  painter->restore();
}

void PaintPlainToolButtonFrame(QPainter* painter, const QStyleOption& option,
                               const QRect& rect) {
  if (painter == nullptr || rect.isEmpty()) {
    return;
  }
  const bool pressed = option.state.testFlag(QStyle::State_Sunken);
  const bool hovered = option.state.testFlag(QStyle::State_MouseOver);
  const bool selected = option.state.testFlag(QStyle::State_On);
  const bool focused = option.state.testFlag(QStyle::State_HasFocus);
  if (!pressed && !hovered && !selected && !focused) {
    return;
  }

  QColor fill = Qt::transparent;
  if (pressed) {
    fill = PaletteColor(option, QPalette::Text);
    fill.setAlpha(36);
  } else if (selected) {
    fill = PaletteColor(option, QPalette::Highlight);
    fill.setAlpha(48);
  } else if (hovered) {
    fill = PaletteColor(option, QPalette::Text);
    fill.setAlpha(18);
  }

  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setPen(Qt::NoPen);
  painter->setBrush(fill);
  painter->drawRoundedRect(QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5),
                           kControlRadius, kControlRadius);
  if (focused) {
    painter->setBrush(Qt::NoBrush);
    painter->setPen(QPen(PaletteColor(option, QPalette::Highlight), 2.0,
                         Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->drawRoundedRect(QRectF(rect).adjusted(2.0, 2.0, -2.0, -2.0),
                             kControlRadius - 1.0, kControlRadius - 1.0);
  }
  painter->restore();
}

void PaintChevron(QPainter* painter, const QRect& rect, Qt::ArrowType arrow,
                  const QColor& color) {
  if (painter == nullptr || rect.isEmpty()) {
    return;
  }
  constexpr qreal kHalfWidth = 4.0;
  constexpr qreal kHalfHeight = 2.5;
  const QPointF center = QRectF(rect).center();
  QPolygonF points;
  switch (arrow) {
    case Qt::UpArrow:
      points << QPointF(center.x() - kHalfWidth, center.y() + kHalfHeight)
             << QPointF(center.x(), center.y() - kHalfHeight)
             << QPointF(center.x() + kHalfWidth, center.y() + kHalfHeight);
      break;
    case Qt::DownArrow:
      points << QPointF(center.x() - kHalfWidth, center.y() - kHalfHeight)
             << QPointF(center.x(), center.y() + kHalfHeight)
             << QPointF(center.x() + kHalfWidth, center.y() - kHalfHeight);
      break;
    default:
      return;
  }
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setPen(QPen(color, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter->setBrush(Qt::NoBrush);
  painter->drawPolyline(points);
  painter->restore();
}

void PaintSelectionIndicator(QPainter* painter, const QStyleOption& option,
                             bool radio, int extent = kIndicatorExtent) {
  if (painter == nullptr) {
    return;
  }
  const int size =
      qMin(extent, qMin(option.rect.width(), option.rect.height()));
  if (size < 4) {
    return;
  }
  QRectF box(0.0, 0.0, size, size);
  box.moveCenter(QRectF(option.rect).center());
  const bool checked = option.state.testFlag(QStyle::State_On);
  const bool partial = option.state.testFlag(QStyle::State_NoChange);
  const bool selected = checked || partial;
  // Always use Shell tokens — palette Highlight is soft selection_bg (Liquid
  // #CCE4FF) so a white checkmark vanishes; Base/Window may be transparent on
  // glass menus / trees.
  const glass::ShellTokens shell = glass::Shell();
  QColor accent = shell.accent;
  QColor border = selected ? accent : shell.border;
  QColor fill = selected ? accent : shell.base;
  accent.setAlpha(255);
  border.setAlpha(255);
  fill.setAlpha(255);
  if (!option.state.testFlag(QStyle::State_Enabled)) {
    accent.setAlpha(112);
    border.setAlpha(112);
    fill.setAlpha(112);
  }

  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setPen(QPen(border, 1.2));
  painter->setBrush(fill);
  if (radio) {
    painter->drawEllipse(box.adjusted(0.5, 0.5, -0.5, -0.5));
  } else {
    painter->drawRoundedRect(box.adjusted(0.5, 0.5, -0.5, -0.5), 5.0, 5.0);
  }

  if (selected) {
    QColor mark = QColor(Qt::white);
    if (!option.state.testFlag(QStyle::State_Enabled)) {
      mark.setAlpha(176);
    }
    painter->setPen(
        QPen(mark, 1.9, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->setBrush(mark);
    if (radio) {
      const qreal radius = size * 0.23;
      painter->drawEllipse(box.center(), radius, radius);
    } else if (partial) {
      painter->drawLine(QPointF(box.left() + size * 0.25, box.center().y()),
                        QPointF(box.right() - size * 0.25, box.center().y()));
    } else {
      QPainterPath check;
      check.moveTo(box.left() + size * 0.22, box.top() + size * 0.52);
      check.lineTo(box.left() + size * 0.43, box.top() + size * 0.72);
      check.lineTo(box.left() + size * 0.79, box.top() + size * 0.30);
      painter->setBrush(Qt::NoBrush);
      painter->drawPath(check);
    }
  }
  painter->restore();
}

bool UsesDarkAppearance(const QStyleOption& /*option*/) {
  // Glass menus force transparent Window/Base for rounded corners; that makes
  // QPalette::Window look black (alpha-0) and falsely selects dark chrome.
  // Always follow the app shell preference instead.
  return glass::ShellIsDark();
}

QColor MenuPillFill(const QStyleOption& option, const QColor& fallback) {
  // Prefer caller hover tint / shell soft accent — menu Highlight can be weak.
  QColor fill = fallback.isValid() ? fallback : glass::Shell().accent_soft;
  fill.setAlpha(UsesDarkAppearance(option) ? 64 : 44);
  return fill;
}

QColor MenuInk(const QStyleOption& option) {
  Q_UNUSED(option);
  return glass::Shell().text;
}

QColor MenuSelectedText(const QStyleOption& option) {
  Q_UNUSED(option);
  const glass::ShellTokens shell = glass::Shell();
  return glass::ShellIsDark() ? shell.accent : shell.accent_pressed;
}

QColor MenuSecondaryText(const QStyleOption& option, const QColor& fallback) {
  Q_UNUSED(option);
  QColor text = fallback.isValid() ? fallback : glass::Shell().secondary_text;
  text.setAlpha(UsesDarkAppearance(option) ? 176 : 150);
  return text;
}

constexpr char kPropMaxLabelW[] = "panelsMaxLabelW";
constexpr char kPropMaxShortcutW[] = "glassMaxShortcutW";
/** App menus (File/Help): same icon/pad as Panels, no checkbox column.
 *  | pad | icon | gap | text | gap | shortcut | pad | */
constexpr int kAppGapIconText = 10;
constexpr int kAppPadR = 28;
constexpr int kShortcutGap = 36;
constexpr int kShortcutSafety = 18;

int AppMenuLeadingWidth() { return kPadL + kIcon + kAppGapIconText; }

QString StripMenuMnemonic(QString text) {
  text.remove(QLatin1Char('&'));
  return text;
}

int MeasureWidth(const QFontMetrics& fm, const QString& text) {
  return qMax(fm.horizontalAdvance(text), fm.boundingRect(text).width());
}

QString MenuItemLabel(const QString& raw) {
  QString label = raw;
  const int tab = label.indexOf(QLatin1Char('\t'));
  if (tab >= 0) {
    label = label.left(tab);
  }
  return label;
}

QString MenuItemShortcut(const QString& raw) {
  const int tab = raw.indexOf(QLatin1Char('\t'));
  if (tab < 0) {
    return {};
  }
  return raw.mid(tab + 1);
}

int LeadingClusterWidth() {
  return kPadL + kIcon + kGapIconCheck + kCheck + kGapCheckText;
}

int MenuContentWidth(int max_label, bool need_submenu) {
  return LeadingClusterWidth() + max_label + (need_submenu ? kSubmenuW : 0) +
         kPadR;
}

int SimpleMenuContentWidth(int max_label, int max_shortcut, bool need_submenu) {
  const int shortcut_w = max_shortcut > 0 ? (kShortcutGap + max_shortcut) : 0;
  return AppMenuLeadingWidth() + max_label + shortcut_w +
         (need_submenu ? kSubmenuW : 0) + kAppPadR;
}

bool ObjectHasBoolProp(const QObject* object, const char* name) {
  // During item-view layout, styleObject can be a half-destroyed QObject;
  // QObject::property then SEGV's in QtPrivate::compareMemory.
  if (object == nullptr || object->metaObject() == nullptr || name == nullptr) {
    return false;
  }
  const QByteArray key(name);
  const QMetaObject* meta = object->metaObject();
  if (meta->indexOfProperty(name) < 0 &&
      !object->dynamicPropertyNames().contains(key)) {
    return false;
  }
  return object->property(name).toBool();
}

bool IsPanelsMenuObject(const QObject* object) {
  if (object == nullptr || object->metaObject() == nullptr) {
    return false;
  }
  if (object->objectName() == QLatin1String(AppThemeIds::kPanelsMenu)) {
    return true;
  }
  return ObjectHasBoolProp(object, "autovizPanelsMenu");
}

bool IsGlassMenuObject(const QObject* object) {
  if (object == nullptr || object->metaObject() == nullptr) {
    return false;
  }
  if (IsPanelsMenuObject(object)) {
    return true;
  }
  if (object->objectName() == QLatin1String(AppThemeIds::kAppMenu)) {
    return true;
  }
  return ObjectHasBoolProp(object, "autovizGlassMenu");
}

bool IsChangePanelMenuObject(const QObject* object) {
  if (object == nullptr || object->metaObject() == nullptr) {
    return false;
  }
  if (object->objectName() == QLatin1String("AutovizChangePanelMenu")) {
    return true;
  }
  return ObjectHasBoolProp(object, "autovizChangePanelMenu");
}

bool WalkParentsFor(const QWidget* widget, bool (*pred)(const QObject*)) {
  for (const QWidget* parent = widget; parent != nullptr;
       parent = parent->parentWidget()) {
    if (pred(parent)) {
      return true;
    }
  }
  return false;
}

bool IsPanelsMenu(const QWidget* widget, const QStyleOption* option = nullptr) {
  if (IsPanelsMenuObject(widget) ||
      WalkParentsFor(widget, IsPanelsMenuObject)) {
    return true;
  }
  if (option != nullptr && IsPanelsMenuObject(option->styleObject)) {
    return true;
  }
  return false;
}

bool IsChangePanelMenu(const QWidget* widget,
                       const QStyleOption* option = nullptr) {
  if (IsChangePanelMenuObject(widget) ||
      WalkParentsFor(widget, IsChangePanelMenuObject)) {
    return true;
  }
  if (option != nullptr && IsChangePanelMenuObject(option->styleObject)) {
    return true;
  }
  return false;
}

bool IsGlassMenu(const QWidget* widget, const QStyleOption* option = nullptr) {
  // Item views (Displays/Views trees) are never glass menus — skip styleObject
  // walks that UAF during QTreeView layout (QObject::property SEGV).
  if (widget != nullptr &&
      (qobject_cast<const QAbstractItemView*>(widget) != nullptr ||
       qobject_cast<const QAbstractItemView*>(widget->parentWidget()) !=
           nullptr)) {
    return false;
  }
  if (IsActiveGlassMenuPopup()) {
    return true;
  }
  if (IsGlassMenuObject(widget) || WalkParentsFor(widget, IsGlassMenuObject)) {
    return true;
  }
  if (option != nullptr && IsGlassMenuObject(option->styleObject)) {
    return true;
  }
  return false;
}

void PaintChromeBar(QPainter* painter, const QStyleOption& option) {
  const QRect rect = option.rect;
  const glass::ShellTokens shell = glass::Shell();
  painter->save();
  // Opaque plate from shell tokens (light milky or soft charcoal) — never raw
  // QPalette::Window, which can be pure black under a dark desktop theme.
  QColor plate = shell.glass_fill_deep;
  plate.setAlpha(255);
  painter->fillRect(rect, plate);
  glass::PaintShellGlass(*painter, QRectF(rect), 0.0);
  painter->setPen(QPen(shell.glass_rim, 1.0));
  painter->drawLine(rect.left(), rect.bottom(), rect.right(), rect.bottom());
  painter->restore();
}

void PaintGlassMenuPanel(QPainter* painter, const QStyleOption& option) {
  const QRect rect = option.rect;
  const glass::ShellTokens shell = glass::Shell();
  const bool dark = glass::ShellIsDark();
  const bool liquid = glass::ShellIsLiquid();
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  // Clear opaque popup corners so the rounded glass card can show through.
  painter->setCompositionMode(QPainter::CompositionMode_Source);
  painter->fillRect(rect, Qt::transparent);
  painter->setCompositionMode(QPainter::CompositionMode_SourceOver);

  // Liquid Glass menus use a larger radius (qt-liquid-glass Menu / Popover).
  const qreal kRadius = liquid ? 16.0 : 12.0;
  const QRectF card = QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, kRadius, kRadius);

  // Use Shell tokens — menu palette Base/Window are forced transparent for
  // frameless corners and must not drive the fill (that painted solid black).
  QColor material = shell.glass_fill_deep;
  material.setAlpha(dark ? 248 : (liquid ? 228 : 242));
  painter->fillPath(path, material);

  if (liquid) {
    painter->fillPath(path, shell.glass_cool);
    QLinearGradient cool(card.topRight(), card.bottomLeft());
    cool.setColorAt(0.0, QColor(180, 220, 255, 30));
    cool.setColorAt(0.5, QColor(255, 255, 255, 0));
    cool.setColorAt(1.0, QColor(255, 200, 230, 16));
    painter->fillPath(path, cool);
  }

  QColor sheen_start = dark ? QColor(255, 255, 255, 28) : shell.glass_sheen;
  QColor sheen_end = sheen_start;
  sheen_end.setAlpha(0);
  QLinearGradient sheen(card.topLeft(),
                        QPointF(card.left(), card.top() + (liquid ? 48.0 : 36.0)));
  sheen.setColorAt(0.0, sheen_start);
  sheen.setColorAt(1.0, sheen_end);
  painter->fillPath(path, sheen);

  if (liquid) {
    painter->setBrush(Qt::NoBrush);
    painter->setPen(QPen(QColor(255, 90, 120, 40), 1.0));
    painter->drawRoundedRect(card.adjusted(1.0, 1.0, -0.5, -0.5), kRadius,
                             kRadius);
    painter->setPen(QPen(QColor(80, 180, 255, 46), 1.0));
    painter->drawRoundedRect(card.adjusted(0.5, 0.5, -1.0, -1.0), kRadius,
                             kRadius);
  }

  QColor highlight =
      dark ? QColor(255, 255, 255, 38)
           : (liquid ? QColor(255, 255, 255, 190) : QColor(255, 255, 255, 168));
  painter->setPen(QPen(highlight, 1.0));
  painter->drawPath(path);
  QColor border = shell.glass_rim;
  border.setAlpha(dark ? 180 : (liquid ? 110 : 140));
  painter->setPen(QPen(border, 1.0));
  painter->drawRoundedRect(card.adjusted(1.0, 1.0, -1.0, -1.0), kRadius - 1.0,
                           kRadius - 1.0);
  painter->restore();
}

void DrawMenuBarItem(const QStyleOptionMenuItem& opt, QPainter* painter,
                     const QColor& selection_fallback) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled && (opt.state & (QStyle::State_Selected | QStyle::State_Sunken));

  if (selected) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(MenuPillFill(opt, selection_fallback));
    painter->drawRoundedRect(opt.rect.adjusted(1, 1, -1, -1), 10, 10);
  }

  QColor text = enabled ? MenuInk(opt) : glass::Shell().disabled_text;
  if (selected && enabled) {
    text = MenuSelectedText(opt);
  }
  QFont font = opt.font;
  if (selected) {
    font.setWeight(QFont::DemiBold);
  }
  painter->setFont(font);
  painter->setPen(text);
  painter->drawText(opt.rect, Qt::AlignCenter,
                    StripMenuMnemonic(MenuItemLabel(opt.text)));
  painter->restore();
}

void DrawCheckbox(QPainter* painter, const QRect& box, bool checked,
                  bool enabled, const QColor& accent, const QColor& border,
                  const QColor& bg) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  const glass::ShellTokens shell = glass::Shell();
  QColor ink = accent.isValid() && accent.alpha() > 0 ? accent : shell.accent;
  QColor edge = border.isValid() && border.alpha() > 0 ? border : shell.border;
  QColor fill = bg.isValid() && bg.alpha() > 0 ? bg : shell.base;
  ink.setAlpha(255);
  edge.setAlpha(255);
  fill.setAlpha(255);
  if (!enabled) {
    ink.setAlpha(112);
    edge.setAlpha(112);
    fill.setAlpha(112);
  }
  if (checked) {
    painter->setPen(QPen(ink, 1.2));
    painter->setBrush(ink);
  } else {
    painter->setPen(QPen(edge, 1.2));
    painter->setBrush(fill);
  }
  painter->drawRoundedRect(box, 3, 3);
  if (checked) {
    QColor mark(Qt::white);
    if (!enabled) {
      mark.setAlpha(144);
    }
    QPen pen(mark, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    QPainterPath path;
    path.moveTo(box.left() + box.width() * 0.22,
                box.top() + box.height() * 0.52);
    path.lineTo(box.left() + box.width() * 0.42,
                box.top() + box.height() * 0.72);
    path.lineTo(box.left() + box.width() * 0.78,
                box.top() + box.height() * 0.30);
    painter->drawPath(path);
  }
  painter->restore();
}

void DrawPanelsMenuItem(const QStyleOptionMenuItem& opt, QPainter* painter,
                        const QColor& accent_fallback,
                        const QColor& hover_fallback) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const QRect rect = opt.rect;
  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled &&
      (opt.state & (QStyle::State_Selected | QStyle::State_MouseOver));

  const glass::ShellTokens shell = glass::Shell();
  if (opt.menuItemType == QStyleOptionMenuItem::Separator) {
    const int y = rect.top() + rect.height() / 2;
    painter->setPen(QPen(shell.border, 1));
    painter->drawLine(rect.left() + kSepInset, y, rect.right() - kSepInset, y);
    painter->restore();
    return;
  }

  if (selected) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(MenuPillFill(opt, hover_fallback));
    painter->drawRoundedRect(rect.adjusted(kHoverInset, 1, -kHoverInset, -1), 8,
                             8);
  }

  QColor text_color = enabled ? MenuInk(opt) : shell.disabled_text;
  if (selected && enabled) {
    text_color = MenuSelectedText(opt);
  }

  const int mid_y = rect.top() + rect.height() / 2;
  const QFontMetrics fm(opt.fontMetrics);
  const int text_h = fm.height();
  const int band_h = qMax(kCheck, qMax(kIcon, text_h));
  const int band_top = mid_y - band_h / 2;
  const int text_y = band_top + (band_h - text_h) / 2;

  const QString label = MenuItemLabel(opt.text);
  const bool is_submenu = opt.menuItemType == QStyleOptionMenuItem::SubMenu;

  int max_label = 0;
  if (opt.styleObject != nullptr) {
    max_label = opt.styleObject->property(kPropMaxLabelW).toInt();
  }
  if (max_label <= 0) {
    max_label = MeasureWidth(fm, StripMenuMnemonic(label));
  }

  int x = rect.left() + kPadL;

  // 1) Icon — force exact 20×20 slot (avoids mixed intrinsic pixmap sizes).
  {
    const QRect icon_rect(x, band_top + (band_h - kIcon) / 2, kIcon, kIcon);
    if (!opt.icon.isNull()) {
      const qreal dpr =
          painter->device() != nullptr
              ? painter->device()->devicePixelRatioF()
              : (qApp != nullptr ? qApp->devicePixelRatio() : 1.0);
      const QPixmap pm = opt.icon.pixmap(
          QSize(kIcon, kIcon), dpr, enabled ? QIcon::Normal : QIcon::Disabled,
          selected ? QIcon::On : QIcon::Off);
      if (!pm.isNull()) {
        painter->drawPixmap(icon_rect, pm);
      } else {
        opt.icon.paint(painter, icon_rect, Qt::AlignCenter,
                       enabled ? QIcon::Normal : QIcon::Disabled,
                       selected ? QIcon::On : QIcon::Off);
      }
    }
    x += kIcon + kGapIconCheck;
  }

  // 2) Checkbox slot (always reserved so text columns align)
  {
    const QRect check_rect(x, band_top + (band_h - kCheck) / 2, kCheck, kCheck);
    if (opt.checkType != QStyleOptionMenuItem::NotCheckable) {
      const QColor check_accent =
          accent_fallback.isValid() ? accent_fallback : shell.accent;
      DrawCheckbox(painter, check_rect, opt.checked, enabled, check_accent,
                   shell.border, shell.base);
    }
    x += kCheck + kGapCheckText;
  }

  // 3) Text — shortcuts intentionally not drawn
  const int label_right = rect.right() - kPadR - (is_submenu ? kSubmenuW : 0);
  const int label_w = qMax(0, label_right - x);
  QFont font = opt.font;
  if (selected && enabled) {
    font.setWeight(QFont::DemiBold);
  }
  painter->setPen(text_color);
  painter->setFont(font);
  painter->drawText(QRect(x, text_y, label_w, text_h),
                    Qt::AlignVCenter | Qt::AlignLeft | Qt::TextShowMnemonic |
                        Qt::TextSingleLine,
                    label);

  if (is_submenu) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(text_color);
    const int ax = rect.right() - kPadR - 2;
    QPolygon arrow;
    arrow << QPoint(ax - 5, mid_y - 4) << QPoint(ax - 5, mid_y + 4)
          << QPoint(ax + 1, mid_y);
    painter->drawPolygon(arrow);
  }

  painter->restore();
}

void DrawSimpleGlassMenuItem(const QStyleOptionMenuItem& opt, QPainter* painter,
                             const QColor& hover_fallback,
                             const QColor& shortcut_fallback) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const QRect rect = opt.rect;
  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled &&
      (opt.state & (QStyle::State_Selected | QStyle::State_MouseOver));

  const glass::ShellTokens shell = glass::Shell();
  if (opt.menuItemType == QStyleOptionMenuItem::Separator) {
    const int y = rect.top() + rect.height() / 2;
    painter->setPen(QPen(shell.border, 1));
    painter->drawLine(rect.left() + kSepInset, y, rect.right() - kSepInset, y);
    painter->restore();
    return;
  }

  if (selected) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(MenuPillFill(opt, hover_fallback));
    painter->drawRoundedRect(rect.adjusted(kHoverInset, 1, -kHoverInset, -1), 8,
                             8);
  }

  QColor text_color = enabled ? MenuInk(opt) : shell.disabled_text;
  if (selected && enabled) {
    text_color = MenuSelectedText(opt);
  }

  const QFontMetrics fm(opt.fontMetrics);
  const QString label = MenuItemLabel(opt.text);
  const QString shortcut = MenuItemShortcut(opt.text);
  const bool is_submenu = opt.menuItemType == QStyleOptionMenuItem::SubMenu;

  int max_shortcut = 0;
  if (opt.styleObject != nullptr) {
    max_shortcut = opt.styleObject->property(kPropMaxShortcutW).toInt();
  }

  const int mid_y = rect.top() + rect.height() / 2;
  const int text_h = fm.height();
  const int text_y = mid_y - text_h / 2;
  const int shortcut_w = max_shortcut > 0
                             ? max_shortcut
                             : (MeasureWidth(fm, shortcut) + kShortcutSafety);
  const int right_reserve =
      kAppPadR + (is_submenu ? kSubmenuW : 0) +
      (!shortcut.isEmpty() ? (kShortcutGap + shortcut_w) : 0);

  // Icon column (same 20px slate icons as Panels).
  const int icon_x = rect.left() + kPadL;
  const int icon_y = mid_y - kIcon / 2;
  if (!opt.icon.isNull()) {
    opt.icon.paint(painter, QRect(icon_x, icon_y, kIcon, kIcon),
                   Qt::AlignCenter, enabled ? QIcon::Normal : QIcon::Disabled,
                   selected ? QIcon::On : QIcon::Off);
  }

  const int text_x = rect.left() + AppMenuLeadingWidth();
  QFont font = opt.font;
  if (selected) {
    font.setWeight(QFont::DemiBold);
  }
  painter->setFont(font);
  painter->setPen(text_color);
  painter->drawText(
      QRect(text_x, text_y,
            qMax(0, rect.width() - AppMenuLeadingWidth() - right_reserve),
            text_h),
      Qt::AlignVCenter | Qt::AlignLeft | Qt::TextShowMnemonic |
          Qt::TextSingleLine,
      label);

  if (!shortcut.isEmpty()) {
    painter->setPen(selected ? MenuSelectedText(opt)
                             : MenuSecondaryText(opt, shortcut_fallback));
    QFont sc = opt.font;
    sc.setWeight(QFont::Normal);
    painter->setFont(sc);
    const int sc_right = rect.right() - kAppPadR - (is_submenu ? kSubmenuW : 0);
    const int sc_x = sc_right - shortcut_w;
    painter->drawText(QRect(sc_x, text_y, shortcut_w, text_h),
                      Qt::AlignVCenter | Qt::AlignRight | Qt::TextSingleLine,
                      shortcut);
  }

  if (is_submenu) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(text_color);
    const int ax = rect.right() - kAppPadR + 4;
    QPolygon arrow;
    arrow << QPoint(ax - 5, mid_y - 4) << QPoint(ax - 5, mid_y + 4)
          << QPoint(ax + 1, mid_y);
    painter->drawPolygon(arrow);
  }

  painter->restore();
}

/**
 * RViz2-style item-view checkbox: white square, gray rim, dark checkmark.
 * Fusion + transparent tree palettes otherwise paint solid black indicators.
 */
void PaintRvizItemViewCheck(QPainter* painter, const QStyleOption* option) {
  if (painter == nullptr || option == nullptr) {
    return;
  }
  PaintSelectionIndicator(painter, *option, false, 16);
}

}  // namespace

ProxyStyle::ProxyStyle(QStyle* base_style)
    : QProxyStyle(base_style != nullptr
                      ? base_style
                      : QStyleFactory::create(QStringLiteral("Fusion"))) {}

void ProxyStyle::setAccentColor(const QColor& accent) { accent_ = accent; }

void ProxyStyle::setSelectionBackground(const QColor& color) {
  selection_bg_ = color;
}

void ProxyStyle::setHoverBackground(const QColor& color) { hover_bg_ = color; }

void ProxyStyle::setMenuShortcutColor(const QColor& color) {
  menu_shortcut_ = color;
}

void ProxyStyle::drawPrimitive(PrimitiveElement element,
                               const QStyleOption* option, QPainter* painter,
                               const QWidget* widget) const {
  if (element == PE_PanelMenuBar || element == PE_PanelToolBar) {
    if (option != nullptr) {
      PaintChromeBar(painter, *option);
    }
    return;
  }
  if (element == PE_PanelMenu && IsGlassMenu(widget, option)) {
    if (option != nullptr) {
      PaintGlassMenuPanel(painter, *option);
    }
    return;
  }
  if ((element == PE_IndicatorMenuCheckMark ||
       element == PE_IndicatorItemViewItemCheck) &&
      IsGlassMenu(widget, option)) {
    return;
  }
  if (element == PE_PanelLineEdit && option != nullptr) {
    PaintRoundedControlFrame(painter, *option, option->rect);
    return;
  }
  if (element == PE_IndicatorCheckBox && option != nullptr) {
    PaintSelectionIndicator(painter, *option, false);
    return;
  }
  if (element == PE_IndicatorRadioButton && option != nullptr) {
    PaintSelectionIndicator(painter, *option, true);
    return;
  }
  // Displays / Views property trees: RViz2 checkbox chrome (not Fusion black
  // fill).
  if (element == PE_IndicatorItemViewItemCheck && option != nullptr) {
    const QWidget* host = widget;
    if (host != nullptr &&
        qobject_cast<const QAbstractItemView*>(host) == nullptr) {
      host = host->parentWidget();
    }
    if (host == nullptr ||
        qobject_cast<const QAbstractItemView*>(host) != nullptr) {
      PaintRvizItemViewCheck(painter, option);
      return;
    }
  }
  QProxyStyle::drawPrimitive(element, option, painter, widget);
}

void ProxyStyle::drawControl(ControlElement element, const QStyleOption* option,
                             QPainter* painter, const QWidget* widget) const {
  if (element == CE_HeaderSection && option != nullptr) {
    painter->save();
    painter->fillRect(option->rect,
                      PaletteColor(*option, QPalette::AlternateBase));
    painter->setPen(QPen(PaletteColor(*option, QPalette::Mid), 1.0));
    painter->drawLine(option->rect.bottomLeft(), option->rect.bottomRight());
    painter->restore();
    return;
  }
  if (element == CE_MenuBarItem) {
    if (const auto* menu_opt =
            qstyleoption_cast<const QStyleOptionMenuItem*>(option)) {
      DrawMenuBarItem(*menu_opt, painter, selection_bg_);
      return;
    }
  }
  if (element == CE_MenuItem && IsGlassMenu(widget, option)) {
    if (const auto* menu_opt =
            qstyleoption_cast<const QStyleOptionMenuItem*>(option)) {
      QStyleOptionMenuItem opt(*menu_opt);
      if (IsPanelsMenu(widget, option)) {
        DrawPanelsMenuItem(opt, painter, accent_, hover_bg_);
      } else {
        DrawSimpleGlassMenuItem(opt, painter, hover_bg_, menu_shortcut_);
      }
      return;
    }
  }
  QProxyStyle::drawControl(element, option, painter, widget);
}

void ProxyStyle::drawComplexControl(ComplexControl control,
                                    const QStyleOptionComplex* option,
                                    QPainter* painter,
                                    const QWidget* widget) const {
  if (option == nullptr || painter == nullptr) {
    QProxyStyle::drawComplexControl(control, option, painter, widget);
    return;
  }

  if (control == CC_ToolButton) {
    const auto* tool = qstyleoption_cast<const QStyleOptionToolButton*>(option);
    const auto* button = qobject_cast<const QToolButton*>(widget);
    if (tool == nullptr || button == nullptr) {
      QProxyStyle::drawComplexControl(control, option, painter, widget);
      return;
    }

    if (button->autoRaise()) {
      PaintPlainToolButtonFrame(painter, *tool, tool->rect);
    } else {
      PaintRoundedControlFrame(painter, *tool, tool->rect);
    }
    const bool has_menu =
        tool->features.testFlag(QStyleOptionToolButton::HasMenu);
    constexpr int kMenuIndicatorWidth = 30;
    QRect content = tool->rect;
    QRect arrow_rect;
    if (has_menu) {
      arrow_rect = tool->rect;
      if (tool->direction == Qt::RightToLeft) {
        arrow_rect.setRight(arrow_rect.left() + kMenuIndicatorWidth - 1);
        content.adjust(kMenuIndicatorWidth + 4, 0, -12, 0);
      } else {
        arrow_rect.setLeft(arrow_rect.right() - kMenuIndicatorWidth + 1);
        content.adjust(12, 0, -kMenuIndicatorWidth - 4, 0);
      }
    } else {
      // Keep padding tight so TextBesideIcon labels are not elided inside a
      // correctly sized toolbar button.
      const int horizontal_padding = button->autoRaise() ? 4 : 8;
      content.adjust(horizontal_padding, 0, -horizontal_padding, 0);
    }

    QColor foreground = PaletteColor(*tool, QPalette::ButtonText);
    if (button->autoRaise() && tool->state.testFlag(State_On) &&
        tool->state.testFlag(State_Enabled)) {
      foreground = PaletteColor(*tool, QPalette::Highlight);
    }
    if (tool->features.testFlag(QStyleOptionToolButton::Arrow) &&
        tool->icon.isNull() && tool->text.isEmpty()) {
      PaintChevron(painter, content, tool->arrowType, foreground);
    } else if (tool->icon.isNull() && !tool->text.isEmpty()) {
      const QFont font = widget->font();
      const QFontMetrics metrics(font);
      const QString label = metrics.elidedText(tool->text, Qt::ElideRight,
                                               qMax(0, content.width()));
      painter->save();
      painter->setFont(font);
      painter->setPen(foreground);
      painter->drawText(content,
                        Qt::AlignLeading | Qt::AlignVCenter |
                            Qt::TextShowMnemonic | Qt::TextSingleLine,
                        label);
      painter->restore();
    } else {
      QStyleOptionToolButton label(*tool);
      label.rect = content;
      label.features.setFlag(QStyleOptionToolButton::Arrow, false);
      label.arrowType = Qt::NoArrow;
      QPalette palette = label.palette;
      palette.setColor(PaletteGroup(label), QPalette::ButtonText, foreground);
      label.palette = palette;
      QProxyStyle::drawControl(CE_ToolButtonLabel, &label, painter, widget);
    }

    if (has_menu) {
      PaintChevron(painter, arrow_rect, Qt::DownArrow, foreground);
    }
    return;
  }

  if (control == CC_ComboBox) {
    const auto* combo = qstyleoption_cast<const QStyleOptionComboBox*>(option);
    if (combo == nullptr) {
      QProxyStyle::drawComplexControl(control, option, painter, widget);
      return;
    }
    if (combo->frame) {
      PaintRoundedControlFrame(painter, *combo, combo->rect);
    }
    if (combo->subControls.testFlag(SC_ComboBoxArrow)) {
      const QRect arrow = QProxyStyle::subControlRect(CC_ComboBox, combo,
                                                      SC_ComboBoxArrow, widget);
      PaintChevron(painter, arrow, Qt::DownArrow,
                   PaletteColor(*combo, QPalette::Text));
    }
    return;
  }

  if (control == CC_SpinBox) {
    const auto* spin = qstyleoption_cast<const QStyleOptionSpinBox*>(option);
    if (spin == nullptr) {
      QProxyStyle::drawComplexControl(control, option, painter, widget);
      return;
    }
    if (spin->frame) {
      PaintRoundedControlFrame(painter, *spin, spin->rect);
    }
    if (spin->buttonSymbols != QAbstractSpinBox::NoButtons) {
      const QColor arrow_color = PaletteColor(*spin, QPalette::Text);
      if (spin->subControls.testFlag(SC_SpinBoxUp)) {
        PaintChevron(
            painter,
            QProxyStyle::subControlRect(CC_SpinBox, spin, SC_SpinBoxUp, widget),
            Qt::UpArrow, arrow_color);
      }
      if (spin->subControls.testFlag(SC_SpinBoxDown)) {
        PaintChevron(painter,
                     QProxyStyle::subControlRect(CC_SpinBox, spin,
                                                 SC_SpinBoxDown, widget),
                     Qt::DownArrow, arrow_color);
      }
    }
    return;
  }

  if (control == CC_ScrollBar) {
    const auto* slider = qstyleoption_cast<const QStyleOptionSlider*>(option);
    if (slider == nullptr) {
      QProxyStyle::drawComplexControl(control, option, painter, widget);
      return;
    }
    // Own the entire scrollbar — skip Fusion arrows / groove chrome.
    const QRect groove = QProxyStyle::subControlRect(
        CC_ScrollBar, slider, SC_ScrollBarGroove, widget);
    const QRect handle = QProxyStyle::subControlRect(
        CC_ScrollBar, slider, SC_ScrollBarSlider, widget);
    const glass::ShellTokens shell = glass::Shell();
    const bool hovered = slider->state.testFlag(State_MouseOver) ||
                         slider->state.testFlag(State_Sunken);
    const bool active =
        slider->activeSubControls.testFlag(SC_ScrollBarSlider) ||
        slider->state.testFlag(State_Sunken);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(Qt::NoPen);

    // No hard track — only a whisper of glass when hovered (Liquid Glass).
    if (hovered && !groove.isEmpty()) {
      QColor track = shell.glass_fill;
      track.setAlpha(glass::ShellIsDark() ? 40 : 70);
      QRectF track_rect = QRectF(groove);
      if (slider->orientation == Qt::Vertical) {
        track_rect.adjust(3.5, 4.0, -3.5, -4.0);
      } else {
        track_rect.adjust(4.0, 3.5, -4.0, -3.5);
      }
      painter->setBrush(track);
      painter->drawRoundedRect(track_rect, 6.0, 6.0);
    }

    if (!handle.isEmpty() && handle.width() > 2 && handle.height() > 2) {
      QRectF thumb(handle);
      if (slider->orientation == Qt::Vertical) {
        // Floating capsule inset from the list edge.
        const qreal inset = hovered ? 2.5 : 3.5;
        thumb.adjust(inset, 2.0, -inset, -2.0);
        thumb.setWidth(qMax(4.0, thumb.width()));
      } else {
        const qreal inset = hovered ? 2.5 : 3.5;
        thumb.adjust(2.0, inset, -2.0, -inset);
        thumb.setHeight(qMax(4.0, thumb.height()));
      }
      const qreal radius =
          qMin(thumb.width(), thumb.height()) * 0.5;

      // Frosted pill body.
      QColor body = shell.glass_fill_deep;
      if (glass::ShellIsDark()) {
        body = QColor(255, 255, 255, active ? 120 : (hovered ? 88 : 64));
      } else {
        body = shell.accent;
        body.setAlpha(active ? 170 : (hovered ? 125 : 88));
      }
      painter->setBrush(body);
      painter->drawRoundedRect(thumb, radius, radius);

      // Specular glass sheen.
      QLinearGradient sheen(thumb.topLeft(),
                            slider->orientation == Qt::Vertical
                                ? thumb.topRight()
                                : thumb.bottomLeft());
      sheen.setColorAt(0.0, QColor(255, 255, 255, active ? 110 : 70));
      sheen.setColorAt(0.55, QColor(255, 255, 255, 18));
      sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
      painter->setBrush(sheen);
      painter->drawRoundedRect(thumb, radius, radius);

      // Soft rim so the pill reads on both light and dark glass plates.
      QColor rim = QColor(255, 255, 255, hovered ? 140 : 90);
      if (glass::ShellIsDark()) {
        rim = QColor(255, 255, 255, hovered ? 70 : 40);
      }
      painter->setBrush(Qt::NoBrush);
      painter->setPen(QPen(rim, 1.0));
      painter->drawRoundedRect(thumb.adjusted(0.5, 0.5, -0.5, -0.5),
                               radius, radius);
    }
    painter->restore();
    return;
  }

  QProxyStyle::drawComplexControl(control, option, painter, widget);
}

QSize ProxyStyle::sizeFromContents(ContentsType type,
                                   const QStyleOption* option,
                                   const QSize& size,
                                   const QWidget* widget) const {
  if (type == CT_MenuItem && IsGlassMenu(widget, option)) {
    if (const auto* menu_opt =
            qstyleoption_cast<const QStyleOptionMenuItem*>(option)) {
      if (menu_opt->menuItemType == QStyleOptionMenuItem::Separator) {
        return QSize(size.width(), 17);
      }
      int max_label = 0;
      int max_shortcut = 0;
      if (widget != nullptr) {
        max_label = widget->property(kPropMaxLabelW).toInt();
        max_shortcut = widget->property(kPropMaxShortcutW).toInt();
      }
      if (max_label <= 0) {
        const QFontMetrics fm(menu_opt->fontMetrics);
        max_label =
            MeasureWidth(fm, StripMenuMnemonic(MenuItemLabel(menu_opt->text)));
      }
      const bool submenu =
          menu_opt->menuItemType == QStyleOptionMenuItem::SubMenu;
      if (IsPanelsMenu(widget, option)) {
        return QSize(MenuContentWidth(max_label, submenu), kRowH);
      }
      return QSize(SimpleMenuContentWidth(max_label, max_shortcut, submenu),
                   kRowH);
    }
  }
  QSize result = QProxyStyle::sizeFromContents(type, option, size, widget);
  switch (type) {
    case CT_LineEdit:
    case CT_ComboBox:
    case CT_SpinBox:
    case CT_CheckBox:
    case CT_RadioButton:
      result.setHeight(qMax(result.height(), kRegularControlHeight));
      break;
    case CT_ToolButton:
      if (const auto* button = qobject_cast<const QToolButton*>(widget);
          button != nullptr) {
        const auto* tool =
            qstyleoption_cast<const QStyleOptionToolButton*>(option);
        // Icon-only chrome (+/− / dock toggles) stays square.
        if (button->objectName() ==
                QLatin1String("AutovizToolbarChromeButton") ||
            button->objectName() ==
                QLatin1String("AutovizToolbarSidebarToggle")) {
          result.setWidth(qMax(result.width(), 28));
          result.setHeight(qMax(result.height(), 28));
          break;
        }
        const Qt::ToolButtonStyle style =
            tool != nullptr ? tool->toolButtonStyle : button->toolButtonStyle();
        const QString text = tool != nullptr ? tool->text : button->text();
        const QSize icon_size =
            tool != nullptr ? tool->iconSize : button->iconSize();
        const bool has_icon =
            tool != nullptr ? !tool->icon.isNull() : !button->icon().isNull();
        const bool has_menu =
            tool != nullptr &&
            tool->features.testFlag(QStyleOptionToolButton::HasMenu);
        bool in_command_bar = false;
        bool in_main_toolbar = false;
        for (const QWidget* ancestor = button->parentWidget();
             ancestor != nullptr; ancestor = ancestor->parentWidget()) {
          in_command_bar =
              in_command_bar ||
              ancestor->objectName() == QLatin1String("AutovizCommandBar");
          in_main_toolbar =
              in_main_toolbar ||
              ancestor->objectName() == QLatin1String("AutovizToolBar");
        }

        // Stylesheet sizeHints often collapse TextBesideIcon to icon-only
        // width; CE_ToolButtonLabel then elides ("Move...mera"). Size from
        // icon + full label instead.
        if (style == Qt::ToolButtonTextBesideIcon ||
            style == Qt::ToolButtonTextUnderIcon ||
            style == Qt::ToolButtonTextOnly) {
          const QFontMetrics fm(button->font());
          int width = 12;  // match drawComplexControl padding
          if (has_icon && style != Qt::ToolButtonTextOnly) {
            width += icon_size.width() + 6;
          }
          if (!text.isEmpty()) {
            width += fm.horizontalAdvance(text);
          }
          if (has_menu) {
            width += 18;
          }
          result.setWidth(qMax(result.width(), width));
          const int minimum_height = in_command_bar
                                         ? kRegularControlHeight
                                         : (in_main_toolbar ? 28 : 32);
          result.setHeight(qMax(result.height(), minimum_height));
        } else {
          result.setWidth(qMax(result.width(), 28));
          result.setHeight(qMax(result.height(),
                                in_command_bar ? kRegularControlHeight : 28));
        }
      }
      break;
    case CT_HeaderSection:
      result.setHeight(qMax(result.height(), kHeaderHeight));
      break;
    default:
      break;
  }
  return result;
}

int ProxyStyle::styleHint(StyleHint hint, const QStyleOption* option,
                          const QWidget* widget,
                          QStyleHintReturn* return_data) const {
  // Change-panel picker must show every catalog row; a scrollable QMenu clips
  // the QWidgetAction and forces mouse-wheel browsing.
  if (hint == SH_Menu_Scrollable && IsChangePanelMenu(widget, option)) {
    return 0;
  }
  if (hint == SH_Menu_Scrollable && IsGlassMenu(widget, option)) {
    return 1;
  }
  // Prefer overlay / transient scrollbars where the platform supports them.
  if (hint == SH_ScrollBar_Transient) {
    return 1;
  }
  return QProxyStyle::styleHint(hint, option, widget, return_data);
}

int ProxyStyle::pixelMetric(PixelMetric metric, const QStyleOption* option,
                            const QWidget* widget) const {
  switch (metric) {
    case PM_ScrollBarExtent:
      return kScrollBarExtent;
    case PM_ScrollBarSliderMin:
      return 36;
    case PM_TabBarTabHSpace:
      return 12;
    case PM_TabBarTabVSpace:
      return 6;
    case PM_SmallIconSize:
      if (IsGlassMenu(widget, option)) {
        return kIcon;
      }
      break;
    case PM_IndicatorWidth:
    case PM_ExclusiveIndicatorWidth:
    case PM_IndicatorHeight:
    case PM_ExclusiveIndicatorHeight:
      if (IsGlassMenu(widget, option)) {
        return kCheck;
      }
      return kIndicatorExtent;
    case PM_MenuHMargin:
      if (IsGlassMenu(widget, option)) {
        return 8;
      }
      break;
    case PM_MenuVMargin:
      if (IsGlassMenu(widget, option)) {
        return 8;
      }
      break;
    case PM_ToolBarExtensionExtent:
      return 22;
    case PM_ToolBarHandleExtent:
      return 0;
    case PM_ToolBarItemSpacing:
      return 4;
    case PM_ToolBarItemMargin:
      return 2;
    case PM_DockWidgetSeparatorExtent:
      // Nested MainPanelHost separators were effectively ungrippable at the
      // Fusion default; give a usable hit target for center-panel resize.
      return 6;
    default:
      break;
  }
  return QProxyStyle::pixelMetric(metric, option, widget);
}

}  // namespace autoviz
