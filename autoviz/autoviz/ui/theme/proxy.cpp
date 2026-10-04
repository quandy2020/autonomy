/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/proxy.hpp"

#include <QApplication>
#include <QFontMetrics>
#include <QKeySequence>
#include <QLinearGradient>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygon>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleOption>
#include <QStyleOptionMenuItem>
#include <QWidget>

#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/theme/glass.hpp"

namespace autoviz {
namespace {

constexpr int kScrollBarExtent = 6;

// Horizontal row (shortcuts are not shown):
// | pad | icon | gap | check | gap | text | [submenu] | pad |
constexpr int kPadL = 12;
constexpr int kIcon = 16;
constexpr int kGapIconCheck = 8;
constexpr int kCheck = 14;
constexpr int kGapCheckText = 10;
constexpr int kSubmenuW = 12;
constexpr int kPadR = 12;
constexpr int kRowH = 30;
constexpr int kHoverInset = 4;
constexpr int kSepInset = 14;

QColor AccentPillFill(int alpha = 78) {
  QColor c = glass::Shell().accent;
  c.setAlpha(alpha);
  return c;
}

QColor AccentSelectedText() { return glass::Shell().accent_pressed; }

constexpr char kPropMaxLabelW[] = "panelsMaxLabelW";
constexpr char kPropMaxShortcutW[] = "glassMaxShortcutW";
/** App menus (File/Help): same icon/pad as Panels, no checkbox column.
 *  | pad | icon | gap | text | gap | shortcut | pad | */
constexpr int kAppGapIconText = 10;
constexpr int kAppPadR = 28;
constexpr int kShortcutGap = 36;
constexpr int kShortcutSafety = 18;

int AppMenuLeadingWidth() {
  return kPadL + kIcon + kAppGapIconText;
}

QString StripMenuMnemonic(QString text) {
  text.remove(QLatin1Char('&'));
  return text;
}

int MeasureWidth(const QFontMetrics& fm, const QString& text) {
  return qMax(fm.horizontalAdvance(text), fm.boundingRect(text).width());
}

/** Prefer the wider of Native / Portable — macOS glyphs can under-measure. */
int MeasureShortcutWidth(const QFontMetrics& fm, const QKeySequence& sequence) {
  if (sequence.isEmpty()) {
    return 0;
  }
  return qMax(MeasureWidth(fm, sequence.toString(QKeySequence::NativeText)),
              MeasureWidth(fm, sequence.toString(QKeySequence::PortableText))) +
         kShortcutSafety;
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
  return LeadingClusterWidth() + max_label +
         (need_submenu ? kSubmenuW : 0) + kPadR;
}

int SimpleMenuContentWidth(int max_label, int max_shortcut, bool need_submenu) {
  const int shortcut_w = max_shortcut > 0 ? (kShortcutGap + max_shortcut) : 0;
  return AppMenuLeadingWidth() + max_label + shortcut_w +
         (need_submenu ? kSubmenuW : 0) + kAppPadR;
}

bool ObjectHasBoolProp(const QObject* object, const char* name) {
  return object != nullptr && object->property(name).toBool();
}

bool IsPanelsMenuObject(const QObject* object) {
  if (object == nullptr) {
    return false;
  }
  if (object->objectName() == QLatin1String(AppThemeIds::kPanelsMenu)) {
    return true;
  }
  return ObjectHasBoolProp(object, "autovizPanelsMenu");
}

bool IsGlassMenuObject(const QObject* object) {
  if (object == nullptr) {
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

bool WalkParentsFor(const QWidget* widget,
                    bool (*pred)(const QObject*)) {
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

bool IsGlassMenu(const QWidget* widget, const QStyleOption* option = nullptr) {
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

void PaintChromeBar(QPainter* painter, const QRect& rect) {
  painter->save();
  // Opaque milky plate first — prevents sky-blue window wash from tinting chrome.
  painter->fillRect(rect, QColor(252, 253, 253));
  glass::PaintShellGlass(*painter, QRectF(rect), 0.0);
  const glass::ShellTokens shell = glass::Shell();
  painter->setPen(QPen(shell.glass_rim, 1.0));
  painter->drawLine(rect.left(), rect.bottom(), rect.right(), rect.bottom());
  painter->restore();
}

void PaintGlassMenuPanel(QPainter* painter, const QRect& rect) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  // Clear opaque popup corners so the rounded glass card can show through.
  painter->setCompositionMode(QPainter::CompositionMode_Source);
  painter->fillRect(rect, Qt::transparent);
  painter->setCompositionMode(QPainter::CompositionMode_SourceOver);

  constexpr qreal kRadius = 12.0;
  const QRectF card = QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, kRadius, kRadius);
  painter->fillPath(path, QColor(252, 253, 253, 242));
  QLinearGradient sheen(card.topLeft(),
                        QPointF(card.left(), card.top() + 36.0));
  sheen.setColorAt(0.0, QColor(255, 255, 255, 160));
  sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
  painter->fillPath(path, sheen);
  painter->setPen(QPen(QColor(255, 255, 255, 200), 1.0));
  painter->drawPath(path);
  painter->setPen(QPen(QColor(203, 213, 225, 140), 1.0));
  painter->drawRoundedRect(card.adjusted(1.0, 1.0, -1.0, -1.0), kRadius - 1.0,
                           kRadius - 1.0);
  painter->restore();
}

void DrawMenuBarItem(const QStyleOptionMenuItem& opt, QPainter* painter) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled && (opt.state & (QStyle::State_Selected | QStyle::State_Sunken));

  if (selected) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(AccentPillFill());
    painter->drawRoundedRect(opt.rect.adjusted(1, 1, -1, -1), 10, 10);
  }

  QColor text = enabled ? QColor(0x1E, 0x29, 0x3B) : QColor(0x94, 0xA3, 0xB8);
  if (selected && enabled) {
    text = AccentSelectedText();
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

void DrawCheckbox(QPainter* painter, const QRect& box, bool checked, bool enabled,
                  const QColor& accent, const QColor& border, const QColor& bg) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QColor edge = border;
  QColor fill = bg;
  if (!enabled) {
    const glass::ShellTokens shell = glass::Shell();
    edge = shell.border;
    fill = shell.toolbar_bg;
  }
  if (checked) {
    painter->setPen(QPen(accent, 1));
    painter->setBrush(accent);
  } else {
    painter->setPen(QPen(edge, 1));
    painter->setBrush(fill);
  }
  painter->drawRoundedRect(box, 3, 3);
  if (checked) {
    QPen pen(Qt::white, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    QPainterPath path;
    path.moveTo(box.left() + box.width() * 0.22, box.top() + box.height() * 0.52);
    path.lineTo(box.left() + box.width() * 0.42, box.top() + box.height() * 0.72);
    path.lineTo(box.left() + box.width() * 0.78, box.top() + box.height() * 0.30);
    painter->drawPath(path);
  }
  painter->restore();
}

void DrawPanelsMenuItem(const QStyleOptionMenuItem& opt, QPainter* painter,
                        const QColor& accent) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const QRect rect = opt.rect;
  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled && (opt.state & (QStyle::State_Selected | QStyle::State_MouseOver));

  if (opt.menuItemType == QStyleOptionMenuItem::Separator) {
    const int y = rect.top() + rect.height() / 2;
    painter->setPen(QPen(QColor(0xE2, 0xE8, 0xF0), 1));
    painter->drawLine(rect.left() + kSepInset, y, rect.right() - kSepInset, y);
    painter->restore();
    return;
  }

  if (selected) {
    painter->setPen(Qt::NoPen);
    QColor pill = accent.isValid() ? accent : glass::Shell().accent;
    pill.setAlpha(78);
    painter->setBrush(pill);
    painter->drawRoundedRect(rect.adjusted(kHoverInset, 1, -kHoverInset, -1), 8,
                             8);
  }

  QColor text_color =
      enabled ? QColor(0x1E, 0x29, 0x3B) : QColor(0x94, 0xA3, 0xB8);
  if (selected && enabled) {
    text_color = AccentSelectedText();
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
          QSize(kIcon, kIcon), dpr,
          enabled ? QIcon::Normal : QIcon::Disabled,
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
      DrawCheckbox(painter, check_rect, opt.checked, enabled, accent,
                   QColor(0x94, 0xA3, 0xB8), QColor(0xFF, 0xFF, 0xFF));
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
                             const QColor& shortcut_color) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter->setRenderHint(QPainter::TextAntialiasing, true);

  const QRect rect = opt.rect;
  const bool enabled = opt.state & QStyle::State_Enabled;
  const bool selected =
      enabled && (opt.state & (QStyle::State_Selected | QStyle::State_MouseOver));

  if (opt.menuItemType == QStyleOptionMenuItem::Separator) {
    QColor sep = QColor(0xE2, 0xE8, 0xF0);
    const int y = rect.top() + rect.height() / 2;
    painter->setPen(QPen(sep, 1));
    painter->drawLine(rect.left() + kSepInset, y, rect.right() - kSepInset, y);
    painter->restore();
    return;
  }

  if (selected) {
    painter->setPen(Qt::NoPen);
    painter->setBrush(AccentPillFill());
    painter->drawRoundedRect(rect.adjusted(kHoverInset, 1, -kHoverInset, -1), 8,
                             8);
  }

  QColor text_color = enabled ? QColor(0x1E, 0x29, 0x3B) : QColor(0x94, 0xA3, 0xB8);
  if (selected && enabled) {
    text_color = AccentSelectedText();
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
  const int shortcut_w =
      max_shortcut > 0 ? max_shortcut
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
      Qt::AlignVCenter | Qt::AlignLeft | Qt::TextShowMnemonic | Qt::TextSingleLine,
      label);

  if (!shortcut.isEmpty()) {
    painter->setPen(selected ? QColor(0x0F, 0x76, 0x6E) : shortcut_color);
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

}  // namespace

ProxyStyle::ProxyStyle(QStyle* base_style)
    : QProxyStyle(base_style != nullptr
                      ? base_style
                      : QStyleFactory::create(QStringLiteral("Fusion"))) {}

void ProxyStyle::setAccentColor(const QColor& accent) { accent_ = accent; }

void ProxyStyle::setSelectionBackground(const QColor& color) {
  selection_bg_ = color;
}

void ProxyStyle::setHoverBackground(const QColor& color) {
  hover_bg_ = color;
}

void ProxyStyle::setMenuShortcutColor(const QColor& color) {
  menu_shortcut_ = color;
}

void ProxyStyle::drawPrimitive(PrimitiveElement element,
                                      const QStyleOption* option,
                                      QPainter* painter,
                                      const QWidget* widget) const {
  if (element == PE_PanelMenuBar || element == PE_PanelToolBar) {
    if (option != nullptr) {
      PaintChromeBar(painter, option->rect);
    }
    return;
  }
  if (element == PE_PanelMenu && IsGlassMenu(widget, option)) {
    if (option != nullptr) {
      PaintGlassMenuPanel(painter, option->rect);
    }
    return;
  }
  if ((element == PE_IndicatorMenuCheckMark ||
       element == PE_IndicatorItemViewItemCheck) &&
      IsGlassMenu(widget, option)) {
    return;
  }
  QProxyStyle::drawPrimitive(element, option, painter, widget);
}

void ProxyStyle::drawControl(ControlElement element,
                                    const QStyleOption* option, QPainter* painter,
                                    const QWidget* widget) const {
  if (element == CE_MenuBarItem) {
    if (const auto* menu_opt =
            qstyleoption_cast<const QStyleOptionMenuItem*>(option)) {
      DrawMenuBarItem(*menu_opt, painter);
      return;
    }
  }
  if (element == CE_MenuItem && IsGlassMenu(widget, option)) {
    if (const auto* menu_opt =
            qstyleoption_cast<const QStyleOptionMenuItem*>(option)) {
      QStyleOptionMenuItem opt(*menu_opt);
      if (opt.state & (QStyle::State_Selected | QStyle::State_MouseOver)) {
        QPalette pal = opt.palette;
        pal.setColor(QPalette::Highlight, hover_bg_);
        pal.setColor(QPalette::HighlightedText, QColor(0x0F, 0x76, 0x6E));
        opt.palette = pal;
      }
      if (IsPanelsMenu(widget, option)) {
        DrawPanelsMenuItem(opt, painter, accent_);
      } else {
        DrawSimpleGlassMenuItem(opt, painter, menu_shortcut_);
      }
      return;
    }
  }
  QProxyStyle::drawControl(element, option, painter, widget);
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
  return QProxyStyle::sizeFromContents(type, option, size, widget);
}

int ProxyStyle::styleHint(StyleHint hint, const QStyleOption* option,
                                 const QWidget* widget,
                                 QStyleHintReturn* return_data) const {
  if (hint == SH_Menu_Scrollable && IsGlassMenu(widget, option)) {
    return 1;
  }
  return QProxyStyle::styleHint(hint, option, widget, return_data);
}

int ProxyStyle::pixelMetric(PixelMetric metric, const QStyleOption* option,
                                   const QWidget* widget) const {
  switch (metric) {
    case PM_ScrollBarExtent:
      return kScrollBarExtent;
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
      break;
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
