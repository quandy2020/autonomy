/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file proxy.hpp
 * @brief Fusion @c QProxyStyle — scroll metrics, menu pills, palette hooks.
 *
 * Installed by @ref ApplyAppTheme(). Panels menu checkboxes primarily use QSS;
 * this proxy paints teal hover pills on glass menus and tweaks pixel metrics.
 *
 * @see ApplyAppTheme()
 * @see PrepareAppMenu()
 * @see IsActiveGlassMenuPopup()
 */

#pragma once

#include <QColor>
#include <QProxyStyle>
#include <QSize>

class QPainter;
class QStyleOption;
class QStyleOptionTab;
class QWidget;

namespace autoviz {

/**
 * @class ProxyStyle
 * @brief Fusion proxy — scroll metrics and palette hooks; Panels menu
 *        checkboxes use QSS.
 *
 * ## Custom painting
 *
 * - Menu item hover / selection pills when @ref IsActiveGlassMenuPopup()
 * - Accent-aware highlights via @ref setAccentColor()
 *
 * ## Metrics
 *
 * Overrides scroll-bar and splitter metrics for denser Foxglove-like chrome.
 *
 * @note Prefer updating colors through the setters after theme construction
 *       rather than subclassing further.
 *
 * @see glass::ShellTokens
 * @see AppThemeAccentColor()
 */
class ProxyStyle : public QProxyStyle {
  Q_OBJECT

 public:
  /**
   * @brief Wraps @p base_style (typically Fusion).
   * @param base_style Style to proxy; @c nullptr lets Qt pick a default.
   */
  explicit ProxyStyle(QStyle* base_style = nullptr);

  /**
   * @brief Sets the accent used for highlights and checked affordances.
   * @param accent Accent color (usually @ref AppThemeAccentColor()).
   */
  void setAccentColor(const QColor& accent);

  /**
   * @brief Sets the selection background wash.
   * @param color Selection fill.
   */
  void setSelectionBackground(const QColor& color);

  /**
   * @brief Sets the hover background wash (menus / lists).
   * @param color Hover fill.
   */
  void setHoverBackground(const QColor& color);

  /**
   * @brief Sets the menu shortcut text color.
   * @param color Shortcut label color.
   */
  void setMenuShortcutColor(const QColor& color);

  /**
   * @brief Draws primitives (frames, panels) with glass-aware overrides.
   *
   * @param element Primitive element id.
   * @param option Style option.
   * @param painter Target painter.
   * @param widget Optional associated widget.
   */
  void drawPrimitive(PrimitiveElement element, const QStyleOption* option,
                     QPainter* painter,
                     const QWidget* widget = nullptr) const override;

  /**
   * @brief Draws controls (menu items, tabs) with teal hover pills when active.
   *
   * @param element Control element id.
   * @param option Style option.
   * @param painter Target painter.
   * @param widget Optional associated widget.
   */
  void drawControl(ControlElement element, const QStyleOption* option,
                   QPainter* painter,
                   const QWidget* widget = nullptr) const override;

  /**
   * @brief Adjusts content sizes (e.g. denser menu items).
   *
   * @param type Contents type.
   * @param option Style option.
   * @param size Proposed size.
   * @param widget Optional associated widget.
   * @return Adjusted size.
   */
  QSize sizeFromContents(ContentsType type, const QStyleOption* option,
                         const QSize& size,
                         const QWidget* widget = nullptr) const override;

  /**
   * @brief Style hints (e.g. menu delay, rubber band).
   *
   * @param hint Hint id.
   * @param option Optional style option.
   * @param widget Optional widget.
   * @param return_data Optional return data.
   * @return Hint value.
   */
  int styleHint(StyleHint hint, const QStyleOption* option = nullptr,
                const QWidget* widget = nullptr,
                QStyleHintReturn* return_data = nullptr) const override;

  /**
   * @brief Pixel metrics (scroll bars, splitters, …).
   *
   * @param metric Metric id.
   * @param option Optional style option.
   * @param widget Optional widget.
   * @return Metric in pixels.
   */
  int pixelMetric(PixelMetric metric, const QStyleOption* option = nullptr,
                  const QWidget* widget = nullptr) const override;

 private:
  QColor accent_{0x30, 0x8C, 0xC6};          /**< Highlight accent. */
  QColor selection_bg_{0xE6, 0xF2, 0xFF};    /**< Selection wash. */
  QColor hover_bg_{0xE6, 0xF2, 0xFF};        /**< Hover wash. */
  QColor menu_shortcut_{0x66, 0x66, 0x66};   /**< Menu shortcut text. */
};

}  // namespace autoviz
