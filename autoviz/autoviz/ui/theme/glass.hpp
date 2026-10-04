/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file glass.hpp
 * @brief Shared “frosted glass” design tokens and paint helpers.
 *
 * Two token families:
 * - @ref OverlayTokens — dark navy glass for 3D viewport HUD / floating tools
 * - @ref ShellTokens — cool light frosted surfaces for global Autoviz chrome
 *
 * Access via @ref Overlay() / @ref Shell() singletons (return by value).
 *
 * @see ApplyAppTheme()
 * @see PaintPanelFrostedCard()
 * @see ViewportHudOverlay
 * @see ViewportFloatingToolbar
 */

#pragma once

#include <QColor>
#include <QString>

class QPainter;
class QRectF;

namespace autoviz {
namespace glass {

/**
 * @struct OverlayTokens
 * @brief Dark navy frosted-glass colors for 3D viewport overlays.
 *
 * Used by @ref ViewportHudOverlay and @ref ViewportFloatingToolbar.
 */
struct OverlayTokens {
  QColor fill{15, 23, 42, 18};           /**< Primary glass fill. */
  QColor fill_deep{15, 23, 42, 22};      /**< Deeper fill for contrast. */
  QColor cool_a{56, 189, 248, 16};       /**< Cool cyan wash A. */
  QColor cool_b{125, 211, 252, 6};       /**< Cool cyan wash B. */
  QColor sheen{224, 242, 254, 16};       /**< Top-edge sheen. */
  QColor rim{186, 230, 253, 45};         /**< Outer rim stroke. */
  QColor accent{0x7D, 0xD3, 0xFC};       /**< Accent #7DD3FC. */
  QColor accent_mid{0x93, 0xC5, 0xFD};   /**< Mid accent #93C5FD. */
  QColor accent_soft{0xBA, 0xE6, 0xFD};  /**< Soft accent #BAE6FD. */
  QColor title{186, 230, 253, 150};      /**< Dial / label title. */
  QColor value{240, 249, 255, 220};      /**< Active numeric value. */
  QColor value_idle{186, 230, 253, 90};  /**< Idle / zero value. */
  QColor unit{186, 230, 253, 120};       /**< Unit suffix text. */
  QColor track{186, 230, 253, 42};       /**< Dial track. */
  QColor icon{0xE0, 0xF2, 0xFE};         /**< Toolbar icon idle. */
  QColor icon_hover{0xF0, 0xF9, 0xFF};   /**< Toolbar icon hover. */
  QColor icon_on{0x7D, 0xD3, 0xFC};      /**< Toolbar icon checked. */
  QColor divider{186, 230, 253, 70};     /**< Group divider. */
  QColor button_hover_bg{125, 211, 252, 18};    /**< Button hover fill. */
  QColor button_hover_border{186, 230, 253, 40}; /**< Button hover border. */
  QColor button_on_bg{56, 189, 248, 28};        /**< Button checked fill. */
  QColor button_on_border{125, 211, 252, 70};   /**< Button checked border. */
  int radius = 14;                       /**< Card corner radius (px). */
  int toolbar_radius = 16;               /**< Floating toolbar radius (px). */
};

/**
 * @struct ShellTokens
 * @brief Cool light frosted-glass colors for application chrome.
 *
 * Drives @ref BuildThemePalette() and shell QSS helpers
 * (@ref ShellCssBg(), @ref PaintShellGlass(), …).
 */
struct ShellTokens {
  /** Opaque wash behind frosted chrome (so rgba glass reads as translucent). */
  QColor window{0xEE, 0xF6, 0xF4};           /**< Window wash. */
  QColor window_text{0x1E, 0x29, 0x3B};      /**< Window text. */
  QColor base{0xFF, 0xFF, 0xFF};             /**< Base surface. */
  QColor alternate_base{0xF5, 0xFA, 0xF8};   /**< Alternating row base. */
  QColor card{0xFF, 0xFF, 0xFF};             /**< Card surface. */
  QColor text{0x1E, 0x29, 0x3B};             /**< Primary text. */
  QColor secondary_text{0x64, 0x74, 0x8B};   /**< Secondary / muted text. */
  QColor button{0xF5, 0xFA, 0xF8};           /**< Button face. */
  QColor button_text{0x1E, 0x29, 0x3B};      /**< Button label. */
  QColor accent{0x08, 0x91, 0xB2};           /**< Shell accent (cyan). */
  QColor accent_pressed{0x0E, 0x74, 0x90};   /**< Pressed accent. */
  QColor accent_soft{0x99, 0xF6, 0xE4};      /**< Soft accent wash. */
  QColor selection_bg{0xCC, 0xFB, 0xF1};     /**< Selection background. */
  QColor selection_text{0x1E, 0x29, 0x3B};   /**< Selection text. */
  QColor link{0x08, 0x91, 0xB2};             /**< Hyperlink. */
  QColor border{0xD5, 0xE0, 0xDD};           /**< Default border. */
  QColor border_light{0xE8, 0xF0, 0xEE};     /**< Light border. */
  QColor tooltip_base{0xFF, 0xFF, 0xFF};     /**< Tooltip background. */
  QColor tooltip_text{0x1E, 0x29, 0x3B};     /**< Tooltip text. */
  QColor disabled_text{0x94, 0xA3, 0xB8};    /**< Disabled text. */
  QColor toolbar_bg{0xF7, 0xFA, 0xF9};       /**< Toolbar background. */
  QColor hover_bg{0xE6, 0xF4, 0xF1};         /**< Hover wash. */
  QColor dock_title{0xF7, 0xFA, 0xF9};       /**< Dock title bar. */
  QColor status_bar{0xEE, 0xF6, 0xF4};        /**< Status bar. */

  /** Simulated frosted glass: milky white (乳白), barely any cool tint. */
  QColor glass_fill{255, 255, 255, 220};       /**< Primary glass fill. */
  QColor glass_fill_deep{252, 253, 253, 245};  /**< Deeper glass fill. */
  QColor glass_cool{245, 252, 250, 14};        /**< Near-white whisper cyan. */
  QColor glass_sheen{255, 255, 255, 180};      /**< Top sheen. */
  QColor glass_rim{226, 232, 230, 120};        /**< Soft gray-white rim. */
  int panel_radius = 12;                       /**< Panel card radius (px). */
  int title_radius = 10;                       /**< Title bar radius (px). */
  int control_radius = 10;                     /**< Control radius (px). */
};

/**
 * @brief Returns a copy of the overlay token set.
 * @see OverlayTokens
 */
OverlayTokens Overlay();

/**
 * @brief Returns a copy of the shell token set.
 * @see ShellTokens
 */
ShellTokens Shell();

/**
 * @brief Formats @p c as @c #RRGGBB for QSS.
 * @param c Source color (alpha ignored).
 * @return Hex color string.
 */
QString Hex(const QColor& c);

/**
 * @brief Formats @p c as @c rgba(r,g,b,a) for QSS (alpha 0–1).
 * @param c Source color.
 * @return CSS rgba() string.
 */
QString Rgba(const QColor& c);

/**
 * @brief QSS stop-list for a vertical frosted shell bar (title / toolbar / footer).
 * @return Comma-separated @c qlineargradient stops.
 */
QString ShellGlassBarGradientCss();

/**
 * @brief QSS stop-list for the app window wash behind glass chrome.
 * @return Comma-separated @c qlineargradient stops.
 */
QString ShellWindowWashCss();

/**
 * @brief Common panel CSS color snippets (prefer over hardcoded @c #hex).
 * @{
 */
QString ShellCssBg();      /**< Window / panel background. */
QString ShellCssSurface(); /**< Card / surface fill. */
QString ShellCssBorder();  /**< Default border. */
QString ShellCssText();    /**< Primary text. */
QString ShellCssMuted();   /**< Secondary text. */
QString ShellCssAccent();  /**< Accent color. */
/** @} */

/**
 * @brief Paint a frosted shell plate (title bars, tool strips).
 *
 * @param painter Active painter.
 * @param rect Target rectangle in painter coordinates.
 * @param radius Corner radius in device-independent pixels.
 */
void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius);

}  // namespace glass
}  // namespace autoviz
