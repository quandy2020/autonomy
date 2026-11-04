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
 * @enum ShellAppearance
 * @brief User/OS preference for @ref Shell() chrome tokens.
 *
 * @c kLiquid simulates Apple Liquid Glass materials (Frosted / ClearGlass /
 * Chromatic cues from qt-liquid-glass) via painted layers — native
 * @c NSGlassEffectView is macOS-only and a no-op on Linux/Docker.
 */
enum class ShellAppearance {
  kSystem = 0, /**< Follow desktop color scheme. */
  kLight = 1,  /**< Force milky light chrome. */
  kDark = 2,   /**< Force dark chrome. */
  kLiquid = 3, /**< Simulated Liquid Glass (light, refractive sheen). */
};

/**
 * @brief Sets the shell appearance used by @ref Shell() / paint helpers.
 * @param appearance Preference from Settings (default @ref
 * ShellAppearance::kLight).
 */
void SetShellAppearance(ShellAppearance appearance);

/**
 * @brief Returns the configured shell appearance (not yet resolved for system).
 */
ShellAppearance CurrentShellAppearance();

/**
 * @brief @c true when the resolved shell is dark (after expanding @c kSystem).
 */
bool ShellIsDark();

/**
 * @brief @c true when the user selected @ref ShellAppearance::kLiquid.
 */
bool ShellIsLiquid();

/**
 * @struct OverlayTokens
 * @brief Dark navy frosted-glass colors for 3D viewport overlays.
 *
 * Used by @ref ViewportHudOverlay and @ref ViewportFloatingToolbar.
 */
struct OverlayTokens {
  QColor fill{15, 23, 42, 18};               /**< Primary glass fill. */
  QColor fill_deep{15, 23, 42, 22};          /**< Deeper fill for contrast. */
  QColor cool_a{56, 189, 248, 16};           /**< Cool cyan wash A. */
  QColor cool_b{125, 211, 252, 6};           /**< Cool cyan wash B. */
  QColor sheen{224, 242, 254, 16};           /**< Top-edge sheen. */
  QColor rim{186, 230, 253, 45};             /**< Outer rim stroke. */
  QColor accent{0x7D, 0xD3, 0xFC};           /**< Accent #7DD3FC. */
  QColor accent_mid{0x93, 0xC5, 0xFD};       /**< Mid accent #93C5FD. */
  QColor accent_soft{0xBA, 0xE6, 0xFD};      /**< Soft accent #BAE6FD. */
  QColor title{186, 230, 253, 150};          /**< Dial / label title. */
  QColor value{240, 249, 255, 220};          /**< Active numeric value. */
  QColor value_idle{186, 230, 253, 90};      /**< Idle / zero value. */
  QColor unit{186, 230, 253, 120};           /**< Unit suffix text. */
  QColor track{186, 230, 253, 42};           /**< Dial track. */
  QColor icon{0xE0, 0xF2, 0xFE};             /**< Toolbar icon idle. */
  QColor icon_hover{0xF0, 0xF9, 0xFF};       /**< Toolbar icon hover. */
  QColor icon_on{0x7D, 0xD3, 0xFC};          /**< Toolbar icon checked. */
  QColor divider{186, 230, 253, 70};         /**< Group divider. */
  QColor button_hover_bg{125, 211, 252, 18}; /**< Button hover fill. */
  QColor button_hover_border{186, 230, 253, 40}; /**< Button hover border. */
  QColor button_on_bg{56, 189, 248, 28};         /**< Button checked fill. */
  QColor button_on_border{125, 211, 252, 70};    /**< Button checked border. */
  int radius = 14;         /**< Card corner radius (px). */
  int toolbar_radius = 16; /**< Floating toolbar radius (px). */
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
  QColor window{0xF2, 0xF2, 0xF7};         /**< Grouped background. */
  QColor window_text{0x1C, 0x1C, 0x1E};    /**< Primary label. */
  QColor base{0xFF, 0xFF, 0xFF};           /**< Base surface. */
  QColor alternate_base{0xF2, 0xF2, 0xF7}; /**< Alternating row base. */
  QColor card{0xFF, 0xFF, 0xFF};           /**< Card surface. */
  QColor text{0x1C, 0x1C, 0x1E};           /**< Primary label. */
  QColor secondary_text{0x63, 0x63, 0x66}; /**< Secondary label. */
  QColor button{0xE5, 0xE5, 0xEA};         /**< Secondary fill. */
  QColor button_text{0x1C, 0x1C, 0x1E};    /**< Button label. */
  QColor accent{0x00, 0x7A, 0xFF};         /**< System blue. */
  QColor accent_pressed{0x00, 0x62, 0xCC}; /**< Pressed system blue. */
  QColor accent_soft{0xD9, 0xEC, 0xFF};    /**< Tinted selection fill. */
  QColor selection_bg{0xD9, 0xEC, 0xFF};   /**< Selection background. */
  QColor selection_text{0x1C, 0x1C, 0x1E}; /**< Selection text. */
  QColor link{0x00, 0x66, 0xCC};           /**< Accessible link blue. */
  QColor border{0xC6, 0xC6, 0xC8};         /**< Opaque separator. */
  QColor border_light{0xE5, 0xE5, 0xEA};   /**< Secondary separator. */
  QColor tooltip_base{0xFF, 0xFF, 0xFF};   /**< Tooltip background. */
  QColor tooltip_text{0x1C, 0x1C, 0x1E};   /**< Tooltip text. */
  QColor disabled_text{0x8E, 0x8E, 0x93};  /**< Tertiary label. */
  QColor toolbar_bg{0xF2, 0xF2, 0xF7};     /**< Toolbar background. */
  QColor hover_bg{0xE5, 0xE5, 0xEA};       /**< Neutral hover fill. */
  QColor dock_title{0xF2, 0xF2, 0xF7};     /**< Dock title bar. */
  QColor status_bar{0xF2, 0xF2, 0xF7};     /**< Status bar. */

  /** Simulated iOS material for platforms without native vibrancy. */
  QColor glass_fill{255, 255, 255, 224};      /**< Primary material fill. */
  QColor glass_fill_deep{255, 255, 255, 245}; /**< Opaque fallback fill. */
  QColor glass_cool{242, 242, 247, 28};       /**< Neutral material wash. */
  QColor glass_sheen{255, 255, 255, 150};     /**< Top-edge highlight. */
  QColor glass_rim{198, 198, 200, 100};       /**< Material separator. */
  int panel_radius = 16;                      /**< Panel card radius (px). */
  int title_radius = 12;                      /**< Title bar radius (px). */
  int control_radius = 10;                    /**< Control radius (px). */
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
 * @brief QSS stop-list for a vertical frosted shell bar (title / toolbar /
 * footer).
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
 * @brief Opaque shell plate for menu bar / tool bar / status bar QSS.
 * @return @c #RRGGBB from @ref ShellTokens::toolbar_bg (alpha forced).
 */
QString ShellCssChromePlate();

/**
 * @brief Built-in QSS for the application menu bar (Liquid / Light / Dark).
 */
QString ShellMenuBarStyleSheet();

/**
 * @brief Built-in QSS for @c QToolBar#AutovizToolBar (and plain @c QToolBar).
 * @param object_scoped When @c true, selectors use @c #AutovizToolBar only.
 */
QString ShellToolBarStyleSheet(bool object_scoped = true);

/**
 * @brief Built-in QSS for the application status bar.
 */
QString ShellStatusBarStyleSheet();

/**
 * @brief Paint a frosted shell plate (title bars, tool strips).
 *
 * @param painter Active painter.
 * @param rect Target rectangle in painter coordinates.
 * @param radius Corner radius in device-independent pixels.
 */
void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius);

/**
 * @brief Soft accent wash for selected list/tree rows (Liquid / Light / Dark).
 * @param alpha Fill alpha 0–255 (default matches legacy teal pill).
 */
QColor ShellSelectionFill(int alpha = 78);

/**
 * @brief Soft accent wash for hovered list/tree rows.
 */
QColor ShellHoverFill(int alpha = 56);

/**
 * @brief Ink color for selected row labels / monochrome icons when active.
 */
QColor ShellSelectionInk();

}  // namespace glass
}  // namespace autoviz
