/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tokens.hpp
 * @brief Flat design tokens for Autoviz chrome and overlays.
 *
 * Appearances: Dark / Light / System (follows desktop color scheme).
 * Opaque fills only — no frosted / Liquid glass materials.
 *
 * @see ApplyAppTheme()
 * @see ViewportHudOverlay
 */

#pragma once

#include <QColor>
#include <QString>

class QPainter;
class QRectF;

namespace autoviz {
namespace theme {

/**
 * @brief Shell appearance preference.
 *
 * @c kLiquid is accepted for legacy prefs and treated as light.
 */
enum class ShellAppearance {
  kSystem = 0, /**< Follow desktop light/dark. */
  kLight = 1,  /**< Force light flat chrome. */
  kDark = 2,   /**< Force dark flat chrome. */
  kLiquid = 3, /**< @deprecated Maps to light. */
};

/**
 * @brief Sets the live shell appearance (triggers token rebuild on next Shell()).
 */
void SetShellAppearance(ShellAppearance appearance);

/**
 * @brief Current appearance preference (may be @c kSystem).
 */
ShellAppearance CurrentShellAppearance();

/**
 * @brief @c true when the resolved shell is dark (System uses desktop scheme).
 */
bool ShellIsDark();

/**
 * @brief Always @c false — Liquid Glass is removed.
 */
inline bool ShellIsLiquid() { return false; }

/**
 * @brief Overrides @ref ShellTokens::accent (and derived soft/pressed/link).
 *
 * Pass an invalid @c QColor to clear and restore defaults.
 */
void SetAccentOverride(const QColor& accent);

/**
 * @brief Current accent override, or an invalid color when unset.
 */
QColor AccentOverride();

/**
 * @struct OverlayTokens
 * @brief Opaque colors for 3D viewport HUD overlays (always dark plate).
 */
struct OverlayTokens {
  QColor fill{0x1C, 0x1C, 0x1E};
  QColor fill_deep{0x14, 0x14, 0x16};
  QColor cool_a{0x2C, 0x2C, 0x2E};
  QColor cool_b{0x3A, 0x3A, 0x3C};
  QColor sheen{0x3A, 0x3A, 0x3C};
  QColor rim{0x48, 0x48, 0x4A};
  QColor accent{0xA8, 0xC9, 0x4A};
  QColor accent_mid{0x8F, 0xB0, 0x3A};
  QColor accent_soft{0x3A, 0x3A, 0x3C};
  QColor title{0xAE, 0xAE, 0xB2};
  QColor value{0xF2, 0xF2, 0xF7};
  QColor value_idle{0x63, 0x63, 0x66};
  QColor unit{0xAE, 0xAE, 0xB2};
  QColor track{0x48, 0x48, 0x4A};
  QColor icon{0xE0, 0xE0, 0xE5};
  QColor icon_hover{0xF2, 0xF2, 0xF7};
  QColor icon_on{0xA8, 0xC9, 0x4A};
  QColor divider{0x48, 0x48, 0x4A};
  QColor button_hover_bg{0x3A, 0x3A, 0x3C};
  QColor button_hover_border{0x48, 0x48, 0x4A};
  QColor button_on_bg{0x3A, 0x3A, 0x3C};
  QColor button_on_border{0x3A, 0x3A, 0x3C};
  int radius = 10;
  int toolbar_radius = 12;
};

/**
 * @struct ShellTokens
 * @brief Opaque flat colors for application chrome.
 */
struct ShellTokens {
  QColor window{0xF2, 0xF2, 0xF7};
  QColor window_text{0x1C, 0x1C, 0x1E};
  QColor base{0xFF, 0xFF, 0xFF};
  QColor alternate_base{0xF2, 0xF2, 0xF7};
  QColor card{0xFF, 0xFF, 0xFF};
  QColor text{0x1C, 0x1C, 0x1E};
  QColor secondary_text{0x8E, 0x8E, 0x93};
  QColor button{0xE5, 0xE5, 0xEA};
  QColor button_text{0x1C, 0x1C, 0x1E};
  QColor accent{0xA8, 0xC9, 0x4A};
  QColor accent_pressed{0x8F, 0xB0, 0x3A};
  QColor accent_soft{0xE2, 0xE2, 0xE6};
  QColor selection_bg{0xE2, 0xE2, 0xE6};
  QColor selection_text{0x5B, 0x7A, 0x1E};
  QColor link{0x5B, 0x7A, 0x1E};
  QColor border{0xC6, 0xC6, 0xC8};
  QColor border_light{0xE5, 0xE5, 0xEA};
  QColor tooltip_base{0xFF, 0xFF, 0xFF};
  QColor tooltip_text{0x1C, 0x1C, 0x1E};
  QColor disabled_text{0x8E, 0x8E, 0x93};
  QColor toolbar_bg{0xF2, 0xF2, 0xF7};
  QColor hover_bg{0xE8, 0xE8, 0xEC};
  QColor dock_title{0xF2, 0xF2, 0xF7};
  QColor status_bar{0xF2, 0xF2, 0xF7};

  QColor panel_bg{0xF5, 0xF5, 0xF7};
  QColor panel_surface{0xFF, 0xFF, 0xFF};
  QColor panel_cool{0xF2, 0xF2, 0xF7};
  QColor panel_sheen{0xFF, 0xFF, 0xFF};
  QColor panel_border{0xC6, 0xC6, 0xC8};

  /** @deprecated Prefer @ref panel_bg. */
  QColor glass_fill{0xF5, 0xF5, 0xF7};
  /** @deprecated Prefer @ref panel_surface. */
  QColor glass_fill_deep{0xFF, 0xFF, 0xFF};
  /** @deprecated Prefer @ref panel_cool. */
  QColor glass_cool{0xF2, 0xF2, 0xF7};
  /** @deprecated Prefer @ref panel_sheen. */
  QColor glass_sheen{0xFF, 0xFF, 0xFF};
  /** @deprecated Prefer @ref panel_border. */
  QColor glass_rim{0xC6, 0xC6, 0xC8};

  int panel_radius = 8;
  int title_radius = 6;
  int control_radius = 6;
};

/** @brief Returns overlay tokens (viewport HUD stays dark). */
OverlayTokens Overlay();

/** @brief Returns shell tokens for the resolved appearance + accent. */
ShellTokens Shell();

QString Hex(const QColor& c);
QString Rgba(const QColor& c);

QString ShellBarCss();
QString ShellWindowWashCss();
QString ShellCssBg();
QString ShellCssSurface();
QString ShellCssBorder();
QString ShellCssText();
QString ShellCssMuted();
QString ShellCssAccent();
QString ShellCssChromePlate();

QString ShellMenuBarStyleSheet();
QString ShellToolBarStyleSheet(bool object_scoped = true);
QString ShellStatusBarStyleSheet();

void PaintPanelFill(QPainter& painter, const QRectF& rect, qreal radius);
void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius);

QColor ShellSelectionFill(int alpha = 78);
QColor ShellHoverFill(int alpha = 56);
QColor ShellSelectionInk();

inline void SetShellAccentOverride(const QColor& accent) {
  SetAccentOverride(accent);
}
inline QColor ShellAccentOverride() { return AccentOverride(); }
inline QString ShellGlassBarGradientCss() { return ShellBarCss(); }

}  // namespace theme
}  // namespace autoviz
