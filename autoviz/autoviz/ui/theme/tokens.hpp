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
  QColor accent{0x0E, 0x93, 0x84};
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
  QColor window{0xED, 0xF0, 0xF5};
  QColor window_text{0x10, 0x18, 0x28};
  QColor base{0xFF, 0xFF, 0xFF};
  QColor alternate_base{0xF4, 0xF6, 0xFA};
  QColor card{0xFF, 0xFF, 0xFF};
  QColor text{0x10, 0x18, 0x28};
  QColor secondary_text{0x66, 0x70, 0x85};
  QColor button{0xEE, 0xF1, 0xF6};
  QColor button_text{0x10, 0x18, 0x28};
  QColor accent{0xA8, 0xC9, 0x4A};
  QColor accent_pressed{0x0B, 0x7A, 0x6E};
  QColor accent_soft{0xD7, 0xF0, 0xEC};
  QColor selection_bg{0xD7, 0xF0, 0xEC};
  QColor selection_text{0x0B, 0x5F, 0x57};
  QColor link{0x0E, 0x7C, 0x72};
  QColor border{0xD5, 0xDB, 0xE5};
  QColor border_light{0xE7, 0xEB, 0xF2};
  QColor tooltip_base{0xFF, 0xFF, 0xFF};
  QColor tooltip_text{0x10, 0x18, 0x28};
  QColor disabled_text{0x98, 0xA2, 0xB3};

  /** Destructive / error ink for outlined buttons and labels. */
  QColor danger{0xDC, 0x26, 0x26};
  QColor danger_hover{0xB9, 0x1C, 0x1C};
  QColor danger_pressed{0x99, 0x1B, 0x1B};
  QColor danger_soft{0xFE, 0xE2, 0xE2};
  QColor danger_border{0xFC, 0xA5, 0xA5};
  /** Solid destructive fills (E-stop style actions). */
  QColor danger_filled{0xE1, 0x1D, 0x48};
  QColor danger_filled_hover{0xBE, 0x12, 0x3C};
  QColor danger_filled_pressed{0x9F, 0x12, 0x39};
  /** Caution / static-transform accent. */
  QColor warning{0xD9, 0x77, 0x06};
  /** Disabled text on an accent-filled surface. */
  QColor on_accent_muted{0xFF, 0xFF, 0xFF, 0x8C};
  QColor toolbar_bg{0xF4, 0xF6, 0xFA};
  QColor hover_bg{0xE9, 0xEE, 0xF5};
  QColor dock_title{0xF1, 0xF4, 0xF9};
  QColor status_bar{0xF4, 0xF6, 0xFA};

  QColor panel_bg{0xF4, 0xF6, 0xFA};
  QColor panel_surface{0xFF, 0xFF, 0xFF};
  QColor panel_cool{0xEE, 0xF2, 0xF8};
  QColor panel_sheen{0xFF, 0xFF, 0xFF};
  QColor panel_border{0xDF, 0xE5, 0xEE};

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

  int panel_radius = 14;
  int title_radius = 10;
  int control_radius = 10;
  // Qt clamps @c border-radius above half the widget height and then falls
  // back to a square corner (verified on the button styles). Keep these values
  // at or below ~16px for the 32px control height.
  int radius_xs = 4;   /**< Swatches, inline tags, hairlines. */
  int radius_sm = 8;   /**< Chips, icon buttons, small surfaces. */
  int radius_md = 12;  /**< Cards, badges, segmented containers. */

  // ---------------------------------------------------------------------
  // Layout metrics.
  //
  // Widgets and QSS must read these instead of embedding pixel literals;
  // adding a new size means adding a token here, not a magic number at the
  // call site. QSS reaches the same numbers through style::tokens()
  // ({{h-field}}, {{h-btn}}, {{r-btn}}, {{font-*}}, ...).
  // ---------------------------------------------------------------------
  int dock_title_height = 38;    /**< Custom panel header height. */
  int dock_title_tool = 28;      /**< Title-bar tool / close button size. */
  int title_icon = 20;           /**< Title icon column width. */
  int title_slack = 48;          /**< Elided-title minimum width budget. */
  int field_height = 30;         /**< Settings field height. */
  int field_height_compact = 26; /**< Native control min-height. */
  int field_padding_v = 3;       /**< Field inner vertical padding. */
  int field_padding_h = 10;      /**< Field inner horizontal padding. */
  int button_height = 32;        /**< Primary / ghost / danger button height. */
  int button_radius = 12;        /**< Qt clamps radii above height / 2. */
  int header_padding_v = 7;      /**< Table header vertical padding. */
  int header_padding_h = 8;      /**< Table header horizontal padding. */
  int content_margin = 12;       /**< Panel content inset. */
  int content_spacing = 6;       /**< Between stacked panel rows. */
  int section_spacing = 8;       /**< Between settings sections. */
  int font_small = 11;           /**< Labels, hints, chips. */
  int font_body = 13;            /**< Default body text. */
  int font_title = 14;           /**< Panel / dock titles. */
  int font_section = 13;         /**< Group / section headers. */
  int gutter = 7;                /**< Splitter gap between mosaic cards. */

  // Spacing scale (px). QSS reaches these as {{space-1}} .. {{space-8}};
  // C++ widgets use the tokens so paddings never drift apart.
  int space_1 = 2;   /**< Hairline gap. */
  int space_2 = 4;   /**< Tight cluster. */
  int space_3 = 6;   /**< Row gap. */
  int space_4 = 8;   /**< Default gutter. */
  int space_5 = 10;  /**< Field inner padding. */
  int space_6 = 12;  /**< Card padding. */
  int space_7 = 14;  /**< Wide padding. */
  int space_8 = 16;  /**< Section padding. */
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
