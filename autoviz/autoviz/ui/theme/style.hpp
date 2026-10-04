/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file style.hpp
 * @brief Tokenized stylesheet loader and tiny non-selector paint atoms.
 *
 * Stylesheets live as resources keyed by short paths
 * (@c "theme", @c "chrome/dock", @c "widget/primary_button", …).
 * @ref tokens() supplies @c {{placeholder}} substitutions for Shell vs Frost
 * tones.
 *
 * @see ApplyAppTheme()
 * @see BuildAppThemeStylesheet()
 * @see glass::ShellTokens
 */

#pragma once

#include <QHash>
#include <QString>

namespace autoviz {
namespace style {

/**
 * @enum Tone
 * @brief Token set: Shell = glass app chrome; Frost = cyan frosted panels.
 */
enum class Tone {
  Shell, /**< Cool light application chrome tokens. */
  Frost, /**< Cyan frosted panel / overlay tokens. */
};

/**
 * @enum Role
 * @brief Label color roles for @ref type().
 */
enum class Role {
  Muted,      /**< Secondary chrome text. */
  Body,       /**< Primary body text. */
  Accent,     /**< Accent-colored text. */
  PanelMuted, /**< Muted text on frosted panels. */
  PanelBody,  /**< Body text on frosted panels. */
  Ok,         /**< Success / OK status. */
  Danger,     /**< Error / destructive status. */
};

/**
 * @enum Mark
 * @brief Tiny non-selector paint atoms for @ref mark().
 */
enum class Mark {
  Clear,   /**< Transparent / clear fill. */
  Rule,    /**< Hairline rule. */
  Invalid, /**< Invalid-state marker. */
  Swatch,  /**< Color swatch (arg = CSS color). */
  Preview, /**< Preview chip. */
  Fill,    /**< Solid fill (arg = @c "r,g,b"). */
  VLine,   /**< Vertical divider line. */
};

/**
 * @brief Placeholder token map for the given @p tone.
 * @param tone Shell or Frost token set.
 * @return Map of placeholder name → CSS value.
 */
QHash<QString, QString> tokens(Tone tone = Tone::Shell);

/**
 * @brief Load a stylesheet by short path.
 *
 * Path examples:
 * - @c "theme" | @c "menu" | @c "chrome" | @c "widget" | @c "panel"
 * - @c "chrome/dock" | @c "widget/primary_button"
 * - @c "teleop" | @c "teleop/header"
 *
 * @param path Short resource path without extension.
 * @return Expanded QSS string (Shell tokens by default).
 */
QString sheet(const QString& path);

/**
 * @brief Load a stylesheet with an explicit @p tone token set.
 * @param path Short resource path.
 * @param tone Token set for placeholders.
 * @return Expanded QSS string.
 */
QString sheet(const QString& path, Tone tone);

/**
 * @brief Load a stylesheet with extra placeholder overrides.
 * @param path Short resource path.
 * @param extra Additional / overriding placeholders.
 * @return Expanded QSS string.
 */
QString sheet(const QString& path, const QHash<QString, QString>& extra);

/**
 * @brief Builds inline label QSS from a @ref Role.
 *
 * @param role Color role.
 * @param size_px Font size in pixels.
 * @param weight Font weight (QFont weight int).
 * @param italic Italic face when @c true.
 * @param extra Extra QSS declarations appended.
 * @return Inline stylesheet fragment.
 */
QString type(Role role, int size_px = 12, int weight = 400, bool italic = false,
             const QString& extra = QString());

/**
 * @brief Builds a tiny paint-atom QSS fragment.
 *
 * @param kind Mark kind.
 * @param arg CSS color for @ref Mark::Swatch; @c "r,g,b" for @ref Mark::Fill.
 * @param radius Corner radius used by @ref Mark::Swatch.
 * @return QSS fragment.
 */
QString mark(Mark kind, const QString& arg = QString(), int radius = 2);

}  // namespace style
}  // namespace autoviz
