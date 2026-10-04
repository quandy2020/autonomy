/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_path_utils.hpp
 * @brief Foxglove-style combined plot value paths and suggestion helpers.
 *
 * Combines @c /channel and field paths for the Value editor, splits them using
 * discovered channel names, and builds browse / suggestion lists via
 * @ref common::VisualizationManager.
 *
 * @see PlotSettingsWidget
 * @see PlotValueSuggestList
 * @see message_field_tree.hpp
 */

#pragma once

#include <QString>
#include <QStringList>

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace plot {

/**
 * @brief Foxglove-style @c /channel or @c /channel.field.path.
 *
 * @param channel Channel name (with or without leading slash).
 * @param field_path Optional field path; empty → channel-only path.
 * @return Combined display / edit string.
 */
QString CombinedPlotValuePath(const QString& channel, const QString& field_path);

/**
 * @brief Split a combined plot path using known channel names (longest match first).
 *
 * @param combined Combined path from the Value editor.
 * @param known_channels Discovered (+ default) channel names.
 * @param channel Out: matched channel.
 * @param field_path Out: remainder field path (may be empty).
 * @return @c true when a channel match was found.
 */
bool SplitPlotValuePath(const QString& combined, const QStringList& known_channels,
                        QString* channel, QString* field_path);

/**
 * @brief True when a numeric leaf field is selected (channel-only paths are invalid).
 *
 * @param channel Channel name.
 * @param field_path Field path (must be non-empty and numeric).
 * @return Whether the pair is plottable as a series Y.
 */
bool IsPlottablePlotValuePath(const QString& channel, const QString& field_path);

/**
 * @brief All browsable plot paths across discovered channels.
 *
 * @param manager Visualization manager (may be @c nullptr → empty).
 * @return Combined paths for autocomplete / browse.
 */
QStringList AllPlotBrowsePaths(common::VisualizationManager* manager);

/**
 * @brief Browsable paths for one channel (includes @c /channel and nested fields).
 *
 * @param channel Channel name.
 * @param message_type Schema type for field enumeration.
 * @return Paths for that channel.
 */
QStringList PlotBrowsePathsForChannel(const QString& channel,
                                      const std::string& message_type);

/**
 * @brief Contextual suggestions for the plot Value path editor (Foxglove-style
 *        @c . drilling).
 *
 * @param manager Visualization manager.
 * @param typed_prefix Current editor text.
 * @return Suggestion strings for @ref PlotValueSuggestList.
 */
QStringList PlotValuePathSuggestions(common::VisualizationManager* manager,
                                     const QString& typed_prefix);

/**
 * @brief Message type string for a discovered channel, or empty if unknown.
 *
 * @param manager Visualization manager.
 * @param channel Channel name.
 * @return Message type, or empty string.
 */
std::string MessageTypeForChannel(common::VisualizationManager* manager,
                                  const QString& channel);

/**
 * @brief Discovered channels plus built-in tutorial defaults.
 *
 * @param manager Visualization manager (may be @c nullptr).
 * @return Channel name list.
 */
QStringList AllKnownChannels(common::VisualizationManager* manager);

}  // namespace plot
}  // namespace autoviz
