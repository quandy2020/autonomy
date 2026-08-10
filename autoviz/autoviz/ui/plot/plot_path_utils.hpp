/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QStringList>

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace plot {

/** Foxglove-style `/channel` or `/channel.field.path`. */
QString CombinedPlotValuePath(const QString& channel, const QString& field_path);

/** Split a combined plot path using known channel names (longest match first). */
bool SplitPlotValuePath(const QString& combined, const QStringList& known_channels,
                        QString* channel, QString* field_path);

/** True when a numeric leaf field is selected (channel-only paths are invalid). */
bool IsPlottablePlotValuePath(const QString& channel, const QString& field_path);

/** All browsable plot paths across discovered channels. */
QStringList AllPlotBrowsePaths(common::VisualizationManager* manager);

/** Browsable paths for one channel (includes `/channel` and nested fields). */
QStringList PlotBrowsePathsForChannel(const QString& channel,
                                      const std::string& message_type);

/** Contextual suggestions for the plot Value path editor (Foxglove-style `.` drilling). */
QStringList PlotValuePathSuggestions(common::VisualizationManager* manager,
                                     const QString& typed_prefix);

/** Message type string for a discovered channel, or empty if unknown. */
std::string MessageTypeForChannel(common::VisualizationManager* manager,
                                  const QString& channel);

/** Discovered channels plus built-in tutorial defaults. */
QStringList AllKnownChannels(common::VisualizationManager* manager);

}  // namespace plot
}  // namespace autoviz
