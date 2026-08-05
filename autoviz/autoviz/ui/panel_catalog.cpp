/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_catalog.hpp"

#include <iterator>

namespace autoviz {
namespace {

const PanelCatalogEntry kPanelCatalog[] = {
    {"ViewportDock", "Panel3D", "3D", "3D visualization viewport"},
    {"", "PanelAudio", "Audio", "Play audio streams (not yet available)"},
    {"PlaybackDock", "PanelDataSource", "Data Source Info",
     "Replay Autolink .record files and seek playback"},
    {"GaugeDock", "PanelGauge", "Gauge",
     "Display scalar channel fields on an analog gauge"},
    {"ImageDock", "PanelImage", "Image", "Visualize camera images and overlays"},
    {"IndicatorDock", "PanelIndicator", "Indicator",
     "Map scalar channel fields to colored status labels using ordered rules"},
    {"LogDock", "PanelLog", "Log", "View foxglove.Log and captured glog output"},
    {"MapDock", "PanelMap", "Map",
     "Visualize GPS tracks, geo fences, and GeoJSON on an interactive map"},
    {"HelpDock", "PanelMarkdown", "Markdown",
     "Static help and documentation panel"},
    {"", "PanelParameters", "Parameters",
     "View and edit runtime parameters (not yet available)"},
    {"PlotDock", "PanelPlot", "Plot", "Plot numeric fields over time"},
    {"PublishDock", "PanelPublish", "Publish",
     "Publish protobuf JSON messages to Autolink channels"},
    {"ChannelsDock", "PanelRawMessages", "Raw Messages",
     "Inspect Autolink channel traffic"},
    {"TopicsDock", "PanelChannelGraph", "Topics",
     "Browse channels and drag fields to Plot, Table, and Gauge panels"},
    {"ProblemsDock", "PanelLog", "Problems",
     "Playback and connection diagnostics"},
    {"", "PanelService", "Service Call",
     "Call services from the UI (not yet available)"},
    {"", "PanelStack", "Stack",
     "Stack panels vertically (not yet available)"},
    {"", "PanelState", "State Transitions",
     "Visualize state machines (not yet available)"},
    {"", "PanelTab", "Tab", "Tabbed panel container (not yet available)"},
    {"TableDock", "PanelTable", "Table",
     "Show repeated message arrays as sortable tables"},
    {"TeleopDock", "PanelTeleop", "Teleop",
     "Publish geometry_msgs/Twist velocity commands (cmd_vel)"},
    {"", "PanelChannelGraph", "Channel Graph",
     "Visualize Autolink channel connections (not yet available)"},
    {"TfTreeDock", "PanelTransformTree", "Transform Tree",
     "Show the transform tree"},
    {"", "PanelParameters", "Variables",
     "Global variables for message path expressions (planned)"},
};

}  // namespace

QVector<PanelCatalogEntry> PanelCatalog() {
  return {std::begin(kPanelCatalog), std::end(kPanelCatalog)};
}

}  // namespace autoviz
