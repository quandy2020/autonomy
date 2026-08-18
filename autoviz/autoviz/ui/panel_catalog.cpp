/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_catalog.hpp"

#include <iterator>

namespace autoviz {
namespace {

const PanelCatalogEntry kPanelCatalog[] = {
    {"ViewportDock", "Panel3D", "3D", "3D visualization viewport"},
    {"AudioDock", "PanelAudio", "Audio",
     "Play and visualize foxglove.RawAudio PCM streams"},
    {"PlaybackDock", "PanelDataSource", "Data Source Info",
     "Open or drop Autolink .record files and play them back"},
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
    {"ParametersDock", "PanelParameters", "Parameters",
     "View and edit Autolink runtime parameters"},
    {"PlotDock", "PanelPlot", "Plot", "Plot numeric fields over time"},
    {"PublishDock", "PanelPublish", "Publish",
     "Publish protobuf JSON messages to Autolink channels"},
    {"ChannelsDock", "PanelRawMessages", "Raw Messages",
     "Inspect Autolink channel traffic"},
    {"ChannelBrowserDock", "PanelChannels", "Channels",
     "Browse channels and drag fields to visualization panels"},
    {"ProblemsDock", "PanelProblems", "Problems",
     "Playback and connection diagnostics"},
    {"ServiceDock", "PanelService", "Service Call",
     "Call Autolink services with JSON requests and inspect responses"},
    {"", "PanelStack", "Stack",
     "Stack panels vertically (not yet available)"},
    {"StateTransitionDock", "PanelState", "State Transitions",
     "Visualize discrete state changes over time (modes, enums, booleans)"},
    {"", "PanelTab", "Tab", "Tabbed panel container (not yet available)"},
    {"TableDock", "PanelTable", "Table",
     "Show repeated message arrays as sortable tables"},
    {"TeleopDock", "PanelTeleop", "Teleop",
     "Publish geometry_msgs/Twist velocity commands (cmd_vel)"},
    {"ChannelGraphDock", "PanelChannelGraph", "Channel Graph",
     "Visualize Autolink node, channel, and service topology"},
    {"VariablesDock", "PanelVariables", "Variables",
     "Global variables for message path expressions ($name)"},
    {"VariableSliderDock", "PanelVariableSlider", "Variable Slider",
     "Slider control for numeric global variables"},
    {"TfTreeDock", "PanelTransformTree", "Transform Tree",
     "Show the transform tree"},
};

}  // namespace

QVector<PanelCatalogEntry> PanelCatalog() {
  return {std::begin(kPanelCatalog), std::end(kPanelCatalog)};
}

}  // namespace autoviz
