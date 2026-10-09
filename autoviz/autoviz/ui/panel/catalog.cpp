/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/catalog.hpp"

#include <iterator>

namespace autoviz {
namespace {

const PanelCatalogEntry kPanelCatalog[] = {
    {"ViewportDock", "Panel3D", "3D View", "3D visualization viewport"},
    {"ImageDock", "PanelImage", "Image", "Visualize camera images and overlays"},
    {"MapDock", "PanelMap", "Map",
     "Visualize GPS tracks, geo fences, and GeoJSON on an interactive map"},
    {"PlotDock", "PanelPlot", "Plot", "Plot numeric fields over time"},
    {"TableDock", "PanelTable", "Table",
     "Browse repeated message fields as a live table"},
    {"PublishDock", "PanelPublish", "Publish",
     "Publish protobuf JSON messages to Autolink channels"},
    {"RecordDock", "PanelRecord", "Record",
     "Play .record / .bag / .mcap (Foxglove + rqt_bag style transport)"},
    {"ChannelsDock", "PanelRawMessages", "Messages",
     "Inspect Autolink channel traffic"},
    {"ChannelBrowserDock", "PanelChannels", "Channels",
     "Browse channels and drag fields to visualization panels"},
    {"ServiceDock", "PanelService", "Service",
     "Call Autolink services with JSON requests and inspect responses"},
    {"", "PanelStack", "Stack",
     "Stack panels vertically (not yet available)"},
    {"", "PanelTab", "Tab", "Tabbed panel container (not yet available)"},
    {"TeleopDock", "PanelTeleop", "Teleop",
     "Publish geometry_msgs/TwistStamped velocity commands (cmd_vel)"},
    {"ChannelGraphDock", "PanelChannelGraph", "Channel Graph",
     "Visualize Autolink node, channel, and service topology"},
    {"TfTreeDock", "PanelTransformTree", "Transforms",
     "Show the transform tree"},
};

}  // namespace

QVector<PanelCatalogEntry> PanelCatalog() {
  return {std::begin(kPanelCatalog), std::end(kPanelCatalog)};
}

}  // namespace autoviz
