/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_config_io.hpp"

namespace autoviz {
namespace channel_graph {

common::ChannelGraphPanelPersistConfig ToPersistConfig(
    const QString& object_name, const ChannelGraphPanelConfig& config) {
  common::ChannelGraphPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.show_services = config.show_services;
  persist.show_channels = config.show_channels;
  persist.auto_refresh = config.auto_refresh;
  persist.neighborhood_mode = config.neighborhood_mode;
  persist.show_edge_labels = config.show_edge_labels;
  persist.quiet_mode = config.quiet_mode;
  persist.hide_leaf_channels = config.hide_leaf_channels;
  persist.hide_dead_end_channels = config.hide_dead_end_channels;
  persist.probe_enabled = config.probe_enabled;
  persist.channel_arrange = static_cast<int>(config.channel_arrange);
  persist.service_arrange = static_cast<int>(config.service_arrange);
  persist.filter = config.filter.toStdString();
  persist.prefix_filter = config.prefix_filter.toStdString();
  return persist;
}

ChannelGraphPanelConfig FromPersistConfig(
    const common::ChannelGraphPanelPersistConfig& persist) {
  ChannelGraphPanelConfig config = DefaultChannelGraphPanelConfig();
  config.show_services = persist.show_services;
  config.show_channels = persist.show_channels;
  config.auto_refresh = persist.auto_refresh;
  config.neighborhood_mode = persist.neighborhood_mode;
  config.show_edge_labels = persist.show_edge_labels;
  config.quiet_mode = persist.quiet_mode;
  config.hide_leaf_channels = persist.hide_leaf_channels;
  config.hide_dead_end_channels = persist.hide_dead_end_channels;
  config.probe_enabled = persist.probe_enabled;
  config.channel_arrange = static_cast<VertexArrangeMode>(persist.channel_arrange);
  config.service_arrange = static_cast<VertexArrangeMode>(persist.service_arrange);
  config.filter = QString::fromStdString(persist.filter);
  config.prefix_filter = QString::fromStdString(persist.prefix_filter);
  return config;
}

}  // namespace channel_graph
}  // namespace autoviz
