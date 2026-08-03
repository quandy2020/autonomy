/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_road_graph_display.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

StrataRoadGraphDisplay::StrataRoadGraphDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::RoadGraph>(
          "StrataRoadGraph", std::move(channel),
          "automsgs.msgs.strata_msgs.RoadGraph") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataRoadGraphDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"node_color", "Node Color", "51;230;77", {}, DisplayPropertyKind::kColor},
          {"edge_color", "Edge Color", "51;204;102", {}, DisplayPropertyKind::kColor},
          {"line_width", "Line Width", "0.04"}};
}

void StrataRoadGraphDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRoadGraphDisplay::processMessage(
    const automsgs::msgs::strata_msgs::RoadGraph& message) {
  nodes_.clear();
  edges_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  std::unordered_map<std::string, QVector3D> node_positions;
  node_positions.reserve(static_cast<size_t>(message.nodes_size()));
  for (const auto& node : message.nodes()) {
    QVector3D local(static_cast<float>(node.coordinates().x()),
                    static_cast<float>(node.coordinates().y()),
                    static_cast<float>(node.coordinates().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    node_positions[node.id()] = local;
    nodes_.push_back(local);
  }

  for (const auto& edge : message.edges()) {
    const auto from_it = node_positions.find(edge.from());
    const auto to_it = node_positions.find(edge.to());
    if (from_it == node_positions.end() || to_it == node_positions.end()) {
      continue;
    }
    edges_.push_back(StoredEdge{from_it->second, to_it->second});
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRoadGraphDisplay::onDraw(rendering::SceneOverlay& scene) {
  node_color_ = common::ParseColorProperty(propertyValue("node_color", "51;230;77"),
                                           QColor(51, 230, 77));
  edge_color_ = common::ParseColorProperty(propertyValue("edge_color", "51;204;102"),
                                           QColor(51, 204, 102));
  const float line_width =
      common::ParseFloatProperty(propertyValue("line_width", "0.04"), 0.04f);

  for (const auto& node : nodes_) {
    scene.addPoint(node, node_color_);
  }
  for (const auto& edge : edges_) {
    if (line_width > 0.f) {
      drawBillboardStripOgreOrGl(context_, scene, name(), {edge.from, edge.to},
                                 edge_color_, line_width);
    } else {
      scene.addLine(edge.from, edge.to, edge_color_);
    }
  }
}

}  // namespace display
}  // namespace autoviz
