/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_poi_display.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

StrataPoiDisplay::StrataPoiDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::PoiMarkerArray>(
          "StrataPoi", std::move(channel),
          "autonomy.commsgs.proto.strata_msgs.PoiMarkerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataPoiDisplay::propertySpecs() const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "51;153;255", {}, DisplayPropertyKind::kColor},
          {"selected_color", "Selected Color", "255;153;0", {}, DisplayPropertyKind::kColor}};
}

void StrataPoiDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataPoiDisplay::processMessage(
    const automsgs::msgs::strata_msgs::PoiMarkerArray& message) {
  pois_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  for (const auto& marker : message.markers()) {
    QVector3D local(static_cast<float>(marker.lng_lat().x()),
                    static_cast<float>(marker.lng_lat().y()),
                    static_cast<float>(marker.lng_lat().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    StoredPoi poi;
    poi.position = local;
    poi.selected = marker.selected();
    poi.label = QString::fromStdString(marker.name());
    pois_.push_back(std::move(poi));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataPoiDisplay::onDraw(rendering::SceneOverlay& scene) {
  const QColor normal_color = common::ParseColorProperty(
      propertyValue("color", "51;153;255"), QColor(51, 153, 255));
  const QColor selected_color = common::ParseColorProperty(
      propertyValue("selected_color", "255;153;0"), QColor(255, 153, 0));
  for (const auto& poi : pois_) {
    scene.addPoint(poi.position, poi.selected ? selected_color : normal_color);
  }
}

}  // namespace display
}  // namespace autoviz
