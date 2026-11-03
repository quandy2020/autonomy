/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/normal_display.hpp"

#include <algorithm>
#include <utility>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"

namespace autoviz {
namespace display {
namespace {

PointFieldInfo findFieldByNames(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    std::initializer_list<const char*> names) {
  for (const char* name : names) {
    for (const auto& field : cloud.fields()) {
      if (field.name() == name) {
        return {field.offset(), static_cast<uint8_t>(field.datatype()), true};
      }
    }
  }
  return {};
}

}  // namespace

NormalDisplay::NormalDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2>(
          "Normal", std::move(channel),
          "automsgs.msgs.sensor_msgs.PointCloud2") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> NormalDisplay::propertySpecs()
    const {
  return {{"color", "Color", "0;200;255", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"length", "Normal Length", "0.05"},
          {"decimate", "Decimate", "10"}};
}

void NormalDisplay::clearReceivedData() {
  segments_.clear();
  normals_available_ = false;
}

void NormalDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::PointCloud2& message) {
  segments_.clear();
  normals_available_ = false;
  if (context_ == nullptr) {
    return;
  }

  const PointFieldInfo xf = findFieldByNames(message, {"x"});
  const PointFieldInfo yf = findFieldByNames(message, {"y"});
  const PointFieldInfo zf = findFieldByNames(message, {"z"});
  const PointFieldInfo nxf =
      findFieldByNames(message, {"normal_x", "nx", "normalx"});
  const PointFieldInfo nyf =
      findFieldByNames(message, {"normal_y", "ny", "normaly"});
  const PointFieldInfo nzf =
      findFieldByNames(message, {"normal_z", "nz", "normalz"});
  if (!xf.valid || !yf.valid || !zf.valid || !nxf.valid || !nyf.valid ||
      !nzf.valid) {
    setStatusWarn("normals require xyz+normal fields");
    if (context_->request_redraw) {
      context_->request_redraw();
    }
    return;
  }
  normals_available_ = true;
  setStatusOk();

  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);

  QColor color = common::ParseColorProperty(propertyValue("color", "0;200;255"),
                                            QColor(0, 200, 255));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const float normal_length =
      common::ParseFloatProperty(propertyValue("length", "0.05"), 0.05f);
  const uint32_t decimation = static_cast<uint32_t>(std::max(
      1, common::ParseIntProperty(propertyValue("decimate", "10"), 10)));

  const uint32_t width = message.width();
  const uint32_t height = std::max(1u, message.height());
  const uint32_t point_step = message.point_step();
  const uint32_t count = width * height;
  if (count == 0 || point_step == 0 || message.data().empty()) {
    return;
  }
  const auto* blob = reinterpret_cast<const uint8_t*>(message.data().data());
  const size_t blob_size = message.data().size();

  for (uint32_t i = 0; i < count; i += decimation) {
    const size_t offset = static_cast<size_t>(i) * point_step;
    if (offset + point_step > blob_size) {
      break;
    }
    const uint8_t* point_ptr = blob + offset;
    const QVector3D start(
        valueFromPointData<float>(point_ptr, xf),
        valueFromPointData<float>(point_ptr, yf),
        valueFromPointData<float>(point_ptr, zf));
    const QVector3D normal(
        valueFromPointData<float>(point_ptr, nxf),
        valueFromPointData<float>(point_ptr, nyf),
        valueFromPointData<float>(point_ptr, nzf));
    const float n_len = normal.length();
    if (n_len < 1e-6f) {
      continue;
    }
    const QVector3D end = start + (normal / n_len) * normal_length;
    segments_.push_back(
        {viz::TransformLocalPoint(tf, have_tf, start),
         viz::TransformLocalPoint(tf, have_tf, end), color});
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void NormalDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!normals_available_) {
    return;
  }
  std::vector<LineSegment3D> lines;
  lines.reserve(segments_.size());
  for (const LineVisual& seg : segments_) {
    lines.push_back({seg.start, seg.end, seg.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/normals", lines);
}

}  // namespace display
}  // namespace autoviz
