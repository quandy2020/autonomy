/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/marker_draw_utils.hpp"

#include <algorithm>

#include <QColor>
#include <QQuaternion>
#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/arrow_mesh_utils.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_mesh_draw.hpp"
#include "autoviz/display/ogre_entity_draw.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/primitive_mesh.hpp"
#include "autoviz/display/transform_utils.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr int32_t kDelete = 2;
constexpr int32_t kDeleteAll = 3;
constexpr int32_t kArrow = 0;
constexpr int32_t kCube = 1;
constexpr int32_t kSphere = 2;
constexpr int32_t kCylinder = 3;
constexpr int32_t kLineStrip = 4;
constexpr int32_t kLineList = 5;
constexpr int32_t kPoints = 8;
constexpr int32_t kCubeList = 6;
constexpr int32_t kSphereList = 7;
constexpr int32_t kTextViewFacing = 9;
constexpr int32_t kMeshResource = 10;
constexpr int32_t kTriangleList = 11;

const ObjMesh& UnitSphereMesh() {
  static const ObjMesh mesh = buildSphereMesh(0.5f);
  return mesh;
}

const ObjMesh& UnitCylinderMesh() {
  static const ObjMesh mesh = buildCylinderMesh(0.5f, 1.f);
  return mesh;
}

void DrawScaledPrimitive(rendering::SceneOverlay& scene, int32_t type,
                         const QMatrix4x4& transform, const QColor& color,
                         bool solid) {
  if (type == kSphere) {
    if (solid) {
      scene.addTriangleMeshSolid(UnitSphereMesh(), transform, color);
    } else {
      scene.addTriangleMeshWireframe(UnitSphereMesh(), transform, color);
    }
    return;
  }
  if (type == kCylinder) {
    if (solid) {
      scene.addTriangleMeshSolid(UnitCylinderMesh(), transform, color);
    } else {
      scene.addTriangleMeshWireframe(UnitCylinderMesh(), transform, color);
    }
    return;
  }
  if (type == kCube) {
    if (solid) {
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), QVector3D(0.5f, 0.5f, 0.5f),
                        transform, color);
    } else {
      scene.addBoxWireframe(QVector3D(0.f, 0.f, 0.f), QVector3D(0.5f, 0.5f, 0.5f),
                            transform, color);
    }
  }
}

void DrawMarkerList(rendering::SceneOverlay& scene,
                    const std::vector<QVector3D>& points, int32_t list_type,
                    const automsgs::msgs::visualization_msgs::Marker&
                        marker,
                    const QColor& color, bool solid) {
  const float sx = std::max(0.01f, static_cast<float>(marker.scale().x()));
  const float sy = std::max(0.01f, static_cast<float>(marker.scale().y()));
  const float sz = std::max(0.01f, static_cast<float>(marker.scale().z()));
  for (const auto& point : points) {
    QMatrix4x4 local;
    local.setToIdentity();
    local.translate(point);
    local.scale(sx, sy, sz);
    DrawScaledPrimitive(scene, list_type == kCubeList ? kCube : kSphere, local,
                        color, solid);
  }
}

QColor MarkerColor(
    const automsgs::msgs::visualization_msgs::Marker& marker) {
  const auto& c = marker.color();
  return QColor(static_cast<int>(c.r() * 255.f),
                static_cast<int>(c.g() * 255.f),
                static_cast<int>(c.b() * 255.f),
                static_cast<int>(c.a() * 255.f));
}

QMatrix4x4 MarkerTransform(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    common::DisplayContext* context) {
  return markerTransformInFixedFrame(marker, context);
}

bool MarkerPointsInFixedFrame(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    common::DisplayContext* context, std::vector<QVector3D>* points) {
  if (points == nullptr) {
    return false;
  }
  points->clear();
  points->reserve(static_cast<std::size_t>(marker.points_size()));
  const auto zero_time = autoviz::commsgs::ZeroTime();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (context != nullptr && !marker.header().frame_id().empty()) {
    try {
      tf = context->tf_buffer->lookupTransform(context->fixed_frame,
                                               marker.header().frame_id(),
                                               zero_time);
      have_tf = true;
    } catch (...) {
    }
  }
  for (const auto& point : marker.points()) {
    QVector3D p(static_cast<float>(point.x()), static_cast<float>(point.y()),
                static_cast<float>(point.z()));
    if (have_tf) {
      p = transformPoint(tf, p);
    }
    points->push_back(p);
  }
  return !points->empty();
}

std::string propertyValue(const common::DisplayPropertyMap& properties,
                          const std::string& key,
                          const std::string& default_value) {
  const auto it = properties.find(key);
  return it != properties.end() ? it->second : default_value;
}

QColor ResolveMarkerDrawColor(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    const common::DisplayPropertyMap& properties) {
  const QColor marker_color = MarkerColor(marker);
  const bool use_custom_color =
      common::ParseBoolProperty(
          propertyValue(properties, "use_custom_color", "false"), false);
  if (!use_custom_color) {
    return marker_color;
  }
  QColor custom_color = common::ParseColorProperty(
      propertyValue(properties, "color", "200;200;200"), marker_color);
  custom_color.setAlpha(marker_color.alpha());
  return custom_color;
}

common::PickHandle registerMarkerPick(
    rendering::SceneOverlay& scene, const std::string& display_prefix,
    const automsgs::msgs::visualization_msgs::Marker& marker,
    const QVector3D& position) {
  auto handler =
      common::CreateSelectionHandler<common::MarkerSelectionHandler>();
  handler->setDisplayInfo(display_prefix, "Marker");
  handler->setMarkerInfo(marker.ns(), marker.id());
  return scene.registerPickEntry(position, -1, handler);
}

}  // namespace

QMatrix4x4 markerTransformInFixedFrame(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    common::DisplayContext* context) {
  QMatrix4x4 local;
  local.setToIdentity();
  const auto& pose = marker.pose();
  local.translate(static_cast<float>(pose.position().x()),
                  static_cast<float>(pose.position().y()),
                  static_cast<float>(pose.position().z()));
  local.rotate(QQuaternion(
      static_cast<float>(pose.orientation().w()),
      static_cast<float>(pose.orientation().x()),
      static_cast<float>(pose.orientation().y()),
      static_cast<float>(pose.orientation().z())));
  local.scale(std::max(0.01f, static_cast<float>(marker.scale().x())),
              std::max(0.01f, static_cast<float>(marker.scale().y())),
              std::max(0.01f, static_cast<float>(marker.scale().z())));

  if (context == nullptr || marker.header().frame_id().empty()) {
    return local;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  try {
    const auto tf = context->tf_buffer->lookupTransform(
        context->fixed_frame, marker.header().frame_id(), zero_time);
    return transformToMatrix(tf) * local;
  } catch (...) {
    return local;
  }
}

void upsertMarker(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    std::map<MarkerKey, StoredMarker>* markers) {
  if (markers == nullptr) {
    return;
  }
  if (marker.action() == kDeleteAll) {
    if (marker.ns().empty()) {
      markers->clear();
    } else {
      for (auto it = markers->begin(); it != markers->end();) {
        if (it->first.ns == marker.ns()) {
          it = markers->erase(it);
        } else {
          ++it;
        }
      }
    }
    return;
  }
  if (marker.action() == kDelete) {
    markers->erase(MarkerKey{marker.ns(), marker.id()});
    return;
  }
  StoredMarker stored;
  stored.marker = marker;
  stored.has_mesh = parseMarkerMesh(marker, &stored.mesh);
  (*markers)[MarkerKey{marker.ns(), marker.id()}] = std::move(stored);
}

void drawStoredMarkers(
    rendering::SceneOverlay& scene, common::DisplayContext* context,
    const common::DisplayPropertyMap& properties,
    const std::map<MarkerKey, StoredMarker>& markers,
    const std::string& display_prefix) {
  const float point_size =
      common::ParseFloatProperty(propertyValue(properties, "point_size", "4"), 4.f);
  const std::string mesh_style = propertyValue(properties, "mesh_style", "solid");

  std::vector<LineSegment3D> ogre_lines;
  std::vector<ColoredPoint3D> ogre_points;
  std::vector<ColoredMeshInstance> ogre_meshes;
  std::vector<TextLabelInstance> ogre_labels;
#ifdef AUTOVIZ_USE_OGRE
  const bool use_ogre = context != nullptr && context->ogre_scene_host != nullptr;
#else
  const bool use_ogre = false;
#endif

  auto appendArrowLines = [&ogre_lines](const QVector3D& start, const QVector3D& end,
                                        const QColor& color) {
    const QVector3D delta = end - start;
    const float length = delta.length();
    if (length < 1e-4f) {
      return;
    }
    const QVector3D direction = delta / length;
    QVector3D side = QVector3D::crossProduct(direction, QVector3D(0.f, 0.f, 1.f));
    if (side.lengthSquared() < 1e-6f) {
      side = QVector3D::crossProduct(direction, QVector3D(0.f, 1.f, 0.f));
    }
    side.normalize();
    const float head = length * 0.2f;
    ogre_lines.push_back({start, end, color});
    ogre_lines.push_back(
        {end, end - direction * head + side * head * 0.35f, color});
    ogre_lines.push_back(
        {end, end - direction * head - side * head * 0.35f, color});
  };

  for (const auto& [key, stored] : markers) {
    (void)key;
    const auto& marker = stored.marker;
    const QColor color = ResolveMarkerDrawColor(marker, properties);
    const QMatrix4x4 transform = MarkerTransform(marker, context);
    const bool solid = mesh_style != "wireframe";

    if (marker.type() == kSphere || marker.type() == kCube ||
        marker.type() == kCylinder) {
      DrawScaledPrimitive(scene, marker.type(), transform, color, solid);
    } else if (marker.type() == kArrow) {
      const QVector3D start = transform.map(QVector3D(0.f, 0.f, 0.f));
      const QVector3D end = transform.map(QVector3D(1.f, 0.f, 0.f));
      const float shaft_d =
          std::max(0.01f, static_cast<float>(marker.scale().y()));
      const float head_d =
          std::max(0.01f, static_cast<float>(marker.scale().z()));
      if (use_ogre && solid) {
        const std::size_t mesh_start = ogre_meshes.size();
        appendSolidArrowMeshes(&ogre_meshes, start, end, color, 0.2f, shaft_d,
                               head_d);
        const common::PickHandle pick =
            registerMarkerPick(scene, display_prefix, marker, start);
        for (std::size_t i = mesh_start; i < ogre_meshes.size(); ++i) {
          ogre_meshes[i].pick_handle = pick;
        }
      } else if (use_ogre) {
        appendArrowLines(start, end, color);
      } else {
        scene.addLine(start, end, color);
        scene.addLine(end, transform.map(QVector3D(0.8f, 0.1f, 0.f)), color);
        scene.addLine(end, transform.map(QVector3D(0.8f, -0.1f, 0.f)), color);
      }
    } else if (marker.type() == kLineStrip && marker.points_size() >= 2) {
      std::vector<QVector3D> points;
      if (MarkerPointsInFixedFrame(marker, context, &points)) {
        for (std::size_t i = 1; i < points.size(); ++i) {
          if (use_ogre) {
            ogre_lines.push_back({points[i - 1], points[i], color});
          } else {
            scene.addLine(points[i - 1], points[i], color);
          }
        }
      }
    } else if (marker.type() == kLineList && marker.points_size() >= 2) {
      std::vector<QVector3D> points;
      if (MarkerPointsInFixedFrame(marker, context, &points)) {
        for (std::size_t i = 1; i < points.size(); i += 2) {
          if (use_ogre) {
            ogre_lines.push_back({points[i - 1], points[i], color});
          } else {
            scene.addLine(points[i - 1], points[i], color);
          }
        }
      }
    } else if (marker.type() == kPoints && marker.points_size() >= 1) {
      std::vector<QVector3D> points;
      if (MarkerPointsInFixedFrame(marker, context, &points)) {
        if (use_ogre) {
          for (const auto& point : points) {
            ogre_points.push_back({point, color});
          }
        } else {
          scene.addPoints(points, color);
        }
      }
    } else if ((marker.type() == kCubeList || marker.type() == kSphereList) &&
               marker.points_size() >= 1) {
      std::vector<QVector3D> points;
      if (MarkerPointsInFixedFrame(marker, context, &points)) {
        DrawMarkerList(scene, points, marker.type(), marker, color, solid);
      }
    } else if (marker.type() == kTextViewFacing) {
      const QVector3D center = transform.map(QVector3D(0.f, 0.f, 0.f));
      const float text_height =
          marker.scale().z() > 0.f ? static_cast<float>(marker.scale().z())
                                   : 0.2f;
      const QString label =
          QString::fromStdString(marker.text()).trimmed();
      if (label.isEmpty()) {
        const float half_extent = std::max(0.04f, text_height * 0.5f);
        scene.addViewFacingQuad(center, half_extent, color);
      } else if (use_ogre) {
        TextLabelInstance text_label;
        text_label.text = label.toStdString();
        text_label.position = center;
        text_label.color = color;
        text_label.char_height = text_height;
        if (marker.scale().x() > 0.f) {
          text_label.space_width = static_cast<float>(marker.scale().x());
        }
        ogre_labels.push_back(std::move(text_label));
      } else {
        const int pixel_height =
            static_cast<int>(std::clamp(text_height * 120.f, 16.f, 128.f));
        const QImage label_image =
            rendering::RasterizeTextLabel(label, color, pixel_height);
        if (label_image.isNull()) {
          continue;
        }
        const float aspect =
            static_cast<float>(label_image.width()) /
            static_cast<float>(std::max(1, label_image.height()));
        const float half_height = text_height * 0.5f;
        const float half_width = half_height * aspect;
        scene.addViewFacingTexturedQuad(center, half_width, half_height,
                                        label_image);
      }
    } else if ((marker.type() == kMeshResource || marker.type() == kTriangleList) &&
               stored.has_mesh) {
      const bool wireframe = mesh_style == "wireframe";
      if (use_ogre) {
        ogre_meshes.push_back(
            {stored.mesh, transform, color, wireframe,
             registerMarkerPick(scene, display_prefix, marker,
                                transform.map(QVector3D(0.f, 0.f, 0.f)))});
      } else if (wireframe) {
        scene.addTriangleMeshWireframe(stored.mesh, transform, color);
      } else {
        scene.addTriangleMeshSolid(stored.mesh, transform, color);
      }
    }
  }

  if (use_ogre) {
    drawLineSegmentsOgreOrGl(context, scene, display_prefix + "/lines", ogre_lines);
    drawEntityMeshesOgreOrGl(context, scene, display_prefix + "/meshes", ogre_meshes);
    if (!ogre_points.empty()) {
      drawColoredPointsOgreOrGl(context, scene, display_prefix + "/points",
                                "MarkerPoints", point_size,
                                rendering::PointCloudStyle::kSquares, ogre_points,
                                false);
    }
    drawLabelsOgreOrGl(context, scene, display_prefix + "/labels", ogre_labels);
  }
}

}  // namespace display
}  // namespace autoviz
