/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Combines:
 *  - grid_map_rviz_plugin elevation mesh
 *  - grid_map_visualization converters (point_cloud / flat_point_cloud /
 *    occupancy_grid / grid_cells / map_region / vectors)
 *****************************************************************************/

#include "autoviz/display/grid_map_display.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/grid_map_color_maps.hpp"
#include "autoviz/display/transform_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {
namespace {

QColor WithAlpha(QColor color, float alpha) {
  color.setAlphaF(std::clamp(alpha, 0.f, 1.f));
  return color;
}

QColor ColorFromPackedFloat(float value) {
  uint32_t bits = 0;
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  return QColor(static_cast<int>((bits >> 16) & 0xff),
                 static_cast<int>((bits >> 8) & 0xff),
                 static_cast<int>(bits & 0xff));
}

float NormalizeIntensity(float value, float min_v, float max_v) {
  if (!(max_v > min_v)) {
    return 0.f;
  }
  return std::clamp((value - min_v) / (max_v - min_v), 0.f, 1.f);
}

QColor LerpColor(float t, const QColor& min_c, const QColor& max_c) {
  t = std::clamp(t, 0.f, 1.f);
  return QColor::fromRgbF(
      min_c.redF() + t * (max_c.redF() - min_c.redF()),
      min_c.greenF() + t * (max_c.greenF() - min_c.greenF()),
      min_c.blueF() + t * (max_c.blueF() - min_c.blueF()));
}

/** RViz map palette: free=white … occupied=black; unknown teal. */
QColor OccupancyPalette(int32_t value, float alpha) {
  if (value < 0) {
    return WithAlpha(QColor(0x70, 0x89, 0x86), alpha);
  }
  if (value > 100) {
    return WithAlpha(QColor(0, 255, 0), alpha);
  }
  const int shade = 255 - (255 * value) / 100;
  return WithAlpha(QColor(shade, shade, shade), alpha);
}

bool ParseLayerMatrix(const automsgs::msgs::std_msgs::Float32MultiArray& array,
                      int* rows, int* cols, std::vector<float>* out) {
  if (rows == nullptr || cols == nullptr || out == nullptr) {
    return false;
  }
  out->clear();
  if (array.data_size() == 0) {
    return false;
  }

  int n_rows = 0;
  int n_cols = 0;
  bool row_major = true;
  if (array.has_layout() && array.layout().dim_size() >= 2) {
    const auto& d0 = array.layout().dim(0);
    const auto& d1 = array.layout().dim(1);
    const std::string& label0 = d0.label();
    if (label0 == "column_index" || label0 == "column") {
      row_major = false;
      n_cols = static_cast<int>(d0.size());
      n_rows = static_cast<int>(d1.size());
    } else if (label0 == "row_index" || label0 == "row") {
      row_major = true;
      n_rows = static_cast<int>(d0.size());
      n_cols = static_cast<int>(d1.size());
    } else {
      row_major = false;
      n_cols = static_cast<int>(d0.size());
      n_rows = static_cast<int>(d1.size());
    }
  }

  if (n_rows <= 0 || n_cols <= 0) {
    const int n = array.data_size();
    n_cols = static_cast<int>(std::lround(std::sqrt(static_cast<double>(n))));
    if (n_cols <= 0) {
      return false;
    }
    n_rows = n / n_cols;
    if (n_rows * n_cols != n) {
      n_rows = 1;
      n_cols = n;
    }
    row_major = true;
  }

  if (static_cast<int>(array.data_size()) < n_rows * n_cols) {
    return false;
  }

  out->assign(static_cast<size_t>(n_rows * n_cols), 0.f);
  const int offset = array.has_layout()
                         ? static_cast<int>(array.layout().data_offset())
                         : 0;
  for (int r = 0; r < n_rows; ++r) {
    for (int c = 0; c < n_cols; ++c) {
      const int src =
          row_major ? (offset + r * n_cols + c) : (offset + c * n_rows + r);
      if (src < 0 || src >= array.data_size()) {
        return false;
      }
      (*out)[static_cast<size_t>(r * n_cols + c)] = array.data(src);
    }
  }
  *rows = n_rows;
  *cols = n_cols;
  return true;
}

}  // namespace

GridMapDisplay::GridMapDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::map_msgs::GridMap>(
          "GridMap", std::move(channel), "automsgs.msgs.map_msgs.GridMap") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> GridMapDisplay::propertySpecs() const {
  const auto cmaps = GridMapColorMapNames();
  return {
      {"visualization", "Visualization", "mesh",
       {"mesh", "point_cloud", "flat_point_cloud", "occupancy_grid",
        "grid_cells", "map_region", "vectors"}},
      {"alpha", "Alpha", "1.0"},

      // --- mesh (rviz_plugin) ---
      {"height_transformer", "Height Transformer", "Layer", {"Layer", "Flat"},
       common::DisplayPropertyKind::kAuto, "visualization", "mesh"},
      {"height_layer", "Height Layer", "elevation", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|vectors"},
      {"color_transformer", "Color Transformer", "IntensityLayer",
       {"IntensityLayer", "ColorLayer", "FlatColor", "None"},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"color_layer", "Color Layer", "elevation", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"colormap", "ColorMap", "default", cmaps,
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"use_colormap", "Use ColorMap", "true", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"invert_colormap", "Invert ColorMap", "false", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"color", "Color", "200;200;200", {}, common::DisplayPropertyKind::kColor,
       "visualization",
       "mesh|point_cloud|flat_point_cloud|grid_cells|map_region|vectors"},
      {"min_color", "Min Color", "0;0;0", {},
       common::DisplayPropertyKind::kColor, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"max_color", "Max Color", "255;255;255", {},
       common::DisplayPropertyKind::kColor, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"autocompute_intensity", "Autocompute Intensity Bounds", "true", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"min_intensity", "Min Intensity", "0.0", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"max_intensity", "Max Intensity", "10.0", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "mesh|point_cloud|flat_point_cloud"},
      {"show_grid_lines", "Show Grid Lines", "true", {},
       common::DisplayPropertyKind::kAuto, "visualization", "mesh"},
      {"grid_cell_decimation", "Grid Cell Decimation", "1", {},
       common::DisplayPropertyKind::kAuto, "visualization", "mesh"},
      {"mesh_decimation", "Mesh Decimation", "1", {},
       common::DisplayPropertyKind::kAuto, "visualization", "mesh"},

      // --- point_cloud / flat ---
      {"layer", "Layer", "elevation", {}, common::DisplayPropertyKind::kAuto,
       "visualization",
       "point_cloud|flat_point_cloud|occupancy_grid|grid_cells"},
      {"flat_height", "Flat Height", "0.0", {},
       common::DisplayPropertyKind::kAuto, "visualization", "flat_point_cloud"},

      // --- occupancy_grid ---
      {"data_min", "Data Min", "0.0", {}, common::DisplayPropertyKind::kAuto,
       "visualization", "occupancy_grid"},
      {"data_max", "Data Max", "1.0", {}, common::DisplayPropertyKind::kAuto,
       "visualization", "occupancy_grid"},

      // --- grid_cells ---
      {"lower_threshold", "Lower Threshold", "-1e9", {},
       common::DisplayPropertyKind::kAuto, "visualization", "grid_cells"},
      {"upper_threshold", "Upper Threshold", "1e9", {},
       common::DisplayPropertyKind::kAuto, "visualization", "grid_cells"},

      // --- vectors ---
      {"layer_prefix", "Layer Prefix", "normal_", {},
       common::DisplayPropertyKind::kAuto, "visualization", "vectors"},
      {"position_layer", "Position Layer", "elevation", {},
       common::DisplayPropertyKind::kAuto, "visualization", "vectors"},
      {"vector_scale", "Vector Scale", "1.0", {},
       common::DisplayPropertyKind::kAuto, "visualization", "vectors"},

      // --- map_region / shared ---
      {"line_width", "Line Width", "0.01", {},
       common::DisplayPropertyKind::kAuto, "visualization",
       "map_region|vectors"},
  };
}

void GridMapDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (has_message_) {
    rebuildGeometry();
  }
}

void GridMapDisplay::clearGeometry() {
  mesh_vertices_.clear();
  mesh_triangles_.clear();
  grid_lines_.clear();
  points_.clear();
  occupancy_image_ = QImage();
  cells_.clear();
  viz_lines_.clear();
  has_geometry_ = false;
}

bool GridMapDisplay::parseLayers(
    const automsgs::msgs::map_msgs::GridMap& message) {
  layers_.clear();
  basic_layers_.assign(message.basic_layers().begin(),
                       message.basic_layers().end());
  if (message.layers_size() == 0 ||
      message.layers_size() != message.data_size()) {
    return false;
  }
  for (int i = 0; i < message.layers_size(); ++i) {
    LayerData layer;
    layer.name = message.layers(i);
    if (!ParseLayerMatrix(message.data(i), &layer.rows, &layer.cols,
                          &layer.values)) {
      return false;
    }
    layers_.push_back(std::move(layer));
  }
  return !layers_.empty();
}

const GridMapDisplay::LayerData* GridMapDisplay::findLayer(
    const std::string& name) const {
  for (const auto& layer : layers_) {
    if (layer.name == name) {
      return &layer;
    }
  }
  return layers_.empty() ? nullptr : &layers_.front();
}

float GridMapDisplay::cellValue(const LayerData& layer, int row,
                               int col) const {
  if (row < 0 || col < 0 || row >= layer.rows || col >= layer.cols) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const int br = (row + outer_start_) % layer.rows;
  const int bc = (col + inner_start_) % layer.cols;
  return layer.values[static_cast<size_t>(br * layer.cols + bc)];
}

bool GridMapDisplay::cellValid(int row, int col,
                              const LayerData* height_layer) const {
  auto finite_at = [&](const LayerData& layer) {
    return std::isfinite(cellValue(layer, row, col));
  };
  if (height_layer != nullptr && !finite_at(*height_layer)) {
    return false;
  }
  for (const auto& name : basic_layers_) {
    for (const auto& candidate : layers_) {
      if (candidate.name == name) {
        if (!finite_at(candidate)) {
          return false;
        }
        break;
      }
    }
  }
  return true;
}

QColor GridMapDisplay::colorForCell(int row, int col,
                                    const LayerData* color_layer,
                                    float min_intensity,
                                    float max_intensity) const {
  const std::string mode =
      propertyValue("color_transformer", "IntensityLayer");
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);

  if (mode == "None") {
    return QColor(0, 0, 0, 0);
  }
  if (mode == "FlatColor" || color_layer == nullptr) {
    return WithAlpha(
        common::ParseColorProperty(propertyValue("color", "200;200;200")),
        alpha);
  }
  if (mode == "ColorLayer") {
    return WithAlpha(ColorFromPackedFloat(cellValue(*color_layer, row, col)),
                     alpha);
  }
  const float value = cellValue(*color_layer, row, col);
  float t = NormalizeIntensity(value, min_intensity, max_intensity);
  if (common::ParseBoolProperty(propertyValue("invert_colormap", "false"),
                                false)) {
    t = 1.f - t;
  }
  QColor color;
  if (common::ParseBoolProperty(propertyValue("use_colormap", "true"), true)) {
    color = SampleGridMapColorMap(propertyValue("colormap", "default"), t);
  } else {
    color = LerpColor(
        t, common::ParseColorProperty(propertyValue("min_color", "0;0;0")),
        common::ParseColorProperty(propertyValue("max_color", "255;255;255")));
  }
  return WithAlpha(color, alpha);
}

QVector3D GridMapDisplay::cellPosition(int row, int col,
                                      const LayerData* height_layer, bool flat,
                                      float flat_height) const {
  const double top_left_x =
      center_x_ + 0.5 * length_x_ - 0.5 * static_cast<double>(resolution_);
  const double top_left_y =
      center_y_ + 0.5 * length_y_ - 0.5 * static_cast<double>(resolution_);
  float z = flat_height;
  if (!flat && height_layer != nullptr) {
    z = cellValue(*height_layer, row, col);
  }
  return QVector3D(
      static_cast<float>(top_left_x - static_cast<double>(row) * resolution_),
      static_cast<float>(top_left_y - static_cast<double>(col) * resolution_),
      z);
}

void GridMapDisplay::computeIntensityBounds(const LayerData* color_layer,
                                            float* min_v, float* max_v) const {
  *min_v = common::ParseFloatProperty(propertyValue("min_intensity", "0.0"), 0.f);
  *max_v =
      common::ParseFloatProperty(propertyValue("max_intensity", "10.0"), 10.f);
  if (!common::ParseBoolProperty(propertyValue("autocompute_intensity", "true"),
                                 true) ||
      color_layer == nullptr) {
    return;
  }
  *min_v = std::numeric_limits<float>::infinity();
  *max_v = -std::numeric_limits<float>::infinity();
  for (float v : color_layer->values) {
    if (!std::isfinite(v)) {
      continue;
    }
    *min_v = std::min(*min_v, v);
    *max_v = std::max(*max_v, v);
  }
  if (!std::isfinite(*min_v) || !std::isfinite(*max_v)) {
    *min_v = 0.f;
    *max_v = 1.f;
  }
  *max_v = *min_v + std::max(*max_v - *min_v, 0.2f);
}

bool GridMapDisplay::updateFrameTransform() {
  frame_to_fixed_.setToIdentity();
  if (context_ == nullptr) {
    return false;
  }
  const std::string frame =
      current_message_.info().header().frame_id().empty()
          ? context_->fixed_frame
          : current_message_.info().header().frame_id();
  if (frame == context_->fixed_frame) {
    return true;
  }
  try {
    const auto tf = context_->tf_buffer->lookupTransform(
        context_->fixed_frame, frame, autoviz::commsgs::ZeroTime());
    frame_to_fixed_ = transformToMatrix(tf);
    return true;
  } catch (...) {
    return false;
  }
}

void GridMapDisplay::rebuildMesh() {
  const LayerData& ref = layers_.front();
  const int rows = ref.rows;
  const int cols = ref.cols;
  if (rows < 2 || cols < 2) {
    return;
  }

  const bool flat = propertyValue("height_transformer", "Layer") == "Flat";
  const LayerData* height_layer =
      flat ? &ref : findLayer(propertyValue("height_layer", "elevation"));
  const LayerData* color_layer =
      findLayer(propertyValue("color_layer", "elevation"));
  if (height_layer == nullptr) {
    return;
  }

  const std::string color_mode =
      propertyValue("color_transformer", "IntensityLayer");
  const bool draw_mesh = color_mode != "None";
  const bool show_lines =
      common::ParseBoolProperty(propertyValue("show_grid_lines", "true"), true);
  const int mesh_decim = std::max(
      1, common::ParseIntProperty(propertyValue("mesh_decimation", "1"), 1));
  const int line_decim = std::max(
      1,
      common::ParseIntProperty(propertyValue("grid_cell_decimation", "1"), 1));

  float min_i = 0.f;
  float max_i = 1.f;
  computeIntensityBounds(color_layer, &min_i, &max_i);

  std::vector<int> index_to_vertex(static_cast<size_t>(rows * cols), -1);
  auto vert_id = [&](int i, int j) -> int& {
    return index_to_vertex[static_cast<size_t>(i * cols + j)];
  };

  if (draw_mesh) {
    for (int i = 0; i < rows; i += mesh_decim) {
      for (int j = 0; j < cols; j += mesh_decim) {
        if (!cellValid(i, j, height_layer)) {
          continue;
        }
        MeshVertex vertex;
        vertex.position = cellPosition(i, j, height_layer, flat, 0.f);
        vertex.color = colorForCell(i, j, color_layer, min_i, max_i);
        vert_id(i, j) = static_cast<int>(mesh_vertices_.size());
        mesh_vertices_.push_back(vertex);
        if (i == 0 || j == 0) {
          continue;
        }
        std::vector<int> ids;
        for (int k = 0; k < 2; ++k) {
          for (int l = 0; l < 2; ++l) {
            const int ii = i - k * mesh_decim;
            const int jj = j - l * mesh_decim;
            if (ii < 0 || jj < 0 || !cellValid(ii, jj, height_layer)) {
              continue;
            }
            const int id = vert_id(ii, jj);
            if (id >= 0) {
              ids.push_back(id);
            }
          }
        }
        if (ids.size() == 3) {
          mesh_triangles_.push_back({ids[0], ids[1], ids[2]});
        } else if (ids.size() == 4) {
          mesh_triangles_.push_back({ids[0], ids[2], ids[3]});
          mesh_triangles_.push_back({ids[0], ids[3], ids[1]});
        }
      }
    }
  }

  if (show_lines) {
    for (int i = 0; i < rows; i += line_decim) {
      for (int j = 0; j < cols; j += line_decim) {
        if (!cellValid(i, j, height_layer)) {
          continue;
        }
        auto add_edge = [&](int i0, int j0, int i1, int j1) {
          if (!cellValid(i0, j0, height_layer) ||
              !cellValid(i1, j1, height_layer)) {
            return;
          }
          grid_lines_.push_back(
              {cellPosition(i0, j0, height_layer, flat, 0.f),
               cellPosition(i1, j1, height_layer, flat, 0.f)});
        };
        const int jn = std::min(j + line_decim, cols - 1);
        const int in = std::min(i + line_decim, rows - 1);
        if (jn != j) {
          add_edge(i, j, i, jn);
        }
        if (in != i) {
          add_edge(i, j, in, j);
        }
      }
    }
  }
}

void GridMapDisplay::rebuildPointCloud(bool flat) {
  const LayerData* layer =
      findLayer(propertyValue("layer", propertyValue("height_layer", "elevation")));
  if (layer == nullptr) {
    return;
  }
  const LayerData* color_layer =
      findLayer(propertyValue("color_layer", layer->name));
  const float flat_height =
      common::ParseFloatProperty(propertyValue("flat_height", "0.0"), 0.f);
  float min_i = 0.f;
  float max_i = 1.f;
  computeIntensityBounds(color_layer, &min_i, &max_i);

  const int decim = std::max(
      1, common::ParseIntProperty(propertyValue("mesh_decimation", "1"), 1));
  for (int i = 0; i < layer->rows; i += decim) {
    for (int j = 0; j < layer->cols; j += decim) {
      if (!cellValid(i, j, layer)) {
        continue;
      }
      MeshVertex p;
      p.position = cellPosition(i, j, layer, flat, flat_height);
      p.color = colorForCell(i, j, color_layer, min_i, max_i);
      points_.push_back(p);
    }
  }
}

void GridMapDisplay::rebuildOccupancyGrid() {
  const LayerData* layer = findLayer(propertyValue("layer", "elevation"));
  if (layer == nullptr || layer->rows < 1 || layer->cols < 1) {
    return;
  }
  const float data_min =
      common::ParseFloatProperty(propertyValue("data_min", "0.0"), 0.f);
  const float data_max =
      common::ParseFloatProperty(propertyValue("data_max", "1.0"), 1.f);
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const float range = std::max(data_max - data_min, 1e-6f);

  // Match GridMapRosConverter::toOccupancyGrid reverse cell order.
  const int rows = layer->rows;
  const int cols = layer->cols;
  const int n_cells = rows * cols;
  occupancy_image_ =
      QImage(cols, rows, QImage::Format_RGBA8888);
  occupancy_image_.fill(Qt::transparent);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const float raw = cellValue(*layer, i, j);
      int32_t occ = -1;
      if (std::isfinite(raw)) {
        const float t = std::clamp((raw - data_min) / range, 0.f, 1.f);
        occ = static_cast<int32_t>(std::lround(t * 100.f));
      }
      const int linear = i * cols + j;
      const int rev = n_cells - linear - 1;
      const int img_row = rev / cols;
      const int img_col = rev % cols;
      occupancy_image_.setPixelColor(img_col, rows - 1 - img_row,
                                     OccupancyPalette(occ, alpha));
    }
  }

  // Origin = center - 0.5 * length (same as toOccupancyGrid).
  const float ox = static_cast<float>(center_x_ - 0.5 * length_x_);
  const float oy = static_cast<float>(center_y_ - 0.5 * length_y_);
  const float w = static_cast<float>(cols) * resolution_;
  const float h = static_cast<float>(rows) * resolution_;
  occupancy_corners_[0] = QVector3D(ox, oy + h, 0.02f);      // top-left
  occupancy_corners_[1] = QVector3D(ox + w, oy + h, 0.02f);  // top-right
  occupancy_corners_[2] = QVector3D(ox + w, oy, 0.02f);      // bottom-right
  occupancy_corners_[3] = QVector3D(ox, oy, 0.02f);          // bottom-left
}

void GridMapDisplay::rebuildGridCells() {
  const LayerData* layer = findLayer(propertyValue("layer", "elevation"));
  if (layer == nullptr) {
    return;
  }
  const float lower = common::ParseFloatProperty(
      propertyValue("lower_threshold", "-1e9"), -1e9f);
  const float upper = common::ParseFloatProperty(
      propertyValue("upper_threshold", "1e9"), 1e9f);
  for (int i = 0; i < layer->rows; ++i) {
    for (int j = 0; j < layer->cols; ++j) {
      if (!cellValid(i, j, layer)) {
        continue;
      }
      const float v = cellValue(*layer, i, j);
      if (v < lower || v > upper) {
        continue;
      }
      CellBox cell;
      cell.center = cellPosition(i, j, layer, true, 0.01f);
      cell.width = resolution_;
      cell.height = resolution_;
      cells_.push_back(cell);
    }
  }
}

void GridMapDisplay::rebuildMapRegion() {
  const float hx = length_x_ * 0.5f;
  const float hy = length_y_ * 0.5f;
  const QVector3D c(static_cast<float>(center_x_),
                    static_cast<float>(center_y_), 0.02f);
  const QVector3D p0(c.x() + hx, c.y() + hy, c.z());
  const QVector3D p1(c.x() + hx, c.y() - hy, c.z());
  const QVector3D p2(c.x() - hx, c.y() - hy, c.z());
  const QVector3D p3(c.x() - hx, c.y() + hy, c.z());
  viz_lines_.push_back({p0, p1});
  viz_lines_.push_back({p1, p2});
  viz_lines_.push_back({p2, p3});
  viz_lines_.push_back({p3, p0});
  viz_line_color_ = WithAlpha(
      common::ParseColorProperty(propertyValue("color", "255;255;255")),
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f));
}

void GridMapDisplay::rebuildVectors() {
  const std::string prefix = propertyValue("layer_prefix", "normal_");
  const LayerData* lx = findLayer(prefix + "x");
  const LayerData* ly = findLayer(prefix + "y");
  const LayerData* lz = findLayer(prefix + "z");
  const LayerData* pos =
      findLayer(propertyValue("position_layer", "elevation"));
  if (lx == nullptr || ly == nullptr || lz == nullptr || pos == nullptr) {
    return;
  }
  if (lx->rows != pos->rows || lx->cols != pos->cols) {
    return;
  }
  const float scale =
      common::ParseFloatProperty(propertyValue("vector_scale", "1.0"), 1.f);
  viz_line_color_ = WithAlpha(
      common::ParseColorProperty(propertyValue("color", "0;255;0")),
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f));

  const int decim = std::max(
      1, common::ParseIntProperty(propertyValue("mesh_decimation", "1"), 1));
  for (int i = 0; i < pos->rows; i += decim) {
    for (int j = 0; j < pos->cols; j += decim) {
      if (!cellValid(i, j, pos)) {
        continue;
      }
      const QVector3D start = cellPosition(i, j, pos, false, 0.f);
      const QVector3D end =
          start +
          scale * QVector3D(cellValue(*lx, i, j), cellValue(*ly, i, j),
                            cellValue(*lz, i, j));
      viz_lines_.push_back({start, end});
    }
  }
}

void GridMapDisplay::rebuildGeometry() {
  clearGeometry();
  if (context_ == nullptr || !has_message_ || layers_.empty() ||
      resolution_ <= 0.f) {
    return;
  }
  if (!updateFrameTransform()) {
    return;
  }

  const std::string viz = propertyValue("visualization", "mesh");
  if (viz == "point_cloud") {
    rebuildPointCloud(false);
  } else if (viz == "flat_point_cloud") {
    rebuildPointCloud(true);
  } else if (viz == "occupancy_grid") {
    rebuildOccupancyGrid();
  } else if (viz == "grid_cells") {
    rebuildGridCells();
  } else if (viz == "map_region") {
    rebuildMapRegion();
  } else if (viz == "vectors") {
    rebuildVectors();
  } else {
    rebuildMesh();
  }

  has_geometry_ = !mesh_triangles_.empty() || !grid_lines_.empty() ||
                  !points_.empty() || !occupancy_image_.isNull() ||
                  !cells_.empty() || !viz_lines_.empty();
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void GridMapDisplay::processMessage(
    const automsgs::msgs::map_msgs::GridMap& message) {
  current_message_ = message;
  has_message_ = false;
  clearGeometry();

  if (!message.has_info() || message.info().resolution() <= 0.f) {
    return;
  }
  if (!parseLayers(message)) {
    return;
  }

  resolution_ = message.info().resolution();
  length_x_ = message.info().length_x();
  length_y_ = message.info().length_y();
  center_x_ = message.info().has_pose() ? message.info().pose().position().x()
                                        : 0.0;
  center_y_ = message.info().has_pose() ? message.info().pose().position().y()
                                        : 0.0;
  outer_start_ = static_cast<int>(message.outer_start_index());
  inner_start_ = static_cast<int>(message.inner_start_index());
  if (length_x_ <= 0.f) {
    length_x_ = static_cast<float>(layers_.front().rows) * resolution_;
  }
  if (length_y_ <= 0.f) {
    length_y_ = static_cast<float>(layers_.front().cols) * resolution_;
  }

  has_message_ = true;
  rebuildGeometry();
}

void GridMapDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!has_geometry_) {
    return;
  }

  for (const auto& tri : mesh_triangles_) {
    const MeshVertex& v0 = mesh_vertices_[static_cast<size_t>(tri[0])];
    const MeshVertex& v1 = mesh_vertices_[static_cast<size_t>(tri[1])];
    const MeshVertex& v2 = mesh_vertices_[static_cast<size_t>(tri[2])];
    scene.addColoredTriangle(
        frame_to_fixed_.map(v0.position), frame_to_fixed_.map(v1.position),
        frame_to_fixed_.map(v2.position), v0.color, v1.color, v2.color);
  }

  if (!grid_lines_.empty()) {
    QColor line_color(0, 0, 0);
    line_color.setAlphaF(std::clamp(
        common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f), 0.f,
        1.f));
    for (const auto& line : grid_lines_) {
      scene.addLine(frame_to_fixed_.map(line.a), frame_to_fixed_.map(line.b),
                    line_color);
    }
  }

  for (const auto& p : points_) {
    scene.addPoint(frame_to_fixed_.map(p.position), p.color);
  }

  if (!occupancy_image_.isNull()) {
    scene.addTexturedQuad(
        frame_to_fixed_.map(occupancy_corners_[0]),
        frame_to_fixed_.map(occupancy_corners_[1]),
        frame_to_fixed_.map(occupancy_corners_[2]),
        frame_to_fixed_.map(occupancy_corners_[3]), occupancy_image_,
        rendering::SceneOverlay::TextureFilterMode::kNearest);
  }

  if (!cells_.empty()) {
    QColor color = WithAlpha(
        common::ParseColorProperty(propertyValue("color", "255;200;80")),
        common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f));
    for (const auto& cell : cells_) {
      const QVector3D c = frame_to_fixed_.map(cell.center);
      const float hw = cell.width * 0.5f;
      const float hh = cell.height * 0.5f;
      const QVector3D p0(c.x() - hw, c.y() - hh, c.z());
      const QVector3D p1(c.x() + hw, c.y() - hh, c.z());
      const QVector3D p2(c.x() + hw, c.y() + hh, c.z());
      const QVector3D p3(c.x() - hw, c.y() + hh, c.z());
      scene.addLine(p0, p1, color);
      scene.addLine(p1, p2, color);
      scene.addLine(p2, p3, color);
      scene.addLine(p3, p0, color);
    }
  }

  for (const auto& line : viz_lines_) {
    scene.addLine(frame_to_fixed_.map(line.a), frame_to_fixed_.map(line.b),
                  viz_line_color_);
  }
}

}  // namespace display
}  // namespace autoviz
