/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Autoviz Display for map_msgs/GridMap:
 *  - mesh mode ← grid_map_rviz_plugin
 *  - point_cloud / flat_point_cloud / occupancy_grid / grid_cells /
 *    map_region / vectors ← grid_map_visualization
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QImage>
#include <QMatrix4x4>
#include <QVector3D>
#include <array>
#include <string>
#include <vector>

#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * Renders GridMap using the same visualization types as
 * grid_map_visualization, plus the elevation mesh from grid_map_rviz_plugin.
 */
class GridMapDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::GridMap> {
 public:
  explicit GridMapDisplay(std::string channel);

  std::string typeId() const override { return "GridMap"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(const automsgs::msgs::map_msgs::GridMap& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void onPropertyChanged(const std::string& key) override;

 private:
  struct LayerData {
    std::string name;
    int rows = 0;
    int cols = 0;
    /** Row-major cell values (rows * cols). */
    std::vector<float> values;
  };

  struct MeshVertex {
    QVector3D position;
    QColor color;
  };

  struct MeshLine {
    QVector3D a;
    QVector3D b;
  };

  struct CellBox {
    QVector3D center;
    float width = 0.f;
    float height = 0.f;
  };

  void clearGeometry();
  void rebuildGeometry();
  bool updateFrameTransform();
  bool parseLayers(const automsgs::msgs::map_msgs::GridMap& message);
  const LayerData* findLayer(const std::string& name) const;
  float cellValue(const LayerData& layer, int row, int col) const;
  bool cellValid(int row, int col, const LayerData* height_layer) const;
  QColor colorForCell(int row, int col, const LayerData* color_layer,
                      float min_intensity, float max_intensity) const;
  QVector3D cellPosition(int row, int col, const LayerData* height_layer,
                         bool flat, float flat_height) const;
  void computeIntensityBounds(const LayerData* color_layer, float* min_v,
                              float* max_v) const;

  void rebuildMesh();
  void rebuildPointCloud(bool flat);
  void rebuildOccupancyGrid();
  void rebuildGridCells();
  void rebuildMapRegion();
  void rebuildVectors();

  automsgs::msgs::map_msgs::GridMap current_message_;
  std::vector<LayerData> layers_;
  std::vector<std::string> basic_layers_;
  float resolution_ = 0.f;
  float length_x_ = 0.f;
  float length_y_ = 0.f;
  double center_x_ = 0.0;
  double center_y_ = 0.0;
  int outer_start_ = 0;
  int inner_start_ = 0;
  bool has_message_ = false;

  // mesh / grid lines (rviz_plugin)
  std::vector<MeshVertex> mesh_vertices_;
  std::vector<std::array<int, 3>> mesh_triangles_;
  std::vector<MeshLine> grid_lines_;

  // point_cloud / flat_point_cloud
  std::vector<MeshVertex> points_;

  // occupancy_grid
  QImage occupancy_image_;
  QVector3D occupancy_corners_[4];

  // grid_cells
  std::vector<CellBox> cells_;

  // map_region / vectors share line list
  std::vector<MeshLine> viz_lines_;
  QColor viz_line_color_{255, 255, 255};

  QMatrix4x4 frame_to_fixed_;
  bool has_geometry_ = false;
};

}  // namespace display
}  // namespace autoviz
