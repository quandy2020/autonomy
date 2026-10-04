/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Autoviz Display for map_msgs/GridMap:
 *  - mesh mode ← grid_map_rviz_plugin
 *  - point_cloud / flat_point_cloud / occupancy_grid / grid_cells /
 *    map_region / vectors ← grid_map_visualization
 *****************************************************************************/

/**
 * @file grid_map_display.hpp
 * @brief Renders @c map_msgs/GridMap with the visualization modes of
 *        @c grid_map_visualization plus the elevation mesh from
 *        @c grid_map_rviz_plugin.
 *
 * Supported modes (property-selected): mesh, point cloud, flat point cloud,
 * occupancy grid, grid cells, map region outline, and vector field layers.
 * Layer parsing, intensity coloring, and TF are handled privately after each
 * message / property change.
 *
 * @see ChannelDisplay
 * @see SampleGridMapColorMap
 * @see GridCellsDisplay
 * @see MapDisplay
 */

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
 * @class GridMapDisplay
 * @brief Multi-mode GridMap visualizer (mesh / cloud / occupancy / cells /
 *        region / vectors).
 *
 * ## Pipeline
 *
 * @ref processMessage → @ref parseLayers → @ref rebuildGeometry (mode
 * switch) → @ref onDraw emits the active geometry buffers.
 *
 * Color uses flat color, intensity layer, or a named colormap from
 * @ref SampleGridMapColorMap.
 *
 * @see SampleGridMapColorMap
 * @see ChannelDisplay
 */
class GridMapDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::GridMap> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for GridMap messages.
   */
  explicit GridMapDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "GridMap".
   */
  std::string typeId() const override { return "GridMap"; }

  /**
   * @brief Property schema (mode, layers, colormap, decimation, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Parses layers and rebuilds visualization geometry.
   *
   * @param message map_msgs/GridMap protobuf.
   */
  void processMessage(const automsgs::msgs::map_msgs::GridMap& message)
      override;

  /**
   * @brief Draws the active mode's cached geometry into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Rebuilds geometry when visualization-related properties change.
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

  /**
   * @brief Clears message caches and all geometry buffers.
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct LayerData
   * @brief One GridMap layer as a dense row-major float grid.
   */
  struct LayerData {
    std::string name;           /**< Layer name from the message. */
    int rows = 0;               /**< Number of rows. */
    int cols = 0;               /**< Number of columns. */
    /** Row-major cell values (@c rows * @c cols). */
    std::vector<float> values;
  };

  /**
   * @struct MeshVertex
   * @brief Colored vertex for elevation mesh / point cloud modes.
   */
  struct MeshVertex {
    QVector3D position;  /**< Vertex position in map / fixed frame. */
    QColor color;        /**< Per-vertex RGBA. */
  };

  /**
   * @struct MeshLine
   * @brief Line segment for grid lines, map region, or vector glyphs.
   */
  struct MeshLine {
    QVector3D a;  /**< Segment start. */
    QVector3D b;  /**< Segment end. */
  };

  /**
   * @struct CellBox
   * @brief Rectangle used by grid_cells visualization mode.
   */
  struct CellBox {
    QVector3D center;   /**< Cell center. */
    float width = 0.f;  /**< Cell width (meters). */
    float height = 0.f; /**< Cell height (meters). */
  };

  /** Clears all mode-specific geometry buffers and @c has_geometry_. */
  void clearGeometry();

  /** Dispatches to the active mode rebuild based on properties. */
  void rebuildGeometry();

  /**
   * @brief Resolves map frame → fixed frame into @c frame_to_fixed_.
   *
   * @return @c true when the transform is available.
   */
  bool updateFrameTransform();

  /**
   * @brief Extracts layer grids and metadata from @p message.
   *
   * @param message Source GridMap.
   * @return @c true on successful parse.
   */
  bool parseLayers(const automsgs::msgs::map_msgs::GridMap& message);

  /**
   * @brief Looks up a parsed layer by name.
   *
   * @param name Layer name.
   * @return Pointer into @c layers_, or @c nullptr if missing.
   */
  const LayerData* findLayer(const std::string& name) const;

  /**
   * @brief Reads one cell value from @p layer.
   *
   * @param layer Layer grid.
   * @param row Row index.
   * @param col Column index.
   * @return Cell value (NaN if out of range — see .cpp).
   */
  float cellValue(const LayerData& layer, int row, int col) const;

  /**
   * @brief Whether cell (@p row,@p col) should be drawn given basic layers.
   *
   * @param row Row index.
   * @param col Column index.
   * @param height_layer Optional height layer for validity.
   * @return @c true when the cell is considered valid.
   */
  bool cellValid(int row, int col, const LayerData* height_layer) const;

  /**
   * @brief Computes display color for one cell from color layer / colormap.
   *
   * @param row Row index.
   * @param col Column index.
   * @param color_layer Optional intensity/color layer.
   * @param min_intensity Intensity lower bound.
   * @param max_intensity Intensity upper bound.
   * @return RGBA for the cell.
   */
  QColor colorForCell(int row, int col, const LayerData* color_layer,
                      float min_intensity, float max_intensity) const;

  /**
   * @brief Maps cell indices to a 3D position (elevation or flat).
   *
   * @param row Row index.
   * @param col Column index.
   * @param height_layer Elevation layer (ignored when @p flat).
   * @param flat When @c true, uses @p flat_height for Z.
   * @param flat_height Constant height in flat mode.
   * @return Cell center / vertex position.
   */
  QVector3D cellPosition(int row, int col, const LayerData* height_layer,
                         bool flat, float flat_height) const;

  /**
   * @brief Fills intensity min/max from properties or autocompute.
   *
   * @param color_layer Layer used for intensity statistics.
   * @param[out] min_v Minimum intensity.
   * @param[out] max_v Maximum intensity.
   */
  void computeIntensityBounds(const LayerData* color_layer, float* min_v,
                              float* max_v) const;

  /** Builds elevation mesh vertices/triangles and optional grid lines. */
  void rebuildMesh();

  /**
   * @brief Builds point cloud (or flat point cloud) vertices.
   *
   * @param flat When @c true, ignores elevation layer height.
   */
  void rebuildPointCloud(bool flat);

  /** Builds occupancy-grid style image + corner quads. */
  void rebuildOccupancyGrid();

  /** Builds thresholded grid cell boxes. */
  void rebuildGridCells();

  /** Builds map region outline lines. */
  void rebuildMapRegion();

  /** Builds vector-field glyph lines from vector layers. */
  void rebuildVectors();

  /** Latest GridMap message retained for rebuilds. */
  automsgs::msgs::map_msgs::GridMap current_message_;

  /** Parsed numeric layers. */
  std::vector<LayerData> layers_;

  /** Basic (validity) layer names from the message. */
  std::vector<std::string> basic_layers_;

  float resolution_ = 0.f;   /**< Cell resolution (meters). */
  float length_x_ = 0.f;     /**< Map length in X. */
  float length_y_ = 0.f;     /**< Map length in Y. */
  double center_x_ = 0.0;    /**< Map center X in map frame. */
  double center_y_ = 0.0;    /**< Map center Y in map frame. */
  int outer_start_ = 0;      /**< Outer index start (circular buffer). */
  int inner_start_ = 0;      /**< Inner index start (circular buffer). */
  bool has_message_ = false; /**< @c true after a successful parse. */

  // mesh / grid lines (rviz_plugin)
  std::vector<MeshVertex> mesh_vertices_;              /**< Elevation mesh verts. */
  std::vector<std::array<int, 3>> mesh_triangles_;     /**< Triangle indices. */
  std::vector<MeshLine> grid_lines_;                   /**< Optional mesh grid. */

  // point_cloud / flat_point_cloud
  std::vector<MeshVertex> points_;  /**< Point cloud samples. */

  // occupancy_grid
  QImage occupancy_image_;             /**< Occupancy texture. */
  QVector3D occupancy_corners_[4];     /**< Texture quad corners. */

  // grid_cells
  std::vector<CellBox> cells_;  /**< Thresholded cell boxes. */

  // map_region / vectors share line list
  std::vector<MeshLine> viz_lines_;              /**< Region / vector lines. */
  QColor viz_line_color_{255, 255, 255};         /**< Line draw color. */

  QMatrix4x4 frame_to_fixed_;   /**< Map frame → fixed frame. */
  bool has_geometry_ = false;   /**< @c true when a mode buffer is ready. */
};

}  // namespace display
}  // namespace autoviz
