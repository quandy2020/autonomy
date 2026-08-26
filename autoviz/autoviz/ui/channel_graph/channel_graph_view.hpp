/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QGraphicsView>
#include <QHash>
#include <QPointF>
#include <QString>

#include <vector>

#include "autoviz/integration/topology_graph_builder.hpp"

class QGraphicsScene;
class QMouseEvent;
class QTimer;

namespace autoviz {
namespace channel_graph {

/** How channel / service vertices are arranged in the canvas. */
enum class VertexArrangeMode {
  kColumn = 0,  // single vertical column
  kGrid = 1,    // multi-column grid wrapping by height / width
};

struct ChannelGraphLayoutOptions {
  double channel_block_left_x = -160.0;
  double channel_block_right_x = 160.0;
  double channel_column_x = 0.0;  // center of channel block (headers)
  double channel_col_gap = 340.0;
  int channel_columns = 1;
  int channel_rows = 1;
  double writer_column_x = -620.0;
  double reader_column_x = 620.0;
  double both_writer_column_x = -400.0;
  double both_reader_column_x = 400.0;
  double service_origin_x = 0.0;
  double service_origin_y = 0.0;
  double service_col_gap = 220.0;
  double service_row_gap = 180.0;
  int service_columns = 1;
  double channel_row_gap = 78.0;
  double node_row_gap = 150.0;
  double section_gap = 220.0;
};

/** Interactive graph canvas for Autolink channel topology. */
class ChannelGraphView : public QGraphicsView {
  Q_OBJECT

 public:
  explicit ChannelGraphView(QWidget* parent = nullptr);

  void setGraph(const integration::TopologyGraph& graph, bool preserve_positions);
  QHash<QString, QPointF> savedPositions() const;
  void zoomToFit();
  void resetSavedPositions();
  void setArrangeModes(VertexArrangeMode channel_mode, VertexArrangeMode service_mode);
  void setNeighborhoodMode(bool enabled);
  void setShowEdgeLabels(bool enabled);
  VertexArrangeMode channelArrangeMode() const { return channel_arrange_mode_; }
  VertexArrangeMode serviceArrangeMode() const { return service_arrange_mode_; }
  bool neighborhoodMode() const { return neighborhood_mode_; }
  bool showEdgeLabels() const { return show_edge_labels_; }

 signals:
  void graphRendered(int vertex_count, int edge_count);
  void vertexDoubleClicked(const QString& vertex_id,
                           integration::GraphVertexKind kind,
                           const QString& label, const QString& detail);

 protected:
  void wheelEvent(QWheelEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;

 private slots:
  void onVertexMoved();
  void onSelectionChanged();
  void onFlowTick();
  void onActivityTick();

 private:
  void applyAutoLayout(const integration::TopologyGraph& graph);
  void updateColumnHeaders(int service_count);
  void clearFocusHighlight();
  void applyFocusHighlight(const QString& vertex_id);
  void setFlowAnimationActive(bool active);
  void applyEdgeLabelPolicy();
  QPointF defaultPositionForVertex(const integration::GraphVertex& vertex,
                                   int channel_index, int service_index,
                                   int channel_count, int service_count) const;

  QGraphicsScene* scene_ = nullptr;
  ChannelGraphLayoutOptions layout_;
  VertexArrangeMode channel_arrange_mode_ = VertexArrangeMode::kGrid;
  VertexArrangeMode service_arrange_mode_ = VertexArrangeMode::kGrid;
  bool neighborhood_mode_ = true;
  bool show_edge_labels_ = false;
  QHash<QString, QPointF> saved_positions_;
  QString last_topology_hash_;
  QString focused_vertex_id_;
  QTimer* flow_timer_ = nullptr;
  QTimer* activity_timer_ = nullptr;
  double flow_phase_ = 0.0;
  double zoom_factor_ = 1.0;
};

}  // namespace channel_graph
}  // namespace autoviz
