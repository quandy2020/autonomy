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

namespace autoviz {
namespace channel_graph {

struct ChannelGraphLayoutOptions {
  double channel_column_x = 0.0;
  double writer_column_x = -460.0;
  double reader_column_x = 460.0;
  double service_row_gap = 100.0;
  double channel_row_gap = 58.0;
  double service_column_x = 820.0;
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

 signals:
  void graphRendered(int vertex_count, int edge_count);

 protected:
  void wheelEvent(QWheelEvent* event) override;
  void showEvent(QShowEvent* event) override;

 private slots:
  void onVertexMoved();

 private:
  void applyAutoLayout(const integration::TopologyGraph& graph);
  QPointF defaultPositionForVertex(const integration::GraphVertex& vertex,
                                   int channel_index, int service_index,
                                   int channel_count, int service_count) const;

  QGraphicsScene* scene_ = nullptr;
  ChannelGraphLayoutOptions layout_;
  QHash<QString, QPointF> saved_positions_;
  QString last_topology_hash_;
  double zoom_factor_ = 1.0;
};

}  // namespace channel_graph
}  // namespace autoviz
