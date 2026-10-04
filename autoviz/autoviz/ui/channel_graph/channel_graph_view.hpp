/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_graph_view.hpp
 * @brief Interactive QGraphicsView canvas for Autolink channel / service
 *        topology.
 *
 * Renders vertices and edges from @ref integration::TopologyGraph, supports
 * grid/column arrange modes, neighborhood focus, edge-label toggles, and
 * animated flow / activity ticks.
 *
 * @see ChannelGraphPanel
 * @see integration::TopologyGraph
 * @see integration::TopologyGraphBuilder
 */

#pragma once

#include <QGraphicsView>
#include <QHash>
#include <QPoint>
#include <QPointF>
#include <QString>

#include <utility>
#include <vector>

#include "autoviz/integration/topology_graph_builder.hpp"

class QContextMenuEvent;
class QGraphicsScene;
class QMouseEvent;
class QTimer;

namespace autoviz {
namespace channel_graph {

/**
 * @enum VertexArrangeMode
 * @brief How channel / service vertices are arranged in the canvas.
 */
enum class VertexArrangeMode {
  kColumn = 0,  /**< Single vertical column. */
  kGrid = 1,    /**< Multi-column grid wrapping by height / width. */
};

/**
 * @struct ChannelGraphLayoutOptions
 * @brief Numeric layout knobs for auto-placing channel and service blocks.
 *
 * Consumed by @ref ChannelGraphView::applyAutoLayout(). Callers typically keep
 * the defaults; panel config only exposes arrange mode, not these distances.
 */
struct ChannelGraphLayoutOptions {
  double channel_block_left_x = -160.0;   /**< Left edge of the channel block. */
  double channel_block_right_x = 160.0;   /**< Right edge of the channel block. */
  double channel_column_x = 0.0;          /**< Center X of channel headers. */
  double channel_col_gap = 340.0;         /**< Horizontal gap between channel columns. */
  int channel_columns = 1;                /**< Computed column count for channels. */
  int channel_rows = 1;                   /**< Computed row count for channels. */
  double writer_column_x = -620.0;        /**< X for writer-only node column. */
  double reader_column_x = 620.0;         /**< X for reader-only node column. */
  double both_writer_column_x = -400.0;   /**< X for nodes that both write (left). */
  double both_reader_column_x = 400.0;    /**< X for nodes that both read (right). */
  double service_origin_x = 0.0;          /**< Origin X of the service grid. */
  double service_origin_y = 0.0;          /**< Origin Y of the service grid. */
  double service_col_gap = 220.0;         /**< Horizontal gap between service columns. */
  double service_row_gap = 180.0;         /**< Vertical gap between service rows. */
  int service_columns = 1;                /**< Computed column count for services. */
  double channel_row_gap = 78.0;          /**< Vertical gap between channel rows. */
  double node_row_gap = 150.0;            /**< Vertical gap between node rows. */
  double section_gap = 220.0;             /**< Gap between channel and service sections. */
};

/**
 * @class ChannelGraphView
 * @brief Interactive graph canvas for Autolink channel topology.
 *
 * ## Interaction
 *
 * - Wheel zooms about the cursor; @ref zoomToFit() frames the whole scene.
 * - Vertices are movable; positions are remembered in @c saved_positions_
 *   across topology refreshes when @c preserve_positions is true.
 * - Double-click emits @ref vertexDoubleClicked() for the properties strip.
 * - Neighborhood mode dims non-adjacent vertices when one is selected.
 *
 * ## Animation
 *
 * @c flow_timer_ advances edge flow dashes; @c activity_timer_ pulses
 * recently active vertices.
 *
 * @see ChannelGraphPanel::refreshGraph()
 * @see VertexArrangeMode
 */
class ChannelGraphView : public QGraphicsView {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty scene with open-hand drag and antialiasing.
   *
   * @param parent Qt parent (typically @ref ChannelGraphPanel).
   */
  explicit ChannelGraphView(QWidget* parent = nullptr);

  /**
   * @brief Replaces the scene contents from a topology graph.
   *
   * @param graph Source vertices and edges from TopologyGraphBuilder.
   * @param preserve_positions When @c true, reuses @c saved_positions_ for
   *        matching vertex ids; otherwise runs a full @ref applyAutoLayout().
   * @see graphRendered()
   */
  void setGraph(const integration::TopologyGraph& graph, bool preserve_positions);

  /**
   * @brief Returns the last known user-adjusted vertex positions.
   *
   * @return Map of vertex id → scene position.
   */
  QHash<QString, QPointF> savedPositions() const;

  /**
   * @brief Frames the entire scene in the viewport and resets zoom tracking.
   */
  void zoomToFit();

  /**
   * @brief Clears @c saved_positions_ so the next layout is fully automatic.
   */
  void resetSavedPositions();

  /**
   * @brief Sets arrange modes for channel and service vertex blocks.
   *
   * Does not rebuild immediately — the next @ref setGraph() / panel refresh
   * applies the new modes.
   *
   * @param channel_mode Arrange mode for channel vertices.
   * @param service_mode Arrange mode for service vertices.
   */
  void setArrangeModes(VertexArrangeMode channel_mode, VertexArrangeMode service_mode);

  /**
   * @brief Enables or disables neighborhood focus highlighting.
   *
   * @param enabled When @c true, selecting a vertex dims non-neighbors.
   */
  void setNeighborhoodMode(bool enabled);

  /**
   * @brief Shows or hides text labels on edges.
   *
   * @param enabled When @c true, edge items display relation labels.
   * @see applyEdgeLabelPolicy()
   */
  void setShowEdgeLabels(bool enabled);

  /**
   * @brief Current arrange mode for channel vertices.
   * @return @c channel_arrange_mode_.
   */
  VertexArrangeMode channelArrangeMode() const { return channel_arrange_mode_; }

  /**
   * @brief Current arrange mode for service vertices.
   * @return @c service_arrange_mode_.
   */
  VertexArrangeMode serviceArrangeMode() const { return service_arrange_mode_; }

  /**
   * @brief Whether neighborhood focus mode is enabled.
   * @return @c neighborhood_mode_.
   */
  bool neighborhoodMode() const { return neighborhood_mode_; }

  /**
   * @brief Whether edge labels are shown.
   * @return @c show_edge_labels_.
   */
  bool showEdgeLabels() const { return show_edge_labels_; }

  /**
   * @brief Lists channel vertices currently in the scene (label, message type).
   */
  std::vector<std::pair<QString, QString>> channelVertices() const;

 signals:
  /**
   * @brief Emitted after @ref setGraph() finishes building the scene.
   *
   * @param vertex_count Number of vertex items created.
   * @param edge_count Number of edge items created.
   */
  void graphRendered(int vertex_count, int edge_count);

  /**
   * @brief Emitted when the user double-clicks a vertex.
   *
   * @param vertex_id Stable topology id.
   * @param kind Channel / node / service discriminator.
   * @param label Short display name.
   * @param detail Extra detail string (type, counts, …).
   */
  void vertexDoubleClicked(const QString& vertex_id,
                           integration::GraphVertexKind kind,
                           const QString& label, const QString& detail);

  /**
   * @brief Emitted when the user right-clicks a vertex.
   *
   * @param vertex_id Stable topology id.
   * @param kind Channel / node / service discriminator.
   * @param label Short display name.
   * @param detail Extra detail string (message type for channels).
   * @param global_pos Global cursor position for the menu.
   */
  void vertexContextMenuRequested(const QString& vertex_id,
                                  integration::GraphVertexKind kind,
                                  const QString& label, const QString& detail,
                                  const QPoint& global_pos);

  /**
   * @brief Emitted when the selected vertex changes (or selection clears).
   *
   * @param has_selection Whether a vertex is selected.
   * @param vertex_id Stable topology id (empty when cleared).
   * @param kind Vertex kind (ignored when cleared).
   * @param label Display name (empty when cleared).
   * @param detail Extra detail / message type (empty when cleared).
   */
  void vertexSelectionChanged(bool has_selection, const QString& vertex_id,
                              integration::GraphVertexKind kind,
                              const QString& label, const QString& detail);

 protected:
  /**
   * @brief Zooms the view about the cursor on mouse wheel.
   *
   * @param event Wheel event.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * @brief Starts animation timers when the view becomes visible.
   *
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Forwards presses; may clear neighborhood focus on empty background.
   *
   * @param event Mouse press.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Forwards releases to the base class / items.
   *
   * @param event Mouse release.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Emits @ref vertexDoubleClicked() when a vertex item is hit.
   *
   * @param event Double-click event.
   */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /**
   * @brief Emits @ref vertexContextMenuRequested() when a vertex is hit.
   *
   * @param event Context-menu event.
   */
  void contextMenuEvent(QContextMenuEvent* event) override;

 private slots:
  /**
   * @brief Vertex item moved: records position into @c saved_positions_.
   */
  void onVertexMoved();

  /**
   * @brief Selection changed: applies or clears neighborhood highlight.
   */
  void onSelectionChanged();

  /**
   * @brief Advances edge flow animation phase (@c flow_phase_).
   */
  void onFlowTick();

  /**
   * @brief Pulses activity styling on recently active vertices.
   */
  void onActivityTick();

 private:
  /**
   * @brief Computes default positions for all vertices into the scene.
   *
   * @param graph Topology used for counts and kind-based columns.
   */
  void applyAutoLayout(const integration::TopologyGraph& graph);

  /**
   * @brief Creates or updates section header items (Channels / Services).
   *
   * @param service_count Number of service vertices (0 hides the service header).
   */
  void updateColumnHeaders(int service_count);

  /**
   * @brief Removes neighborhood dimming / focus styling.
   */
  void clearFocusHighlight();

  /**
   * @brief Dims non-neighbor vertices around @p vertex_id.
   *
   * @param vertex_id Focused vertex id.
   */
  void applyFocusHighlight(const QString& vertex_id);

  /**
   * @brief Starts or stops @c flow_timer_ based on edge presence / visibility.
   *
   * @param active When @c true, flow dashes animate.
   */
  void setFlowAnimationActive(bool active);

  /**
   * @brief Shows or hides labels on existing edge items per @c show_edge_labels_.
   */
  void applyEdgeLabelPolicy();

  /**
   * @brief Default scene position for a vertex under the current arrange modes.
   *
   * @param vertex Vertex descriptor.
   * @param channel_index Index among channel vertices (−1 if not a channel).
   * @param service_index Index among service vertices (−1 if not a service).
   * @param channel_count Total channel vertices.
   * @param service_count Total service vertices.
   * @return Scene position.
   */
  QPointF defaultPositionForVertex(const integration::GraphVertex& vertex,
                                   int channel_index, int service_index,
                                   int channel_count, int service_count) const;

  /** Owned graphics scene hosting vertex / edge items. */
  QGraphicsScene* scene_ = nullptr;

  /** Numeric layout distances for auto-placement. */
  ChannelGraphLayoutOptions layout_;

  /** Arrange mode for channel vertices. */
  VertexArrangeMode channel_arrange_mode_ = VertexArrangeMode::kGrid;

  /** Arrange mode for service vertices. */
  VertexArrangeMode service_arrange_mode_ = VertexArrangeMode::kGrid;

  /** When true, selection focuses the neighborhood. */
  bool neighborhood_mode_ = true;

  /** When true, edges show text labels. */
  bool show_edge_labels_ = false;

  /** User-dragged positions keyed by vertex id. */
  QHash<QString, QPointF> saved_positions_;

  /** Hash of the last applied topology; used to detect structural changes. */
  QString last_topology_hash_;

  /** Vertex id currently focused in neighborhood mode (empty if none). */
  QString focused_vertex_id_;

  /** Timer for edge flow dash animation. */
  QTimer* flow_timer_ = nullptr;

  /** Timer for activity pulse animation. */
  QTimer* activity_timer_ = nullptr;

  /** Phase angle / offset for flow dashes. */
  double flow_phase_ = 0.0;

  /** Cumulative wheel zoom factor relative to fit. */
  double zoom_factor_ = 1.0;
};

}  // namespace channel_graph
}  // namespace autoviz
