/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_graph_panel.hpp
 * @brief Dockable panel hosting @ref ChannelGraphView with filter, arrange,
 *        and auto-refresh controls.
 *
 * Builds an @ref integration::TopologyGraph from VisualizationManager /
 * Autolink, pushes it into the view, and persists UI options via
 * @ref ChannelGraphPanelConfig for session / multi-pane cloning.
 *
 * @see ChannelGraphView
 * @see ChannelGraphPanelConfig
 * @see PanelDockWidget
 */

#pragma once

#include <QWidget>

#include <string>
#include <unordered_map>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/channel_graph/channel_graph_view.hpp"

class QLineEdit;
class QLabel;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace channel_graph {

/**
 * @struct ChannelGraphPanelConfig
 * @brief Serializable UI / view options for a Channel Graph pane.
 *
 * Written by @ref ChannelGraphPanel::config() and restored via
 * @ref ChannelGraphPanel::setConfig() / @ref cloneConfigFrom().
 */
struct ChannelGraphPanelConfig {
  bool show_services = true;       /**< Include service vertices in the graph. */
  bool show_channels = true;       /**< Include channel vertices in the graph. */
  bool auto_refresh = true;        /**< Periodically rebuild when topology changes. */
  bool neighborhood_mode = true;   /**< Dim non-neighbors on selection. */
  bool show_edge_labels = false;   /**< Draw text on edges. */
  bool quiet_mode = false;         /**< Hide TF / parameter / rosout noise. */
  bool hide_leaf_channels = false; /**< Hide channels with <2 unique endpoints. */
  bool hide_dead_end_channels = false; /**< Hide writer-only channels. */
  bool probe_enabled = false;      /**< Short-subscribe visible channels for Hz. */
  VertexArrangeMode channel_arrange = VertexArrangeMode::kGrid;  /**< Channel layout. */
  VertexArrangeMode service_arrange = VertexArrangeMode::kGrid;  /**< Service layout. */
  QString filter;                  /**< Substring filter applied to vertex labels. */
  QString prefix_filter;           /**< Optional channel-name prefix filter. */
};

/**
 * @brief Returns a @ref ChannelGraphPanelConfig with built-in defaults.
 *
 * @return Default-constructed config (same as aggregate defaults).
 */
ChannelGraphPanelConfig DefaultChannelGraphPanelConfig();

/**
 * @class ChannelGraphPanel
 * @brief Toolbar + status strip around @ref ChannelGraphView.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────────────┐
 * │ fit refresh channels services neighbors filter │ split … │  title bar
 * ├──────────────────────────────────────────────────┤
 * │                                                  │
 * │              ChannelGraphView                    │
 * │                                                  │
 * ├──────────────────────────────────────────────────┤
 * │ status                                           │
 * └──────────────────────────────────────────────────┘
 * @endcode
 *
 * ## Dock integration
 *
 * @ref installTitleBarTools() adds split / expand / remove / change-panel
 * buttons to a @ref PanelDockWidget title bar; corresponding signals are
 * forwarded for VisualizationFrame to handle.
 *
 * ## Refresh
 *
 * @ref refreshGraph() rebuilds topology; when auto-refresh is on,
 * @c refresh_timer_ compares @c last_topology_hash_ and refreshes only on
 * change. Filter edits are debounced via @c filter_timer_.
 *
 * @see ChannelGraphView
 * @see TfTreePanel
 */
class ChannelGraphPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel, builds UI, and performs an initial refresh.
   *
   * @param manager Non-owning visualization manager for topology sources.
   * @param parent Qt parent (typically a panel dock content host).
   */
  explicit ChannelGraphPanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);
  ~ChannelGraphPanel() override;

  /**
   * @brief Adds split / expand / remove / change-panel tools to @p dock's
   *        title bar and wires their signals.
   *
   * @param dock Host dock widget; must outlive these connections.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Syncs the expand tool button checked state with the dock.
   *
   * @param checked Whether the pane is currently expanded.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Captures current toolbar / view options into a config struct.
   *
   * @return Snapshot suitable for session persistence or cloning.
   * @see setConfig()
   */
  ChannelGraphPanelConfig config() const;

  /**
   * @brief Applies @p config to members, UI controls, and the graph view.
   *
   * Emits @ref configChanged() when values differ from the previous config.
   *
   * @param config Options to apply.
   * @see applyConfigToUi()
   */
  void setConfig(const ChannelGraphPanelConfig& config);

  /**
   * @brief Copies options from @p config without emitting @ref configChanged().
   *
   * Used when duplicating a pane so the frame can assign identity separately.
   *
   * @param config Source options.
   */
  void cloneConfigFrom(const ChannelGraphPanelConfig& config);

 public slots:
  /**
   * @brief Rebuilds the topology graph and pushes it into @c graph_view_.
   *
   * Honors filter / show-services / show-channels from @c config_.
   */
  void refreshGraph();

 signals:
  /**
   * @brief Emitted when the panel receives focus (for active-pane tracking).
   */
  void activated();

  /**
   * @brief Emitted when toolbar options that belong in session config change.
   */
  void configChanged();

  /**
   * @brief Requests splitting this pane horizontally or vertically.
   *
   * @param orientation Desired split orientation.
   */
  void panelSplitRequested(Qt::Orientation orientation);

  /**
   * @brief Requests expanding this pane to fill the dock area.
   */
  void panelExpandRequested();

  /**
   * @brief Requests removing this pane from the layout.
   */
  void panelRemoveRequested();

  /**
   * @brief Requests replacing this pane with another panel type.
   *
   * @param object_name Target panel object name / id.
   */
  void panelChangeRequested(const QString& object_name);

  /**
   * @brief Requests opening @p channel in the Raw Messages panel.
   */
  void openInRawMessagesRequested(const QString& channel);

  /**
   * @brief Requests adding @p channel / @p field_path to the active Plot panel.
   */
  void addToPlotRequested(const QString& channel, const QString& field_path);

  /**
   * @brief Requests opening @p channel / @p field_path in a Table panel.
   */
  void openInTableRequested(const QString& channel, const QString& field_path);

 protected:
  /**
   * @brief Emits @ref activated() when the panel gains keyboard focus.
   *
   * @param event Focus event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Starts auto-refresh timer when shown (if enabled).
   *
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Stops auto-refresh timer when hidden.
   *
   * @param event Hide event.
   */
  void hideEvent(QHideEvent* event) override;

 private slots:
  /**
   * @brief Debounced filter text change — schedules a refresh.
   *
   * @param text New filter string.
   */
  void onFilterChanged(const QString& text);

  /**
   * @brief Namespace menu selection.
   * @param prefix Empty selects every namespace.
   */
  void onPrefixChanged(const QString& prefix);

  /**
   * @brief Show-services toggle.
   * @param enabled New checked state.
   */
  void onShowServicesToggled(bool enabled);

  /**
   * @brief Show-channels toggle.
   * @param enabled New checked state.
   */
  void onShowChannelsToggled(bool enabled);

  /**
   * @brief Channel layout changed.
   * @param mode Column or grid.
   */
  void onChannelArrangeChanged(VertexArrangeMode mode);

  /**
   * @brief Service layout changed.
   * @param mode Column or grid.
   */
  void onServiceArrangeChanged(VertexArrangeMode mode);

  /**
   * @brief Neighborhood-mode toggle.
   * @param enabled New checked state.
   */
  void onNeighborhoodToggled(bool enabled);

  /**
   * @brief Edge-labels toggle.
   * @param enabled New checked state.
   */
  void onShowEdgeLabelsToggled(bool enabled);

  /**
   * @brief Quiet-mode (TF / parameter / rosout) toggle.
   * @param enabled New checked state.
   */
  void onQuietModeToggled(bool enabled);

  /**
   * @brief Hide-leaf-channels toggle.
   * @param enabled New checked state.
   */
  void onHideLeafToggled(bool enabled);

  /**
   * @brief Hide-dead-end-channels toggle.
   * @param enabled New checked state.
   */
  void onHideDeadEndToggled(bool enabled);

  /**
   * @brief Probe-Hz toggle for visible channel vertices.
   * @param enabled New checked state.
   */
  void onProbeToggled(bool enabled);

  /**
   * @brief Auto-refresh toggle; starts/stops @c refresh_timer_.
   * @param enabled New checked state.
   */
  void onAutoRefreshToggled(bool enabled);

  /**
   * @brief Manual refresh button — calls @ref refreshGraph().
   */
  void onRefreshClicked();

  /**
   * @brief Zoom-fit button — forwards to @ref ChannelGraphView::zoomToFit().
   */
  void onZoomFitClicked();

  /**
   * @brief Updates the status label after the view finishes rendering.
   *
   * @param vertex_count Vertices in the scene.
   * @param edge_count Edges in the scene.
   */
  void onGraphRendered(int vertex_count, int edge_count);

  /**
   * @brief Forwards a double-clicked vertex to the properties strip.
   *
   * @param vertex_id Stable topology id.
   * @param kind Vertex kind.
   * @param label Display name.
   * @param detail Extra detail string.
   */
  void onVertexDoubleClicked(const QString& vertex_id,
                             integration::GraphVertexKind kind,
                             const QString& label, const QString& detail);

  /**
   * @brief Builds Copy / Open Raw / Plot / Table context menu for a vertex.
   */
  void onVertexContextMenuRequested(const QString& vertex_id,
                                    integration::GraphVertexKind kind,
                                    const QString& label, const QString& detail,
                                    const QPoint& global_pos);

  /**
   * @brief Updates the footer status strip for the current selection.
   */
  void onVertexSelectionChanged(bool has_selection, const QString& vertex_id,
                                integration::GraphVertexKind kind,
                                const QString& label, const QString& detail);

 private:
  /**
   * @brief Builds toolbar, graph view, status label, and signal wiring.
   */
  void setupUi();

  /**
   * @brief Pushes @c config_ into toolbar widgets without rebuilding the graph.
   */
  void applyConfigToUi();

  /**
   * @brief Repopulates the prefix combo from known channel name prefixes.
   */
  void rebuildPrefixCombo();

  /**
   * @brief Sets @c status_label_ text.
   *
   * @param text Status string (vertex/edge counts, errors, …).
   */
  void updateStatusText(const QString& text);

  /**
   * @brief Pushes arrange / neighborhood / edge-label options into the view.
   */
  void applyViewOptions();

  /**
   * @brief Shows a small properties readout for the double-clicked vertex.
   *
   * @param kind Vertex kind.
   * @param label Display name.
   * @param detail Extra detail string.
   */
  void showVertexProperties(integration::GraphVertexKind kind,
                            const QString& label, const QString& detail);

  /**
   * @brief Builds a one-line status summary for the selected vertex.
   */
  QString formatSelectionSummary(integration::GraphVertexKind kind,
                                 const QString& label,
                                 const QString& detail) const;

  /**
   * @brief Short-subscribes visible channel vertices when probe is enabled.
   */
  void syncStatsProbes();

  /**
   * @brief Drops all probe subscriptions owned by this panel.
   */
  void clearStatsProbes();

  /** Non-owning; topology / channel data source. */
  common::VisualizationManager* manager_ = nullptr;

  /** Current UI / view options. */
  ChannelGraphPanelConfig config_;

  /** Interactive topology canvas. */
  ChannelGraphView* graph_view_ = nullptr;

  /** Title-bar control strip, installed beside split / expand. */
  QWidget* graph_tools_ = nullptr;

  /** Title-bar: toggle channel vertices. */
  QToolButton* show_channels_button_ = nullptr;

  /** Title-bar: toggle service vertices. */
  QToolButton* show_services_button_ = nullptr;

  /** Title-bar: show only the selected vertex and its neighbors. */
  QToolButton* neighborhood_button_ = nullptr;

  /** Toolbar: free-text vertex filter. */
  QLineEdit* filter_edit_ = nullptr;

  /** Footer status text. */
  QLabel* status_label_ = nullptr;

  /** Title-bar expand button (set by @ref installTitleBarTools()). */
  QToolButton* expand_button_ = nullptr;

  /** Periodic topology poll when auto-refresh is on. */
  QTimer* refresh_timer_ = nullptr;

  /** Debounce timer for filter line edits. */
  QTimer* filter_timer_ = nullptr;

  /** Hash of last applied topology; skip refresh when unchanged. */
  QString last_topology_hash_;

  /** Footer text shown when nothing is selected. */
  QString idle_status_text_;

  /** Currently selected vertex (for live status refresh). */
  QString selected_vertex_id_;
  integration::GraphVertexKind selected_vertex_kind_ =
      integration::GraphVertexKind::kNode;
  QString selected_vertex_label_;
  QString selected_vertex_detail_;

  /** Active probe subscriptions keyed by channel name. */
  std::unordered_map<std::string, integration::ChannelReaderRegistry::SubscriptionId>
      probe_subscriptions_;
};

}  // namespace channel_graph
}  // namespace autoviz
