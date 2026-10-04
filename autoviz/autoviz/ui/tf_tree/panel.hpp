/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief TF Tree panel — coalesced frame statistics tree plus graph tab.
 *
 * Subscribes to @ref transform::Buffer change signals, rate-limits rebuilds,
 * and shows per-frame details (parent, authority, age, counts) alongside an
 * optional @ref TfTreeGraphView.
 *
 * @see TfTreeGraphView
 * @see transform::Buffer
 * @see PanelDockWidget
 */

#pragma once

#include <QHash>
#include <QPointer>
#include <QString>
#include <QWidget>

#include "autoviz/common/session_config.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/tf2/signal.hpp"

class QDoubleSpinBox;
class QLineEdit;
class QLabel;
class QTimer;
class QToolButton;
class QTabWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QPoint;

namespace autoviz {

class PanelDockWidget;
class TfTreeGraphView;
namespace common {
class VisualizationManager;
}

/**
 * @class TfTreePanel
 * @brief Dockable TF introspection panel (tree + graph + detail strip).
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────────┐
 * │ Filter [________]  Stale  max-age  N frames  │
 * ├───────────────────┬──────────────────────────┤
 * │ Tree │ Graph tabs │  Detail: parent, age, …  │
 * └───────────────────┴──────────────────────────┘
 * @endcode
 *
 * ## Refresh strategy
 *
 * TF can update at high rate. @ref refresh() only sets @c refresh_pending_;
 * @c refresh_timer_ coalesces into @ref onCoalescedRefresh(), which either
 * rebuilds (@ref rebuildTree()) when @ref structureFingerprint() changes or
 * updates stats in place (@ref updateStatsInPlace()).
 *
 * @ref setPaused() stops coalesced rebuilds while the app is backgrounded.
 *
 * @note Disconnects the TF VoidSignal connection in the destructor.
 *
 * @see TfTreeGraphView
 * @see ChannelGraphPanel
 */
class TfTreePanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel, wires TF change signals, and builds UI.
   *
   * @param tf_buffer Non-owning TF buffer providing frame statistics.
   * @param manager Non-owning visualization manager (clock / fixed frame).
   * @param parent Qt parent widget.
   */
  TfTreePanel(transform::Buffer* tf_buffer, common::VisualizationManager* manager,
              QWidget* parent = nullptr);

  /**
   * @brief Disconnects TF signals and tears down timers.
   */
  ~TfTreePanel() override;

  /**
   * @brief Adds split / expand / remove / change-panel tools to @p dock's
   *        title bar.
   *
   * @param dock Host dock widget.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Syncs the expand tool button checked state with the dock.
   *
   * @param checked Whether the pane is currently expanded.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Snapshot of filter / tab / stale options for session persistence.
   *
   * @return Persist record (object_name left empty; filled by capture).
   */
  common::TfTreePanelPersistConfig config() const;

  /**
   * @brief Applies persist state without emitting @ref configChanged().
   *
   * @param config Session record to restore.
   */
  void setConfig(const common::TfTreePanelPersistConfig& config);

 public slots:
  /**
   * @brief Request a refresh; coalesced + rate-limited (never rebuilds every
   *        TF tick).
   *
   * @see scheduleRefresh()
   * @see onCoalescedRefresh()
   */
  void refresh();

  /**
   * @brief Stop coalesced rebuilds while the app is in the background.
   *
   * @param paused @c true to ignore pending refreshes; @c false to resume.
   */
  void setPaused(bool paused);

 signals:
  /**
   * @brief Emitted when the panel receives focus (active-pane tracking).
   */
  void activated();

  /**
   * @brief Emitted when filter / tab / stale options change.
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

 protected:
  /**
   * @brief Emits @ref activated() when the panel gains keyboard focus.
   *
   * @param event Focus event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Triggers a refresh when the panel becomes visible.
   *
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

 private slots:
  /**
   * @brief Tree selection changed: updates detail strip and graph highlight.
   */
  void onFrameSelectionChanged();

  /**
   * @brief Filter text changed: reapplies visibility / rebuilds graph.
   *
   * @param text Substring filter.
   */
  void onFilterChanged(const QString& text);

  /**
   * @brief Stale-only toggle changed.
   *
   * @param checked Whether Stale-only is enabled.
   */
  void onStaleOnlyToggled(bool checked);

  /**
   * @brief Max-age threshold changed.
   *
   * @param value Threshold in seconds.
   */
  void onMaxAgeChanged(double value);

  /**
   * @brief Active tab changed: persist index.
   *
   * @param index Tab index.
   */
  void onTabChanged(int index);

  /**
   * @brief Tree context menu: copy frame id / path.
   *
   * @param pos Local position in the tree viewport.
   */
  void onTreeContextMenu(const QPoint& pos);

  /**
   * @brief Timer slot performing the coalesced tree/graph update.
   */
  void onCoalescedRefresh();

 private:
  /**
   * @struct FrameNode
   * @brief Cached tree item + stats for one TF frame id.
   */
  struct FrameNode {
    transform::TfFrameStats stats;   /**< Last known statistics for this frame. */
    QTreeWidgetItem* item = nullptr; /**< Row in @c tree_; may be stale after rebuild. */
  };

  /**
   * @brief Builds filter, tabs, tree, graph, detail strip, and wiring.
   */
  void setupUi();

  /**
   * @brief Arms @c refresh_timer_ if a refresh is pending and not paused.
   */
  void scheduleRefresh();

  /**
   * @brief Full rebuild of the tree hierarchy and graph scene.
   */
  void rebuildTree();

  /**
   * @brief Updates per-row texts from @p frames without recreating items.
   *
   * @param frames Latest TF frame statistics.
   */
  void updateStatsInPlace(
      const std::vector<transform::TfFrameStats>& frames);

  /**
   * @brief Fills the detail strip from the selected tree item.
   *
   * @param item Selected frame row, or @c nullptr to clear.
   */
  void updateDetailsForItem(QTreeWidgetItem* item);

  /**
   * @brief Updates @c summary_label_ with frame / tree counts.
   *
   * @param frame_count Number of frames.
   * @param tree_count Number of disconnected TF trees (roots).
   */
  void updateSummaryLabel(int frame_count, int tree_count);

  /**
   * @brief Stable fingerprint of parent/child structure (not rates/ages).
   *
   * @param frames Frame statistics to hash.
   * @return Fingerprint string compared against @c structure_fingerprint_.
   */
  QString structureFingerprint(
      const std::vector<transform::TfFrameStats>& frames) const;

  /**
   * @brief Whether @p stats passes the current text + stale filters.
   *
   * @param frame_id Frame id.
   * @param parent_id Normalized parent id.
   * @param stats Frame statistics (for age).
   * @param now_sec Current clock used for age.
   * @return @c true when the frame should be shown.
   */
  bool framePassesFilters(const QString& frame_id, const QString& parent_id,
                          const transform::TfFrameStats& stats,
                          double now_sec) const;

  /**
   * @brief Age of @p stats relative to @p now_sec, or negative if unknown.
   *
   * @param stats Frame statistics.
   * @param now_sec Current clock.
   * @return Age in seconds, or a negative value when not computable.
   */
  static double frameAgeSec(const transform::TfFrameStats& stats,
                            double now_sec);

  /**
   * @brief Builds root→leaf path for @p item using tree ancestry.
   *
   * @param item Tree row.
   * @return Slash-separated path (e.g. @c map/odom/base_link).
   */
  static QString framePathForItem(const QTreeWidgetItem* item);

  /**
   * @brief Formats an absolute timestamp for the detail strip.
   *
   * @param sec Time in seconds.
   * @return Display string.
   */
  QString formatTimestampSec(double sec) const;

  /**
   * @brief Formats a transform age for the detail strip.
   *
   * @param age_sec Age in seconds.
   * @return Display string (e.g. @c "0.12 s").
   */
  QString formatAgeSec(double age_sec) const;

  /**
   * @brief Current time from the manager clock (or wall) for age computation.
   *
   * @return Time in seconds.
   */
  double currentTimeSec() const;

  /**
   * @brief Emits @ref configChanged() after interactive option changes.
   */
  void emitConfigChanged();

  /** Non-owning TF buffer. */
  transform::Buffer* tf_buffer_ = nullptr;

  /** Non-owning visualization manager (clock / fixed frame). */
  common::VisualizationManager* manager_ = nullptr;

  /** Substring filter for frame ids. */
  QLineEdit* filter_edit_ = nullptr;

  /** Toggle: only show frames older than @c max_age_spin_. */
  QToolButton* stale_only_button_ = nullptr;

  /** Age threshold (seconds) for @c stale_only_button_. */
  QDoubleSpinBox* max_age_spin_ = nullptr;

  /** Summary chip: frame count / tree count. */
  QLabel* summary_label_ = nullptr;

  /** Tabs hosting the tree and graph views. */
  QTabWidget* view_tabs_ = nullptr;

  /** Hierarchical frame statistics tree. */
  QTreeWidget* tree_ = nullptr;

  /** Dot-style TF graph canvas. */
  TfTreeGraphView* graph_view_ = nullptr;

  /** Detail strip title (selected frame id). */
  QLabel* detail_title_ = nullptr;

  /** Detail strip hint when nothing is selected. */
  QLabel* detail_hint_ = nullptr;

  /** Detail strip body container. */
  QWidget* detail_body_ = nullptr;

  /** Detail: parent frame value. */
  QLabel* detail_parent_value_ = nullptr;

  /** Detail: transform type value. */
  QLabel* detail_type_value_ = nullptr;

  /** Detail: authority value. */
  QLabel* detail_authority_value_ = nullptr;

  /** Detail: last transform time value. */
  QLabel* detail_last_time_value_ = nullptr;

  /** Detail: transform age value. */
  QLabel* detail_age_value_ = nullptr;

  /** Detail: transform count value. */
  QLabel* detail_count_value_ = nullptr;

  /** Title-bar expand button (weak; dock may destroy it). */
  QPointer<QToolButton> expand_button_;

  /** Coalesce / rate-limit timer for @ref onCoalescedRefresh(). */
  QTimer* refresh_timer_ = nullptr;

  /** Connection to Buffer transforms-changed signal. */
  transform::tf2::VoidSignal::Connection transforms_changed_connection_;

  /** Frame id → cached tree node / stats. */
  QHash<QString, FrameNode> frame_nodes_;

  /** Currently selected frame id (survives rebuilds). */
  QString selected_frame_id_;

  /** Last structure fingerprint; skip full rebuild when equal. */
  QString structure_fingerprint_;

  /** Set by @ref refresh(); cleared by @ref onCoalescedRefresh(). */
  bool refresh_pending_ = false;

  /** When true, next coalesced refresh always rebuilds the tree. */
  bool force_rebuild_ = false;

  /** When true, coalesced refreshes are skipped (app backgrounded). */
  bool paused_ = false;

  /** Suppress @ref configChanged() while applying @ref setConfig(). */
  bool applying_config_ = false;
};

}  // namespace autoviz
