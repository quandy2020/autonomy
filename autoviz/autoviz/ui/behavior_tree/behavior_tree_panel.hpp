/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"
#include "autoviz/ui/behavior_tree/bt_hook.hpp"

#include <QWidget>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

class QButtonGroup;
class QComboBox;
class QFormLayout;
class QFrame;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QSplitter;
class QTableWidget;
class QTabWidget;
class QTimer;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace behavior_tree {

class BtBlackboardView;
class BtGraphView;
class BtNodePalette;

enum class BtPanelMode {
  kEditor,
  kMonitor,
  kReplay,
};

struct ReplayEvent {
  qint64 timestamp_ns = 0;
  QString node_name;
  BtNodeStatus previous = BtNodeStatus::kIdle;
  BtNodeStatus status = BtNodeStatus::kIdle;
};

/** BehaviorTree.CPP editor, live monitor, and replay panel. */
class BehaviorTreePanel : public QWidget {
  Q_OBJECT

 public:
  explicit BehaviorTreePanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);
  ~BehaviorTreePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

  bool loadFile(const QString& path);
  bool saveFile();
  bool saveAs();

  BtPanelMode mode() const { return mode_; }

 public slots:
  void refreshFileTree();

 signals:
  void activated();
  void configChanged();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private slots:
  void onModeChanged(int mode_id);
  void onOpenClicked();
  void onSaveClicked();
  void onSaveAsClicked();
  void onFitClicked();
  void onLayoutClicked();
  void onLayoutHorizontalClicked();
  void onUndoClicked();
  void onRedoClicked();
  void onExportSvgClicked();
  void onImportModelsClicked();
  void onExportModelsClicked();
  void onFileTreeItemActivated(QTreeWidgetItem* item, int column);
  void onTreeTabChanged(int index);
  void onGraphSelectionChanged(int uid);
  void onGraphTreeChanged();
  void onCreateSubtreeRequested(int uid);
  void onSubtreeExpandRequested(const QString& tree_id);
  void onPortRemapItemDoubleClicked(QTreeWidgetItem* item, int column);
  void onInstanceNameEdited();
  void onPortRemapChanged();
  void onMonitorConnectToggled(bool enabled);
  void onMonitorChannelEdited();
  void onReplayPlayPause();
  void onReplaySliderChanged(int value);
  void onReplayStepClicked();
  void onReplayStepBackClicked();
  void onReplayClearClicked();
  void onReplaySaveClicked();
  void onOpenReplayLogClicked();
  void onReplaySpeedChanged(int index);
  void onToggleLeftSidebar(bool visible);
  void onToggleRightPanel(bool visible);
  void onLogEventSpinChanged(int value);
  void onTransitionRowClicked(int row, int column);
  void onConfigureHookRequested(int uid);
  void onNodeEditorRequested(int uid);
  void onBreakpointListDoubleClicked(QTreeWidgetItem* item, int column);
  void onBreakpointDeleteClicked();
  void onBreakpointClearClicked();
  void onBreakpointSaveClicked();
  void onBreakpointLoadClicked();
  void onAddCustomNodeClicked();
  void onEditCustomNode(const QString& registration_id);
  void onRemoveCustomNode(const QString& registration_id);
  void onTreeTabContextMenu(const QPoint& pos);

 private:
  void setupUi();
  void applyModeUi();
  void setLeftSidebarVisible(bool visible);
  void setRightPanelVisible(bool visible);
  void applyTreesFilter(const QString& text);
  void updateStatusText(const QString& text);
  void setDocument(const BtDocument& doc, const QString& path);
  void rebuildTreeTabs();
  void syncCurrentTreeFromView();
  BtGraphView* graphViewForTreeId(const QString& tree_id) const;
  BtGraphView* currentGraphView() const;
  QString currentTreeId() const;
  void updatePropertyPanel(int uid);
  void updateUndoRedoButtons();
  void updateValidTreeIndicator();
  void subscribeMonitor();
  void unsubscribeMonitor();
  void onMonitorPayload(const std::string& payload);
  void processMonitorLog(const QByteArray& protobuf_bytes);
  void applyStatusChange(const QString& node_name, BtNodeStatus status,
                         std::optional<BtNodeStatus> previous = std::nullopt);
  int uidForNodeName(const QString& node_name) const;
  int uidForNodeNameInTree(const BtAbsTree& tree, const QString& node_name) const;
  void appendTransitionRow(int event_index);
  void openHookDialogForUid(int uid);
  void upsertHook(const BtHook& hook);
  void removeHookByName(const QString& node_name);
  void rebuildBreakpointList();
  void syncHookMarkersToViews();
  bool handleHookOnStatus(const QString& node_name, BtNodeStatus status);
  void appendReplayEvent(const ReplayEvent& event);
  void applyReplayUpTo(int index);
  void setReplayPlaying(bool playing);
  void rebuildTransitionsTable();
  void syncTransitionsSelection(int index);
  void updateReplayTimeLabel();
  void updateLogFooter();
  void updateReplayTimerInterval();
  void markDirty(bool dirty = true);
  void updateTabTitles();
  QString treeIdAtTab(int index) const;
  bool confirmDiscardChanges();
  void wireGraphView(BtGraphView* view);
  void syncModelsToViews();
  void addCustomModel(const BtNodeModel& model);
  void replaceCustomModel(const QString& old_id, const BtNodeModel& new_model);
  void removeCustomModel(const QString& model_id);
  void onTreeNodeModelEdited(const QString& old_id, const QString& new_id);
  void destroySubTreeTab(const QString& tree_id);
  void renameTreeTab(int tab_index, const QString& new_name);
  void setMainTreeTab(int tab_index);

  common::VisualizationManager* manager_ = nullptr;
  BtDocument document_;
  bool dirty_ = false;
  BtPanelMode mode_ = BtPanelMode::kEditor;

  QFrame* mode_rail_ = nullptr;
  QFrame* toolbar_frame_ = nullptr;
  QButtonGroup* mode_group_ = nullptr;
  QToolButton* editor_mode_button_ = nullptr;
  QToolButton* monitor_mode_button_ = nullptr;
  QToolButton* replay_mode_button_ = nullptr;
  QToolButton* open_button_ = nullptr;
  QToolButton* save_button_ = nullptr;
  QToolButton* save_as_button_ = nullptr;
  QToolButton* fit_button_ = nullptr;
  QToolButton* layout_button_ = nullptr;
  QToolButton* layout_h_button_ = nullptr;
  QToolButton* undo_button_ = nullptr;
  QToolButton* redo_button_ = nullptr;
  QToolButton* export_svg_button_ = nullptr;
  QFrame* editor_tools_ = nullptr;
  QFrame* monitor_tools_ = nullptr;
  QToolButton* monitor_connect_button_ = nullptr;
  QToolButton* monitor_follow_button_ = nullptr;
  QToolButton* monitor_record_button_ = nullptr;
  QLineEdit* monitor_channel_edit_ = nullptr;
  QFrame* replay_tools_ = nullptr;
  QToolButton* replay_open_button_ = nullptr;
  QToolButton* replay_save_button_ = nullptr;
  QToolButton* replay_clear_button_ = nullptr;

  QFrame* playback_bar_ = nullptr;
  QComboBox* replay_speed_combo_ = nullptr;
  QToolButton* replay_play_button_ = nullptr;
  QSlider* replay_slider_ = nullptr;
  QToolButton* replay_step_back_button_ = nullptr;
  QToolButton* replay_step_button_ = nullptr;
  QLabel* replay_time_label_ = nullptr;

  QSplitter* content_splitter_ = nullptr;
  QFrame* left_sidebar_ = nullptr;
  QToolButton* toggle_left_sidebar_button_ = nullptr;
  QToolButton* collapse_left_sidebar_button_ = nullptr;
  QLineEdit* trees_filter_edit_ = nullptr;
  QSplitter* left_splitter_ = nullptr;
  QTreeWidget* file_tree_ = nullptr;
  BtNodePalette* palette_ = nullptr;
  QFrame* breakpoints_frame_ = nullptr;
  QTreeWidget* breakpoints_list_ = nullptr;
  QToolButton* breakpoint_load_button_ = nullptr;
  QToolButton* breakpoint_save_button_ = nullptr;
  QToolButton* breakpoint_delete_button_ = nullptr;
  QToolButton* breakpoint_clear_button_ = nullptr;
  QTabWidget* tree_tabs_ = nullptr;
  QFrame* property_panel_ = nullptr;
  QToolButton* toggle_right_panel_button_ = nullptr;
  QToolButton* collapse_right_panel_button_ = nullptr;
  QFrame* editor_property_page_ = nullptr;
  QFrame* transitions_page_ = nullptr;
  QToolButton* log_open_button_ = nullptr;
  QToolButton* log_save_button_ = nullptr;
  QToolButton* log_clear_button_ = nullptr;
  QToolButton* log_collapse_button_ = nullptr;
  QLineEdit* instance_name_edit_ = nullptr;
  QTreeWidget* port_remap_tree_ = nullptr;
  BtBlackboardView* blackboard_view_ = nullptr;
  QTableWidget* transitions_table_ = nullptr;
  QLineEdit* transitions_filter_edit_ = nullptr;
  QFrame* log_footer_bar_ = nullptr;
  QSpinBox* log_event_spin_ = nullptr;
  QLabel* log_event_count_label_ = nullptr;
  QToolButton* log_play_button_ = nullptr;
  QLabel* valid_tree_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QToolButton* expand_button_ = nullptr;

  int selected_uid_ = -1;
  QString highlighted_port_value_;
  bool updating_property_panel_ = false;
  bool updating_transitions_ = false;
  std::vector<std::uint64_t> monitor_subscription_ids_;
  QString monitor_subscribed_channel_;
  QVector<ReplayEvent> replay_events_;
  int replay_index_ = 0;
  bool replay_playing_ = false;
  double replay_speed_ = 1.0;
  QTimer* replay_timer_ = nullptr;
  BtHookMap hooks_;
  bool handling_breakpoint_ = false;
};

}  // namespace behavior_tree
}  // namespace autoviz
