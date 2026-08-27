/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/behavior_tree_panel.hpp"

#include <algorithm>
#include <functional>

#include <QAbstractItemView>
#include <QButtonGroup>
#include <QColor>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFocusEvent>
#include <QFormLayout>
#include <QFrame>
#include <QHash>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QSet>
#include <QShowEvent>
#include <QSize>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QXmlStreamReader>

#include "automsgs/msgs/nav_msgs/behavior_tree.pb.h"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/behavior_tree/bt_blackboard_view.hpp"
#include "autoviz/ui/behavior_tree/bt_custom_node_dialog.hpp"
#include "autoviz/ui/behavior_tree/bt_graph_view.hpp"
#include "autoviz/ui/behavior_tree/bt_groot_style.hpp"
#include "autoviz/ui/behavior_tree/bt_hook.hpp"
#include "autoviz/ui/behavior_tree/bt_hook_dialog.hpp"
#include "autoviz/ui/behavior_tree/bt_icon_loader.hpp"
#include "autoviz/ui/behavior_tree/bt_manifest_loader.hpp"
#include "autoviz/ui/behavior_tree/bt_node_editor_dialog.hpp"
#include "autoviz/ui/behavior_tree/bt_node_palette.hpp"
#include "autoviz/ui/behavior_tree/bt_xml_io.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr char kBg[] = "#f8f9fb";
constexpr char kSurface[] = "#ffffff";
constexpr char kBorder[] = "#cbd5e1";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";

QString ModeRailStyle() {
  return QStringLiteral(
      "QFrame#BtModeRail {"
      "  background: #1f2933; border-right: 1px solid #111827;"
      "}"
      "QToolButton {"
      "  border: none; background: transparent; color: #86efac;"
      "  border-radius: 8px; padding: 8px; margin: 2px 4px;"
      "  min-width: 36px; max-width: 36px; min-height: 36px; max-height: 36px;"
      "}"
      "QToolButton:checked {"
      "  background: rgba(134,239,172,0.18);"
      "}"
      "QToolButton:hover:!checked {"
      "  background: rgba(255,255,255,0.08);"
      "}");
}

QString ToolbarIconButtonStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  background: transparent; border: 1px solid transparent;"
      "  border-radius: 6px; padding: 4px; color: %1;"
      "  min-width: 28px; max-width: 28px; min-height: 28px; max-height: 28px;"
      "}"
      "QToolButton:hover {"
      "  background: rgba(8,145,178,0.10); border-color: %2;"
      "}"
      "QToolButton:pressed {"
      "  background: rgba(8,145,178,0.16);"
      "}"
      "QToolButton:checked {"
      "  background: rgba(8,145,178,0.14); border-color: %3; color: %3;"
      "}"
      "QToolButton:disabled { color: #94a3b8; }")
      .arg(QLatin1String(kText), QLatin1String(kBorder), QLatin1String(kAccent));
}

QFrame* MakeToolbarSeparator(QWidget* parent) {
  auto* separator = new QFrame(parent);
  separator->setFrameShape(QFrame::VLine);
  separator->setFixedWidth(1);
  separator->setStyleSheet(
      QStringLiteral("color: %1; margin: 4px 2px;").arg(QLatin1String(kBorder)));
  return separator;
}

QToolButton* MakeIconToolButton(QWidget* parent, const QIcon& icon, const QString& tooltip,
                                bool checkable = false) {
  auto* button = new QToolButton(parent);
  button->setIcon(icon);
  button->setIconSize(QSize(20, 20));
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setAutoRaise(true);
  button->setToolTip(tooltip);
  button->setCheckable(checkable);
  button->setStyleSheet(ToolbarIconButtonStyle());
  return button;
}

QToolButton* MakeModeButton(QWidget* parent, const QString& tip, const QIcon& icon) {
  auto* button = new QToolButton(parent);
  button->setIcon(icon);
  button->setIconSize(QSize(20, 20));
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setCheckable(true);
  button->setToolTip(tip);
  button->setAutoRaise(true);
  return button;
}

QIcon GrootIcon(const QString& relative_path, int size = 20) {
  if (relative_path.endsWith(QLatin1String(".svg"), Qt::CaseInsensitive)) {
    return BtIconLoader::toolbarIcon(relative_path, size);
  }
  return BtIconLoader::icon(BtIconLoader::grootResource(relative_path), size);
}

QIcon MirroredGrootIcon(const QString& relative_path, int size = 20) {
  const QPixmap source =
      BtIconLoader::toolbarIconFromResource(BtIconLoader::grootResource(relative_path), size)
          .pixmap(size, size);
  QPixmap mirrored(source.size());
  mirrored.fill(Qt::transparent);
  QPainter painter(&mirrored);
  painter.scale(-1.0, 1.0);
  painter.drawPixmap(-source.width(), 0, source);
  return QIcon(mirrored);
}

QString PeekMainTreeId(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return {};
  }
  QXmlStreamReader reader(&file);
  QString main_tree;
  QString first_tree;
  while (!reader.atEnd()) {
    if (reader.readNext() != QXmlStreamReader::StartElement) {
      continue;
    }
    if (reader.name() == QLatin1String("root")) {
      main_tree = reader.attributes().value(QLatin1String("main_tree_to_execute")).toString();
    } else if (reader.name() == QLatin1String("BehaviorTree")) {
      const QString id = reader.attributes().value(QLatin1String("ID")).toString();
      if (first_tree.isEmpty() && !id.isEmpty()) {
        first_tree = id;
      }
      if (!main_tree.isEmpty()) {
        break;
      }
    }
  }
  return !main_tree.isEmpty() ? main_tree : first_tree;
}

}  // namespace

BehaviorTreePanel::BehaviorTreePanel(common::VisualizationManager* manager,
                                       QWidget* parent)
    : manager_(manager), QWidget(parent) {
  Q_UNUSED(manager_);
  setFocusPolicy(Qt::StrongFocus);
  setupUi();
  refreshFileTree();
  document_.models = LoadDefaultManifests();
  palette_->setModels(document_.models);
  applyModeUi();
  updateStatusText(tr("No file loaded"));
}

BehaviorTreePanel::~BehaviorTreePanel() {
  unsubscribeMonitor();
}

void BehaviorTreePanel::setupUi() {
  ApplyPanelShell(this);

  auto* root = new QHBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  // Groot2 far-left mode rail
  mode_rail_ = new QFrame(this);
  mode_rail_->setObjectName(QStringLiteral("BtModeRail"));
  mode_rail_->setFixedWidth(48);
  mode_rail_->setStyleSheet(ModeRailStyle());
  auto* rail_layout = new QVBoxLayout(mode_rail_);
  rail_layout->setContentsMargins(0, 8, 0, 8);
  rail_layout->setSpacing(4);

  mode_group_ = new QButtonGroup(this);
  mode_group_->setExclusive(true);
  editor_mode_button_ =
      MakeModeButton(mode_rail_, tr("Editor"), BtIconLoader::toolbarIcon(QStringLiteral("BT-edit.png"), 20));
  monitor_mode_button_ =
      MakeModeButton(mode_rail_, tr("Monitor"), BtIconLoader::toolbarIcon(QStringLiteral("BT-monitor.png"), 20));
  replay_mode_button_ =
      MakeModeButton(mode_rail_, tr("Replay"), BtIconLoader::toolbarIcon(QStringLiteral("BT-log.png"), 20));
  editor_mode_button_->setChecked(true);
  mode_group_->addButton(editor_mode_button_, static_cast<int>(BtPanelMode::kEditor));
  mode_group_->addButton(monitor_mode_button_, static_cast<int>(BtPanelMode::kMonitor));
  mode_group_->addButton(replay_mode_button_, static_cast<int>(BtPanelMode::kReplay));
  rail_layout->addWidget(editor_mode_button_, 0, Qt::AlignHCenter);
  rail_layout->addWidget(monitor_mode_button_, 0, Qt::AlignHCenter);
  rail_layout->addWidget(replay_mode_button_, 0, Qt::AlignHCenter);
  rail_layout->addStretch(1);
  root->addWidget(mode_rail_);

  auto* main_column = new QWidget(this);
  auto* main_layout = new QVBoxLayout(main_column);
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  toolbar_frame_ = new QFrame(main_column);
  ApplyPanelToolbarChrome(toolbar_frame_);
  toolbar_frame_->setStyleSheet(QStringLiteral(
      "QFrame#AutovizPanelToolbar {"
      "  background: #f1f5f9; border-bottom: 1px solid #cbd5e1;"
      "}"));
  auto* toolbar_layout = new QHBoxLayout(toolbar_frame_);
  toolbar_layout->setContentsMargins(8, 4, 8, 4);
  toolbar_layout->setSpacing(4);

  editor_tools_ = new QFrame(toolbar_frame_);
  auto* editor_layout = new QHBoxLayout(editor_tools_);
  editor_layout->setContentsMargins(0, 0, 0, 0);
  editor_layout->setSpacing(2);

  fit_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("svg/zoom_home.svg")),
                                   tr("Fit graph to view"));
  layout_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("BT-vertical.png")),
                                      tr("Vertical layout"));
  layout_h_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("BT-horizontal.png")),
                                        tr("Horizontal layout"));
  editor_layout->addWidget(fit_button_);
  editor_layout->addWidget(layout_button_);
  editor_layout->addWidget(layout_h_button_);
  editor_layout->addWidget(MakeToolbarSeparator(editor_tools_));

  undo_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("svg/edit_list.svg")),
                                    tr("Undo"));
  redo_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("svg/arrow_right.svg")),
                                    tr("Redo"));
  editor_layout->addWidget(undo_button_);
  editor_layout->addWidget(redo_button_);
  editor_layout->addWidget(MakeToolbarSeparator(editor_tools_));

  export_svg_button_ = MakeIconToolButton(editor_tools_, GrootIcon(QStringLiteral("svg/tree.svg")),
                                          tr("Export SVG"));
  editor_layout->addWidget(export_svg_button_);
  editor_layout->addStretch(1);

  monitor_tools_ = new QFrame(toolbar_frame_);
  auto* monitor_layout = new QHBoxLayout(monitor_tools_);
  monitor_layout->setContentsMargins(0, 0, 0, 0);
  monitor_layout->setSpacing(6);
  monitor_connect_button_ =
      MakeIconToolButton(monitor_tools_, GrootIcon(QStringLiteral("svg/connect.svg")),
                         tr("Subscribe to behavior tree log"), true);
  monitor_follow_button_ =
      MakeIconToolButton(monitor_tools_, GrootIcon(QStringLiteral("svg/target_radar.svg")),
                         tr("Follow live status updates"), true);
  monitor_follow_button_->setChecked(true);
  monitor_record_button_ =
      MakeIconToolButton(monitor_tools_, GrootIcon(QStringLiteral("svg/repeat.svg")),
                         tr("Log recording — append live events to replay buffer"), true);
  monitor_record_button_->setChecked(true);
  auto* channel_label = new QLabel(tr("Channel"), monitor_tools_);
  channel_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px; font-weight: 600; background: transparent;")
          .arg(QLatin1String(kTextMuted)));
  monitor_channel_edit_ = new QLineEdit(monitor_tools_);
  monitor_channel_edit_->setPlaceholderText(tr("/behavior_tree/log"));
  monitor_channel_edit_->setText(QStringLiteral("/behavior_tree/log"));
  monitor_channel_edit_->setMinimumWidth(180);
  StyleFilterLineEdit(monitor_channel_edit_);
  monitor_layout->addWidget(monitor_connect_button_);
  monitor_layout->addWidget(monitor_follow_button_);
  monitor_layout->addWidget(monitor_record_button_);
  monitor_layout->addWidget(channel_label);
  monitor_layout->addWidget(monitor_channel_edit_, 1);

  replay_tools_ = new QFrame(toolbar_frame_);
  auto* replay_layout = new QHBoxLayout(replay_tools_);
  replay_layout->setContentsMargins(0, 0, 0, 0);
  replay_layout->setSpacing(4);
  replay_open_button_ =
      MakeIconToolButton(replay_tools_, GrootIcon(QStringLiteral("svg/download.svg")),
                         tr("Open replay log"));
  replay_save_button_ = MakeIconToolButton(replay_tools_, GrootIcon(QStringLiteral("svg/upload.svg")),
                                           tr("Save replay log"));
  replay_clear_button_ = MakeIconToolButton(replay_tools_, GrootIcon(QStringLiteral("close_x.png")),
                                            tr("Clear replay buffer"));
  replay_layout->addWidget(replay_open_button_);
  replay_layout->addWidget(replay_save_button_);
  replay_layout->addWidget(replay_clear_button_);
  replay_layout->addStretch(1);

  toolbar_layout->addWidget(editor_tools_, 1);
  toolbar_layout->addWidget(monitor_tools_, 1);
  toolbar_layout->addWidget(replay_tools_, 1);

  // Panel visibility toggles (Groot2-style show/hide sidebars).
  toolbar_layout->addWidget(MakeToolbarSeparator(toolbar_frame_));
  toggle_left_sidebar_button_ = MakeIconToolButton(
      toolbar_frame_, IconLoader::load(QStringLiteral(":/autoviz/icons/sidebar_left.svg")),
      tr("Hide left sidebar"), true);
  toggle_left_sidebar_button_->setChecked(true);
  toggle_right_panel_button_ = MakeIconToolButton(
      toolbar_frame_, IconLoader::load(QStringLiteral(":/autoviz/icons/sidebar_right.svg")),
      tr("Hide right panel"), true);
  toggle_right_panel_button_->setChecked(true);
  toolbar_layout->addWidget(toggle_left_sidebar_button_);
  toolbar_layout->addWidget(toggle_right_panel_button_);
  main_layout->addWidget(toolbar_frame_);

  content_splitter_ = new QSplitter(Qt::Horizontal, main_column);
  auto* content_splitter = content_splitter_;

  // Groot2 left sidebar: Project Trees (top) + Models (bottom)
  left_sidebar_ = new QFrame(content_splitter);
  left_sidebar_->setMinimumWidth(240);
  left_sidebar_->setMaximumWidth(360);
  left_sidebar_->setStyleSheet(QStringLiteral(
      "QFrame#BtLeftSidebar {"
      "  background: #f3f4f6; border-right: 1px solid %1;"
      "}")
                                   .arg(QLatin1String(kBorder)));
  left_sidebar_->setObjectName(QStringLiteral("BtLeftSidebar"));
  auto* left_outer = new QVBoxLayout(left_sidebar_);
  left_outer->setContentsMargins(0, 0, 0, 0);
  left_outer->setSpacing(0);

  auto* project_header = new QFrame(left_sidebar_);
  project_header->setStyleSheet(QStringLiteral(
      "QFrame { background: #f3f4f6; border-bottom: 1px solid %1; }")
                                    .arg(QLatin1String(kBorder)));
  auto* project_header_layout = new QHBoxLayout(project_header);
  project_header_layout->setContentsMargins(8, 6, 6, 6);
  project_header_layout->setSpacing(2);
  auto* project_label = new QLabel(tr("Project:"), project_header);
  project_label->setStyleSheet(QStringLiteral("color: %1; font-weight: 700; font-size: 12px;")
                                   .arg(QLatin1String(kText)));
  open_button_ = MakeIconToolButton(project_header, GrootIcon(QStringLiteral("svg/folder.svg")),
                                    tr("Open behavior tree XML"));
  save_button_ = MakeIconToolButton(project_header, GrootIcon(QStringLiteral("svg/save_dark.svg")),
                                    tr("Save"));
  save_as_button_ =
      MakeIconToolButton(project_header, GrootIcon(QStringLiteral("svg/upload.svg")),
                         tr("Save as…"));
  collapse_left_sidebar_button_ = MakeIconToolButton(
      project_header, MirroredGrootIcon(QStringLiteral("svg/arrow_right.svg")),
      tr("Hide left sidebar"));
  project_header_layout->addWidget(project_label);
  project_header_layout->addStretch(1);
  project_header_layout->addWidget(open_button_);
  project_header_layout->addWidget(save_button_);
  project_header_layout->addWidget(save_as_button_);
  project_header_layout->addWidget(collapse_left_sidebar_button_);
  left_outer->addWidget(project_header);

  auto* trees_label = new QLabel(tr("Trees"), left_sidebar_);
  trees_label->setStyleSheet(QStringLiteral(
      "color: %1; font-weight: 700; font-size: 12px; margin: 8px 8px 2px 8px;")
                                 .arg(QLatin1String(kText)));
  left_outer->addWidget(trees_label);

  trees_filter_edit_ = new QLineEdit(left_sidebar_);
  trees_filter_edit_->setPlaceholderText(tr("Filter"));
  trees_filter_edit_->setClearButtonEnabled(true);
  StyleFilterLineEdit(trees_filter_edit_);
  trees_filter_edit_->setStyleSheet(trees_filter_edit_->styleSheet() +
                                    QStringLiteral("QLineEdit { margin: 4px 8px 4px 8px; }"));
  left_outer->addWidget(trees_filter_edit_);

  left_splitter_ = new QSplitter(Qt::Vertical, left_sidebar_);
  file_tree_ = new QTreeWidget(left_splitter_);
  file_tree_->setHeaderHidden(true);
  file_tree_->setMinimumWidth(160);
  file_tree_->setStyleSheet(QStringLiteral(
      "QTreeWidget {"
      "  background: #ffffff; border: 1px solid %1; border-radius: 0px;"
      "  margin: 0 8px 4px 8px; outline: none; font-size: 12px;"
      "}"
      "QTreeWidget::item { padding: 3px 4px; min-height: 22px; }"
      "QTreeWidget::item:selected { background: #d1fae5; color: %2; }"
      "QTreeWidget::item:hover:!selected { background: rgba(15,23,42,0.04); }")
                                .arg(QLatin1String(kBorder), QLatin1String(kText)));
  palette_ = new BtNodePalette(left_splitter_);

  breakpoints_frame_ = new QFrame(left_splitter_);
  breakpoints_frame_->setStyleSheet(QStringLiteral(
      "QFrame { background: %1; border-top: 1px solid %2; }")
                                        .arg(QLatin1String(kSurface), QLatin1String(kBorder)));
  auto* bp_layout = new QVBoxLayout(breakpoints_frame_);
  bp_layout->setContentsMargins(6, 6, 6, 6);
  bp_layout->setSpacing(4);
  auto* bp_header = new QHBoxLayout();
  auto* bp_title = new QLabel(tr("Breakpoints"), breakpoints_frame_);
  bp_title->setStyleSheet(QStringLiteral("color: %1; font-weight: 700;")
                              .arg(QLatin1String(kText)));
  breakpoint_load_button_ =
      MakeIconToolButton(breakpoints_frame_, GrootIcon(QStringLiteral("svg/folder.svg")),
                         tr("Load breakpoints"));
  breakpoint_save_button_ =
      MakeIconToolButton(breakpoints_frame_, GrootIcon(QStringLiteral("svg/save_dark.svg")),
                         tr("Save breakpoints"));
  breakpoint_delete_button_ =
      MakeIconToolButton(breakpoints_frame_, GrootIcon(QStringLiteral("close_x.png")),
                         tr("Delete selected breakpoint"));
  breakpoint_clear_button_ =
      MakeIconToolButton(breakpoints_frame_, GrootIcon(QStringLiteral("svg/asterix.svg")),
                         tr("Clear all breakpoints"));
  bp_header->addWidget(bp_title, 1);
  bp_header->addWidget(breakpoint_load_button_);
  bp_header->addWidget(breakpoint_save_button_);
  bp_header->addWidget(breakpoint_delete_button_);
  bp_header->addWidget(breakpoint_clear_button_);
  bp_layout->addLayout(bp_header);
  breakpoints_list_ = new QTreeWidget(breakpoints_frame_);
  breakpoints_list_->setHeaderHidden(true);
  breakpoints_list_->setRootIsDecorated(false);
  breakpoints_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  bp_layout->addWidget(breakpoints_list_, 1);
  breakpoints_frame_->setVisible(false);

  left_splitter_->addWidget(file_tree_);
  left_splitter_->addWidget(palette_);
  left_splitter_->addWidget(breakpoints_frame_);
  left_splitter_->setStretchFactor(0, 1);
  left_splitter_->setStretchFactor(1, 2);
  left_splitter_->setStretchFactor(2, 1);
  left_outer->addWidget(left_splitter_, 1);
  content_splitter->addWidget(left_sidebar_);

  tree_tabs_ = new QTabWidget(content_splitter);
  tree_tabs_->setTabsClosable(false);
  tree_tabs_->setDocumentMode(true);
  tree_tabs_->tabBar()->setContextMenuPolicy(Qt::CustomContextMenu);

  auto* canvas_column = new QWidget(content_splitter);
  auto* canvas_layout = new QVBoxLayout(canvas_column);
  canvas_layout->setContentsMargins(0, 0, 0, 0);
  canvas_layout->setSpacing(0);
  canvas_layout->addWidget(tree_tabs_, 1);

  // Groot2-style bottom playback bar (Replay mode)
  playback_bar_ = new QFrame(canvas_column);
  playback_bar_->setObjectName(QStringLiteral("BtPlaybackBar"));
  playback_bar_->setStyleSheet(QStringLiteral(
      "QFrame#BtPlaybackBar {"
      "  background: #f1f5f9; border-top: 1px solid %1;"
      "}"
      "QLabel { color: %2; font-size: 11px; font-weight: 600; background: transparent; }")
                                    .arg(QLatin1String(kBorder), QLatin1String(kTextMuted)));
  auto* playback_layout = new QHBoxLayout(playback_bar_);
  playback_layout->setContentsMargins(8, 6, 8, 6);
  playback_layout->setSpacing(6);

  replay_speed_combo_ = new QComboBox(playback_bar_);
  replay_speed_combo_->addItem(QStringLiteral("0.25 X"), 0.25);
  replay_speed_combo_->addItem(QStringLiteral("0.5 X"), 0.5);
  replay_speed_combo_->addItem(QStringLiteral("1.0 X"), 1.0);
  replay_speed_combo_->addItem(QStringLiteral("2.0 X"), 2.0);
  replay_speed_combo_->addItem(QStringLiteral("4.0 X"), 4.0);
  replay_speed_combo_->setCurrentIndex(2);
  replay_speed_combo_->setFixedWidth(80);

  replay_step_back_button_ =
      MakeIconToolButton(playback_bar_, MirroredGrootIcon(QStringLiteral("svg/arrow_right.svg")),
                         tr("Step backward"));
  replay_play_button_ = MakeIconToolButton(playback_bar_, GrootIcon(QStringLiteral("play.png")),
                                           tr("Play / pause replay"), true);
  replay_step_button_ =
      MakeIconToolButton(playback_bar_, GrootIcon(QStringLiteral("svg/arrow_right.svg")),
                         tr("Step forward"));
  replay_slider_ = new QSlider(Qt::Horizontal, playback_bar_);
  replay_slider_->setMinimum(0);
  replay_slider_->setMaximum(0);
  replay_slider_->setStyleSheet(QStringLiteral(
      "QSlider::groove:horizontal { height: 6px; background: #e2e8f0; border-radius: 3px; }"
      "QSlider::handle:horizontal {"
      "  width: 14px; margin: -5px 0; background: %1; border-radius: 7px;"
      "}"
      "QSlider::sub-page:horizontal { background: %1; border-radius: 3px; }")
                                   .arg(QLatin1String(kAccent)));
  replay_time_label_ = new QLabel(QStringLiteral("00.000"), playback_bar_);
  replay_time_label_->setMinimumWidth(64);
  replay_time_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  playback_layout->addWidget(replay_speed_combo_);
  playback_layout->addWidget(replay_step_back_button_);
  playback_layout->addWidget(replay_play_button_);
  playback_layout->addWidget(replay_step_button_);
  playback_layout->addWidget(replay_slider_, 1);
  playback_layout->addWidget(replay_time_label_);
  playback_bar_->setVisible(false);
  canvas_layout->addWidget(playback_bar_);
  content_splitter->addWidget(canvas_column);
  content_splitter->setStretchFactor(1, 1);

  property_panel_ = new QFrame(content_splitter);
  property_panel_->setMinimumWidth(220);
  property_panel_->setMaximumWidth(360);
  property_panel_->setStyleSheet(QStringLiteral(
      "QFrame { background: %1; border-left: 1px solid %2; }")
                                      .arg(QLatin1String(kSurface),
                                           QLatin1String(kBorder)));
  auto* property_stack_layout = new QVBoxLayout(property_panel_);
  property_stack_layout->setContentsMargins(0, 0, 0, 0);
  property_stack_layout->setSpacing(0);

  auto* property_stack = new QStackedWidget(property_panel_);
  property_stack->setObjectName(QStringLiteral("BtPropertyStack"));

  editor_property_page_ = new QFrame(property_stack);
  auto* property_layout = new QVBoxLayout(editor_property_page_);
  property_layout->setContentsMargins(8, 8, 8, 8);
  property_layout->setSpacing(8);

  auto* property_header = new QHBoxLayout();
  property_header->setContentsMargins(0, 0, 0, 0);
  property_header->setSpacing(2);
  auto* property_title = new QLabel(tr("Properties"), editor_property_page_);
  property_title->setStyleSheet(QStringLiteral("color: %1; font-weight: 700;")
                                    .arg(QLatin1String(kText)));
  collapse_right_panel_button_ = MakeIconToolButton(
      editor_property_page_, GrootIcon(QStringLiteral("svg/arrow_right.svg")),
      tr("Hide right panel"));
  property_header->addWidget(property_title, 1);
  property_header->addWidget(collapse_right_panel_button_);
  property_layout->addLayout(property_header);

  auto* instance_form = new QFormLayout();
  instance_name_edit_ = new QLineEdit(editor_property_page_);
  instance_name_edit_->setPlaceholderText(tr("Instance name"));
  StyleFilterLineEdit(instance_name_edit_);
  instance_form->addRow(tr("Name"), instance_name_edit_);
  property_layout->addLayout(instance_form);

  auto* ports_label = new QLabel(tr("Port remap"), editor_property_page_);
  ports_label->setStyleSheet(QStringLiteral("color: %1; font-weight: 600;")
                                 .arg(QLatin1String(kTextMuted)));
  property_layout->addWidget(ports_label);
  port_remap_tree_ = new QTreeWidget(editor_property_page_);
  port_remap_tree_->setHeaderLabels({tr("Port"), tr("Value")});
  port_remap_tree_->header()->setStretchLastSection(true);
  port_remap_tree_->setRootIsDecorated(false);
  port_remap_tree_->setMaximumHeight(160);
  property_layout->addWidget(port_remap_tree_);

  auto* bb_label = new QLabel(tr("Blackboard"), editor_property_page_);
  bb_label->setStyleSheet(QStringLiteral("color: %1; font-weight: 600;")
                              .arg(QLatin1String(kTextMuted)));
  property_layout->addWidget(bb_label);
  blackboard_view_ = new BtBlackboardView(editor_property_page_);
  property_layout->addWidget(blackboard_view_, 1);

  transitions_page_ = new QFrame(property_stack);
  auto* transitions_layout = new QVBoxLayout(transitions_page_);
  transitions_layout->setContentsMargins(8, 8, 8, 8);
  transitions_layout->setSpacing(6);

  auto* log_header = new QHBoxLayout();
  log_header->setContentsMargins(0, 0, 0, 0);
  log_header->setSpacing(2);
  auto* transitions_title = new QLabel(tr("Log"), transitions_page_);
  transitions_title->setStyleSheet(QStringLiteral("color: %1; font-weight: 700;")
                                       .arg(QLatin1String(kText)));
  log_open_button_ =
      MakeIconToolButton(transitions_page_, GrootIcon(QStringLiteral("svg/folder.svg")),
                         tr("Open log"));
  log_save_button_ =
      MakeIconToolButton(transitions_page_, GrootIcon(QStringLiteral("svg/save_dark.svg")),
                         tr("Save log"));
  log_clear_button_ =
      MakeIconToolButton(transitions_page_, GrootIcon(QStringLiteral("close_x.png")),
                         tr("Clear log"));
  log_collapse_button_ = MakeIconToolButton(
      transitions_page_, GrootIcon(QStringLiteral("svg/arrow_right.svg")),
      tr("Hide right panel"));
  log_header->addWidget(transitions_title, 1);
  log_header->addWidget(log_open_button_);
  log_header->addWidget(log_save_button_);
  log_header->addWidget(log_clear_button_);
  log_header->addWidget(log_collapse_button_);
  transitions_layout->addLayout(log_header);

  transitions_filter_edit_ = new QLineEdit(transitions_page_);
  transitions_filter_edit_->setPlaceholderText(tr("Filter by Node Name"));
  StyleFilterLineEdit(transitions_filter_edit_);
  transitions_layout->addWidget(transitions_filter_edit_);
  transitions_table_ = new QTableWidget(transitions_page_);
  transitions_table_->setColumnCount(4);
  transitions_table_->setHorizontalHeaderLabels(
      {tr("Time"), tr("Node Name"), tr("Previous"), tr("Status")});
  transitions_table_->horizontalHeader()->setStretchLastSection(false);
  transitions_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  transitions_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  transitions_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  transitions_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  transitions_table_->verticalHeader()->setVisible(false);
  transitions_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  transitions_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  transitions_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  transitions_table_->setAlternatingRowColors(false);
  transitions_table_->setShowGrid(true);
  transitions_table_->setStyleSheet(QStringLiteral(
      "QTableWidget { gridline-color: #d0d0d0; font-size: 12px; }"
      "QHeaderView::section { background: #f3f4f6; padding: 4px; font-weight: 600; }"));
  transitions_layout->addWidget(transitions_table_, 1);

  // Groot2 Log footer: event index spinbox + play (right).
  log_footer_bar_ = new QFrame(transitions_page_);
  log_footer_bar_->setObjectName(QStringLiteral("BtLogFooter"));
  log_footer_bar_->setStyleSheet(QStringLiteral(
      "QFrame#BtLogFooter {"
      "  background: #f3f4f6; border-top: 1px solid %1;"
      "}"
      "QFrame#BtLogFooter QLabel {"
      "  color: %2; font-size: 12px; background: transparent;"
      "  padding-left: 4px;"
      "}"
      "QFrame#BtLogFooter QSpinBox {"
      "  min-height: 26px; padding: 2px 8px 2px 10px;"
      "  border: 1px solid %1; border-radius: 4px; background: #ffffff;"
      "}"
      "QFrame#BtLogFooter QSpinBox::up-button,"
      "QFrame#BtLogFooter QSpinBox::down-button {"
      "  width: 18px; border-left: 1px solid %1;"
      "}")
                                     .arg(QLatin1String(kBorder), QLatin1String(kTextMuted)));
  auto* log_footer_layout = new QHBoxLayout(log_footer_bar_);
  log_footer_layout->setContentsMargins(10, 6, 10, 6);
  log_footer_layout->setSpacing(10);

  log_event_spin_ = new QSpinBox(log_footer_bar_);
  log_event_spin_->setMinimum(0);
  log_event_spin_->setMaximum(0);
  log_event_spin_->setMinimumWidth(96);
  log_event_spin_->setFixedHeight(28);
  log_event_spin_->setAlignment(Qt::AlignCenter);
  log_event_spin_->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
  log_event_spin_->setToolTip(tr("Current log event index"));
  log_event_count_label_ = new QLabel(tr("of 0"), log_footer_bar_);
  log_event_count_label_->setMinimumWidth(64);
  log_event_count_label_->setContentsMargins(4, 0, 0, 0);

  log_play_button_ =
      MakeIconToolButton(log_footer_bar_, GrootIcon(QStringLiteral("play.png")),
                         tr("Play / pause log replay"), true);

  log_footer_layout->addWidget(log_event_spin_, 0, Qt::AlignVCenter);
  log_footer_layout->addWidget(log_event_count_label_, 0, Qt::AlignVCenter);
  log_footer_layout->addStretch(1);
  log_footer_layout->addWidget(log_play_button_, 0, Qt::AlignVCenter);
  transitions_layout->addWidget(log_footer_bar_);

  property_stack->addWidget(editor_property_page_);
  property_stack->addWidget(transitions_page_);
  property_stack_layout->addWidget(property_stack);
  content_splitter->addWidget(property_panel_);
  content_splitter->setStretchFactor(0, 0);
  content_splitter->setStretchFactor(1, 1);
  content_splitter->setStretchFactor(2, 0);

  main_layout->addWidget(content_splitter, 1);

  auto* status_bar = new QFrame(main_column);
  ApplyPanelFooterChrome(status_bar);
  auto* status_layout = new QHBoxLayout(status_bar);
  status_layout->setContentsMargins(PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV,
                                    PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV);
  valid_tree_label_ = new QLabel(status_bar);
  valid_tree_label_->setFixedSize(14, 14);
  valid_tree_label_->setScaledContents(true);
  status_layout->addWidget(valid_tree_label_);
  status_label_ = new QLabel(status_bar);
  StylePanelStatusLabel(status_label_);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_layout->addWidget(status_label_, 1);
  main_layout->addWidget(status_bar);
  root->addWidget(main_column, 1);

  replay_timer_ = new QTimer(this);
  replay_timer_->setInterval(200);

  connect(mode_group_, &QButtonGroup::idClicked, this, &BehaviorTreePanel::onModeChanged);
  connect(trees_filter_edit_, &QLineEdit::textChanged, this,
          &BehaviorTreePanel::applyTreesFilter);
  connect(open_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onOpenClicked);
  connect(save_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onSaveClicked);
  connect(save_as_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onSaveAsClicked);
  connect(fit_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onFitClicked);
  connect(layout_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onLayoutClicked);
  connect(layout_h_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onLayoutHorizontalClicked);
  connect(undo_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onUndoClicked);
  connect(redo_button_, &QToolButton::clicked, this, &BehaviorTreePanel::onRedoClicked);
  connect(export_svg_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onExportSvgClicked);
  connect(palette_, &BtNodePalette::importModelsRequested, this,
          &BehaviorTreePanel::onImportModelsClicked);
  connect(palette_, &BtNodePalette::exportModelsRequested, this,
          &BehaviorTreePanel::onExportModelsClicked);
  connect(palette_, &BtNodePalette::subtreeOpenRequested, this,
          &BehaviorTreePanel::onSubtreeExpandRequested);
  connect(palette_, &BtNodePalette::lockChanged, this, [this](bool locked) {
    for (int i = 0; i < tree_tabs_->count(); ++i) {
      if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
        view->setReadOnly(mode_ != BtPanelMode::kEditor || locked);
      }
    }
  });
  connect(palette_, &BtNodePalette::addNodeRequested, this,
          &BehaviorTreePanel::onAddCustomNodeClicked);
  connect(palette_, &BtNodePalette::editNodeRequested, this,
          &BehaviorTreePanel::onEditCustomNode);
  connect(palette_, &BtNodePalette::removeNodeRequested, this,
          &BehaviorTreePanel::onRemoveCustomNode);
  connect(tree_tabs_->tabBar(), &QTabBar::customContextMenuRequested, this,
          &BehaviorTreePanel::onTreeTabContextMenu);
  connect(file_tree_, &QTreeWidget::itemActivated, this,
          &BehaviorTreePanel::onFileTreeItemActivated);
  connect(tree_tabs_, &QTabWidget::currentChanged, this,
          &BehaviorTreePanel::onTreeTabChanged);
  connect(instance_name_edit_, &QLineEdit::editingFinished, this,
          &BehaviorTreePanel::onInstanceNameEdited);
  connect(port_remap_tree_, &QTreeWidget::itemChanged, this,
          &BehaviorTreePanel::onPortRemapChanged);
  connect(monitor_connect_button_, &QToolButton::toggled, this,
          &BehaviorTreePanel::onMonitorConnectToggled);
  connect(monitor_channel_edit_, &QLineEdit::editingFinished, this,
          &BehaviorTreePanel::onMonitorChannelEdited);
  connect(replay_play_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayPlayPause);
  connect(log_play_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayPlayPause);
  connect(log_event_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &BehaviorTreePanel::onLogEventSpinChanged);
  connect(replay_slider_, &QSlider::valueChanged, this,
          &BehaviorTreePanel::onReplaySliderChanged);
  connect(replay_step_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayStepClicked);
  connect(replay_step_back_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayStepBackClicked);
  connect(replay_clear_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayClearClicked);
  connect(replay_save_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplaySaveClicked);
  connect(replay_open_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onOpenReplayLogClicked);
  connect(log_open_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onOpenReplayLogClicked);
  connect(log_save_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplaySaveClicked);
  connect(log_clear_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onReplayClearClicked);
  connect(toggle_left_sidebar_button_, &QToolButton::toggled, this,
          &BehaviorTreePanel::onToggleLeftSidebar);
  connect(toggle_right_panel_button_, &QToolButton::toggled, this,
          &BehaviorTreePanel::onToggleRightPanel);
  connect(collapse_left_sidebar_button_, &QToolButton::clicked, this, [this]() {
    setLeftSidebarVisible(false);
  });
  connect(collapse_right_panel_button_, &QToolButton::clicked, this, [this]() {
    setRightPanelVisible(false);
  });
  connect(log_collapse_button_, &QToolButton::clicked, this, [this]() {
    setRightPanelVisible(false);
  });
  connect(replay_speed_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &BehaviorTreePanel::onReplaySpeedChanged);
  connect(transitions_table_, &QTableWidget::cellClicked, this,
          &BehaviorTreePanel::onTransitionRowClicked);
  connect(transitions_filter_edit_, &QLineEdit::textChanged, this, [this](const QString&) {
    rebuildTransitionsTable();
  });
  connect(breakpoint_load_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onBreakpointLoadClicked);
  connect(breakpoint_save_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onBreakpointSaveClicked);
  connect(breakpoint_delete_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onBreakpointDeleteClicked);
  connect(breakpoint_clear_button_, &QToolButton::clicked, this,
          &BehaviorTreePanel::onBreakpointClearClicked);
  connect(breakpoints_list_, &QTreeWidget::itemDoubleClicked, this,
          &BehaviorTreePanel::onBreakpointListDoubleClicked);
  connect(port_remap_tree_, &QTreeWidget::itemDoubleClicked, this,
          &BehaviorTreePanel::onPortRemapItemDoubleClicked);
  connect(replay_timer_, &QTimer::timeout, this, [this]() {
    if (replay_events_.isEmpty()) {
      setReplayPlaying(false);
      return;
    }
    if (replay_index_ >= replay_events_.size() - 1) {
      setReplayPlaying(false);
      return;
    }
    updateReplayTimerInterval();
    replay_index_ += 1;
    replay_slider_->blockSignals(true);
    replay_slider_->setValue(replay_index_);
    replay_slider_->blockSignals(false);
    applyReplayUpTo(replay_index_);
  });
}

void BehaviorTreePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("BehaviorTreeDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void BehaviorTreePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

bool BehaviorTreePanel::loadFile(const QString& path) {
  if (path.isEmpty()) {
    return false;
  }
  if (dirty_ && !confirmDiscardChanges()) {
    return false;
  }
  const auto doc = LoadBtDocument(path);
  if (!doc.has_value()) {
    QMessageBox::warning(this, tr("Open Behavior Tree"),
                         tr("Failed to load:\n%1").arg(path));
    return false;
  }
  setDocument(*doc, path);
  markDirty(false);
  emit configChanged();
  updateStatusText(tr("Loaded %1").arg(QFileInfo(path).fileName()));
  return true;
}

bool BehaviorTreePanel::saveFile() {
  if (document_.source_path.isEmpty()) {
    return saveAs();
  }
  if (!SaveBtDocument(document_, document_.source_path)) {
    QMessageBox::warning(this, tr("Save Behavior Tree"),
                         tr("Failed to save:\n%1").arg(document_.source_path));
    return false;
  }
  markDirty(false);
  emit configChanged();
  updateStatusText(tr("Saved %1").arg(QFileInfo(document_.source_path).fileName()));
  return true;
}

bool BehaviorTreePanel::saveAs() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Behavior Tree"), document_.source_path,
      tr("BehaviorTree XML (*.xml)"));
  if (path.isEmpty()) {
    return false;
  }
  document_.source_path = path;
  return saveFile();
}

void BehaviorTreePanel::refreshFileTree() {
  file_tree_->clear();
  const std::optional<QString> config_root = DiscoverConfigRoot();
  if (!config_root.has_value()) {
    return;
  }

  // DiscoverConfigRoot usually returns .../task/behavior_tree already.
  QString bt_root = config_root.value();
  const QString nested = QDir(bt_root).filePath(QStringLiteral("task/behavior_tree"));
  if (QFileInfo(nested).isDir()) {
    bt_root = QFileInfo(nested).absoluteFilePath();
  }

  auto* project_item = new QTreeWidgetItem(file_tree_, {tr("Project")});
  project_item->setIcon(0, GrootIcon(QStringLiteral("svg/folder.svg")));
  project_item->setFlags(project_item->flags() & ~Qt::ItemIsSelectable);

  QHash<QString, QTreeWidgetItem*> folder_items;
  const QDir root_dir(bt_root);
  const QStringList files = ListBtXmlFiles(bt_root);
  for (const QString& file : files) {
    const QString relative = QDir::fromNativeSeparators(root_dir.relativeFilePath(file));
    const QStringList parts = relative.split(QLatin1Char('/'), Qt::SkipEmptyParts);
    if (parts.isEmpty()) {
      continue;
    }

    QTreeWidgetItem* parent = project_item;
    QString folder_key;
    for (int i = 0; i + 1 < parts.size(); ++i) {
      folder_key += (folder_key.isEmpty() ? QString() : QStringLiteral("/")) + parts.at(i);
      auto it = folder_items.find(folder_key);
      if (it == folder_items.end()) {
        auto* folder = new QTreeWidgetItem(parent, {parts.at(i)});
        folder->setIcon(0, GrootIcon(QStringLiteral("svg/folder.svg")));
        folder->setFlags(folder->flags() & ~Qt::ItemIsSelectable);
        it = folder_items.insert(folder_key, folder);
      }
      parent = it.value();
    }

    const QString stem = QFileInfo(parts.last()).completeBaseName();
    auto* file_item = new QTreeWidgetItem(parent, {stem});
    file_item->setIcon(0, GrootIcon(QStringLiteral("svg/files.svg")));
    file_item->setData(0, Qt::UserRole, file);
    file_item->setToolTip(0, file);

    const QString main_tree = PeekMainTreeId(file);
    if (!main_tree.isEmpty()) {
      auto* tree_item = new QTreeWidgetItem(file_item, {main_tree});
      tree_item->setIcon(0, GrootIcon(QStringLiteral("svg/tree.svg")));
      tree_item->setData(0, Qt::UserRole, file);
      tree_item->setToolTip(0, file);
      tree_item->setForeground(0, QColor(QStringLiteral("#2563eb")));
    }
  }

  file_tree_->expandAll();
  applyTreesFilter(trees_filter_edit_ != nullptr ? trees_filter_edit_->text() : QString());
}

void BehaviorTreePanel::applyTreesFilter(const QString& text) {
  if (file_tree_ == nullptr) {
    return;
  }
  const QString needle = text.trimmed();

  std::function<bool(QTreeWidgetItem*)> filter_item = [&](QTreeWidgetItem* item) -> bool {
    if (item == nullptr) {
      return false;
    }
    bool any_child_visible = false;
    for (int i = 0; i < item->childCount(); ++i) {
      any_child_visible = filter_item(item->child(i)) || any_child_visible;
    }
    const bool self_match =
        needle.isEmpty() || item->text(0).contains(needle, Qt::CaseInsensitive) ||
        item->toolTip(0).contains(needle, Qt::CaseInsensitive);
    // Keep folder/project ancestors when a descendant matches.
    const bool visible = needle.isEmpty() || self_match || any_child_visible;
    item->setHidden(!visible);
    return visible;
  };

  for (int i = 0; i < file_tree_->topLevelItemCount(); ++i) {
    filter_item(file_tree_->topLevelItem(i));
  }
}

void BehaviorTreePanel::setLeftSidebarVisible(bool visible) {
  if (left_sidebar_ == nullptr) {
    return;
  }
  left_sidebar_->setVisible(visible);
  if (toggle_left_sidebar_button_ != nullptr) {
    toggle_left_sidebar_button_->blockSignals(true);
    toggle_left_sidebar_button_->setChecked(visible);
    toggle_left_sidebar_button_->setToolTip(visible ? tr("Hide left sidebar")
                                                    : tr("Show left sidebar"));
    toggle_left_sidebar_button_->blockSignals(false);
  }
}

void BehaviorTreePanel::setRightPanelVisible(bool visible) {
  if (property_panel_ == nullptr) {
    return;
  }
  property_panel_->setVisible(visible);
  if (toggle_right_panel_button_ != nullptr) {
    toggle_right_panel_button_->blockSignals(true);
    toggle_right_panel_button_->setChecked(visible);
    toggle_right_panel_button_->setToolTip(visible ? tr("Hide right panel")
                                                   : tr("Show right panel"));
    toggle_right_panel_button_->blockSignals(false);
  }
}

void BehaviorTreePanel::onToggleLeftSidebar(bool visible) {
  setLeftSidebarVisible(visible);
}

void BehaviorTreePanel::onToggleRightPanel(bool visible) {
  setRightPanelVisible(visible);
}

void BehaviorTreePanel::applyModeUi() {
  const bool editor = mode_ == BtPanelMode::kEditor;
  const bool monitor = mode_ == BtPanelMode::kMonitor;
  const bool replay = mode_ == BtPanelMode::kReplay;

  palette_->setEnabled(editor);
  palette_->setVisible(editor);
  breakpoints_frame_->setVisible(monitor || replay);
  open_button_->setEnabled(editor);
  save_button_->setEnabled(editor);
  save_as_button_->setEnabled(editor);

  // Canvas tool strip: Editor gets full tools; Monitor/Replay keep Fit only.
  editor_tools_->setVisible(editor || monitor || replay);
  layout_button_->setVisible(editor);
  layout_h_button_->setVisible(editor);
  undo_button_->setVisible(editor);
  redo_button_->setVisible(editor);
  export_svg_button_->setVisible(editor);
  layout_button_->setEnabled(editor);
  layout_h_button_->setEnabled(editor);
  undo_button_->setEnabled(editor);
  redo_button_->setEnabled(editor);
  export_svg_button_->setEnabled(editor && tree_tabs_->count() > 0);
  fit_button_->setEnabled(editor || monitor || replay);
  fit_button_->setVisible(true);

  monitor_tools_->setVisible(monitor);
  replay_tools_->setVisible(replay);
  playback_bar_->setVisible(replay);
  if (auto* stack = property_panel_->findChild<QStackedWidget*>(QStringLiteral("BtPropertyStack"))) {
    stack->setCurrentWidget((replay || monitor) ? transitions_page_ : editor_property_page_);
  }
  updateValidTreeIndicator();

  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
      view->setReadOnly(!editor);
      view->setFlowAnimationEnabled(monitor || replay);
    }
  }

  if (monitor) {
    // Entering Monitor: subscribe live status channel (Groot2-like).
    if (!monitor_connect_button_->isChecked()) {
      monitor_connect_button_->setChecked(true);
    } else {
      subscribeMonitor();
    }
  } else {
    unsubscribeMonitor();
  }

  if (replay || monitor) {
    rebuildTransitionsTable();
  }
  if (replay) {
    updateReplayTimeLabel();
  } else {
    setReplayPlaying(false);
  }
  if (monitor || replay) {
    rebuildBreakpointList();
    syncHookMarkersToViews();
  }
}

void BehaviorTreePanel::updateStatusText(const QString& text) {
  if (status_label_ != nullptr) {
    status_label_->setText(text);
  }
}

void BehaviorTreePanel::setDocument(const BtDocument& doc, const QString& path) {
  document_ = doc;
  document_.source_path = path;
  if (document_.models.isEmpty()) {
    document_.models = LoadDefaultManifests();
  } else {
    // Merge autonomy tree_nodes_model.xml so PipelineSequence/RateController etc.
    // resolve to Control/Decorator even when the BT file has no TreeNodesModel.
    document_.models = MergeWithBuiltinModels(document_.models);
    const QHash<QString, BtNodeModel> manifests = LoadDefaultManifests();
    for (auto it = manifests.constBegin(); it != manifests.constEnd(); ++it) {
      if (!document_.models.contains(it.key())) {
        document_.models.insert(it.key(), it.value());
      }
    }
  }
  for (auto it = document_.trees.begin(); it != document_.trees.end(); ++it) {
    ReresolveTreeNodeKinds(it.value(), document_.models);
  }
  palette_->setModels(document_.models);
  rebuildTreeTabs();
}

void BehaviorTreePanel::rebuildTreeTabs() {
  while (tree_tabs_->count() > 0) {
    QWidget* widget = tree_tabs_->widget(0);
    tree_tabs_->removeTab(0);
    widget->deleteLater();
  }

  const QStringList tree_ids = document_.trees.keys();
  QStringList sorted_ids = tree_ids;
  sorted_ids.sort(Qt::CaseInsensitive);
  for (const QString& tree_id : sorted_ids) {
    auto* view = new BtGraphView(tree_tabs_);
    view->setAcceptDrops(true);
    view->setTree(document_.trees.value(tree_id), document_.models);
    view->setReadOnly(mode_ != BtPanelMode::kEditor);
    view->setFlowAnimationEnabled(mode_ == BtPanelMode::kMonitor ||
                                  mode_ == BtPanelMode::kReplay);
    const int tab_index = tree_tabs_->addTab(view, tree_id);
    tree_tabs_->tabBar()->setTabData(tab_index, tree_id);
    wireGraphView(view);
    if (tree_id == document_.main_tree_id) {
      tree_tabs_->setTabIcon(tab_index, GrootIcon(QStringLiteral("svg/star.svg")));
    } else {
      tree_tabs_->setTabIcon(tab_index, GrootIcon(QStringLiteral("svg/tree.svg")));
    }
  }

  if (tree_tabs_->count() > 0) {
    int main_index = sorted_ids.indexOf(document_.main_tree_id);
    if (main_index < 0) {
      main_index = 0;
    }
    tree_tabs_->setCurrentIndex(main_index);
  }
  updateTabTitles();
  updateUndoRedoButtons();
  updateValidTreeIndicator();
}

void BehaviorTreePanel::syncCurrentTreeFromView() {
  BtGraphView* view = currentGraphView();
  if (view == nullptr) {
    return;
  }
  const QString tree_id = currentTreeId();
  if (tree_id.isEmpty()) {
    return;
  }
  document_.trees.insert(tree_id, view->serializableTree());
}

BtGraphView* BehaviorTreePanel::graphViewForTreeId(const QString& tree_id) const {
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (treeIdAtTab(i) == tree_id) {
      return qobject_cast<BtGraphView*>(tree_tabs_->widget(i));
    }
  }
  return nullptr;
}

BtGraphView* BehaviorTreePanel::currentGraphView() const {
  return qobject_cast<BtGraphView*>(tree_tabs_->currentWidget());
}

QString BehaviorTreePanel::treeIdAtTab(int index) const {
  if (index < 0 || index >= tree_tabs_->count()) {
    return {};
  }
  const QVariant data = tree_tabs_->tabBar()->tabData(index);
  if (data.isValid()) {
    return data.toString();
  }
  return tree_tabs_->tabText(index).trimmed().remove(QChar('*')).trimmed();
}

QString BehaviorTreePanel::currentTreeId() const {
  return treeIdAtTab(tree_tabs_->currentIndex());
}

void BehaviorTreePanel::markDirty(bool dirty) {
  if (dirty_ == dirty) {
    return;
  }
  dirty_ = dirty;
  updateTabTitles();
}

void BehaviorTreePanel::updateTabTitles() {
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    const QString tree_id = treeIdAtTab(i);
    QString title = tree_id;
    if (dirty_) {
      title += QStringLiteral(" *");
    }
    tree_tabs_->setTabText(i, title);
  }
}

void BehaviorTreePanel::updatePropertyPanel(int uid) {
  updating_property_panel_ = true;
  selected_uid_ = uid;
  BtGraphView* view = currentGraphView();
  if (view == nullptr || uid < 0) {
    instance_name_edit_->clear();
    port_remap_tree_->clear();
    updating_property_panel_ = false;
    return;
  }
  const BtAbsTree tree = view->tree();
  if (!tree.nodes.contains(uid)) {
    instance_name_edit_->clear();
    port_remap_tree_->clear();
    updating_property_panel_ = false;
    return;
  }
  const BtAbsNode& node = tree.nodes.value(uid);
  instance_name_edit_->setText(node.instance_name);
  port_remap_tree_->clear();
  const BtNodeModel model = document_.models.value(node.registration_id);
  for (const BtPortModel& port : model.ports) {
    auto* item = new QTreeWidgetItem(port_remap_tree_);
    item->setText(0, port.name);
    const QString value = node.port_remap.value(port.name, port.default_value);
    item->setText(1, value);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    if (!highlighted_port_value_.isEmpty() && value == highlighted_port_value_) {
      item->setBackground(1, PortValueHighlightColor());
    }
  }
  port_remap_tree_->expandAll();
  updating_property_panel_ = false;
}

void BehaviorTreePanel::updateUndoRedoButtons() {
  BtGraphView* view = currentGraphView();
  const bool editor = mode_ == BtPanelMode::kEditor;
  undo_button_->setEnabled(editor && view != nullptr && view->canUndo());
  redo_button_->setEnabled(editor && view != nullptr && view->canRedo());
}

void BehaviorTreePanel::updateValidTreeIndicator() {
  if (valid_tree_label_ == nullptr) {
    return;
  }
  bool valid = false;
  if (BtGraphView* view = currentGraphView()) {
    const BtAbsTree tree = view->tree();
    valid = tree.root_uid >= 0 && !tree.nodes.isEmpty();
  }
  const QIcon indicator =
      valid ? GrootIcon(QStringLiteral("green-circle.png"), 14)
            : GrootIcon(QStringLiteral("red-circle.png"), 14);
  valid_tree_label_->setPixmap(indicator.pixmap(14, 14));
  valid_tree_label_->setToolTip(valid ? tr("Valid tree") : tr("Not a valid tree"));

  const bool editor = mode_ == BtPanelMode::kEditor;
  if (layout_button_ != nullptr) {
    layout_button_->setEnabled(editor && valid);
  }
  if (layout_h_button_ != nullptr) {
    layout_h_button_->setEnabled(editor && valid);
  }
  if (save_button_ != nullptr) {
    save_button_->setEnabled(editor && valid);
  }
}

bool BehaviorTreePanel::confirmDiscardChanges() {
  const auto answer = QMessageBox::question(
      this, tr("Unsaved Changes"),
      tr("Discard unsaved behavior tree changes?"),
      QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Cancel);
  return answer == QMessageBox::Discard;
}

void BehaviorTreePanel::subscribeMonitor() {
  const QString channel = monitor_channel_edit_ != nullptr
                              ? monitor_channel_edit_->text().trimmed()
                              : QString();
  if (channel.isEmpty()) {
    updateStatusText(tr("Monitor channel is empty"));
    return;
  }
  // Idempotent: recreating the Autolink reader for /behavior_tree/log after a
  // transient hide/show has been observed to segfault in CreateReader.
  if (!monitor_subscription_ids_.empty() && monitor_subscribed_channel_ == channel) {
    return;
  }
  unsubscribeMonitor();

  const auto id = integration::ChannelReaderRegistry::instance().subscribe(
      channel.toStdString(),
      [this](const std::string& payload) { onMonitorPayload(payload); });
  if (id != 0) {
    monitor_subscription_ids_.push_back(id);
    monitor_subscribed_channel_ = channel;
    updateStatusText(tr("Monitoring %1").arg(channel));
  } else {
    monitor_subscribed_channel_.clear();
    updateStatusText(tr("Failed to subscribe %1 (is Autolink connected?)").arg(channel));
  }
}

void BehaviorTreePanel::unsubscribeMonitor() {
  for (const auto id : monitor_subscription_ids_) {
    if (id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(id);
    }
  }
  monitor_subscription_ids_.clear();
  monitor_subscribed_channel_.clear();
}

void BehaviorTreePanel::onMonitorPayload(const std::string& payload) {
  // Reader callbacks run on the Autolink scheduler thread — marshal to Qt GUI.
  const QByteArray bytes =
      QByteArray::fromStdString(integration::DecodeChannelPayload(payload));
  QPointer<BehaviorTreePanel> self(this);
  QMetaObject::invokeMethod(
      this,
      [self, bytes]() {
        if (self.isNull() || self->mode_ != BtPanelMode::kMonitor) {
          return;
        }
        self->processMonitorLog(bytes);
      },
      Qt::QueuedConnection);
}

void BehaviorTreePanel::processMonitorLog(const QByteArray& protobuf_bytes) {
  if (tree_tabs_ == nullptr || monitor_follow_button_ == nullptr ||
      monitor_record_button_ == nullptr) {
    return;
  }

  automsgs::msgs::nav_msgs::BehaviorTreeLog log;
  if (!log.ParseFromArray(protobuf_bytes.constData(), protobuf_bytes.size())) {
    updateStatusText(tr("Ignored invalid BehaviorTreeLog payload"));
    return;
  }

  int applied = 0;
  int recorded = 0;
  for (const auto& event : log.event_log()) {
    const QString node_name = QString::fromStdString(event.node_name());
    const auto status = StatusFromString(QString::fromStdString(event.current_status()));
    if (!status.has_value()) {
      continue;
    }
    std::optional<BtNodeStatus> previous;
    if (!event.previous_status().empty()) {
      previous = StatusFromString(QString::fromStdString(event.previous_status()));
    }

    if (monitor_follow_button_->isChecked()) {
      applyStatusChange(node_name, *status, previous);
      ++applied;
    }
    if (monitor_record_button_->isChecked()) {
      ReplayEvent replay;
      replay.node_name = node_name;
      replay.previous = previous.value_or(BtNodeStatus::kIdle);
      replay.status = *status;
      if (event.has_timestamp()) {
        replay.timestamp_ns = static_cast<qint64>(event.timestamp().sec()) * 1000000000LL +
                              static_cast<qint64>(event.timestamp().nanosec());
      } else if (log.has_timestamp()) {
        replay.timestamp_ns = static_cast<qint64>(log.timestamp().sec()) * 1000000000LL +
                              static_cast<qint64>(log.timestamp().nanosec());
      } else {
        replay.timestamp_ns = 0;
      }
      appendReplayEvent(replay);
      ++recorded;
    }
  }

  if (applied > 0 || recorded > 0) {
    for (int i = 0; i < tree_tabs_->count(); ++i) {
      if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
        view->refreshSignalFlow();
      }
    }
    updateStatusText(tr("BT log: %1 events (%2 highlighted, %3 recorded) · total %4")
                         .arg(log.event_log_size())
                         .arg(applied)
                         .arg(recorded)
                         .arg(replay_events_.size()));
  }
}

void BehaviorTreePanel::applyStatusChange(const QString& node_name, BtNodeStatus status,
                                          std::optional<BtNodeStatus> previous) {
  BtNodeStatus effective = status;

  // Hooks are keyed by instance/registration; log names are often fullPath.
  BtHook* active_hook = nullptr;
  BtHook hook_storage;
  if (hooks_.contains(node_name)) {
    hook_storage = hooks_.value(node_name);
    active_hook = &hook_storage;
  } else {
    const QString leaf = node_name.section(QLatin1Char('/'), -1);
    if (!leaf.isEmpty() && hooks_.contains(leaf)) {
      hook_storage = hooks_.value(leaf);
      active_hook = &hook_storage;
    } else if (node_name.contains(QLatin1Char('/'))) {
      const QString maybe_instance = node_name.section(QLatin1Char('/'), -2, -2);
      if (!maybe_instance.isEmpty() && hooks_.contains(maybe_instance)) {
        hook_storage = hooks_.value(maybe_instance);
        active_hook = &hook_storage;
      }
    }
  }

  if (!handling_breakpoint_ && active_hook != nullptr) {
    BtHook& hook = *active_hook;
    if (hook.enabled && hook.mode != BtHookMode::kNone) {
      bool fire = true;
      if (hook.position == BtHookPosition::kPre) {
        fire = (status == BtNodeStatus::kRunning);
      } else if (hook.mode == BtHookMode::kBreakpoint) {
        // POST breakpoint: fire on Success/Failure, or Running when configured.
        fire = (status == BtNodeStatus::kSuccess || status == BtNodeStatus::kFailure ||
                (status == BtNodeStatus::kRunning &&
                 hook.desired_status == BtNodeStatus::kRunning));
      }

      if (fire && hook.mode == BtHookMode::kReplace) {
        effective = hook.desired_status;
        updateStatusText(tr("Replace hook on %1 → %2")
                             .arg(node_name, StatusToString(effective)));
        if (hook.once) {
          removeHookByName(node_name);
        }
      } else if (fire && hook.mode == BtHookMode::kBreakpoint) {
        if (monitor_follow_button_ != nullptr && monitor_follow_button_->isChecked()) {
          monitor_follow_button_->setChecked(false);
        }
        setReplayPlaying(false);

        handling_breakpoint_ = true;
        BtBreakpointReachedDialog dialog(node_name, status, hook.desired_status, this);
        const int code = dialog.exec();
        handling_breakpoint_ = false;

        if (code != QDialog::Accepted) {
          updateStatusText(tr("Breakpoint paused on %1").arg(node_name));
          // Keep the live status visible while paused.
        } else {
          effective = dialog.chosenStatus();
          updateStatusText(tr("Breakpoint unlocked %1 → %2")
                               .arg(node_name, StatusToString(effective)));
          if (dialog.removeHook() || hook.once) {
            removeHookByName(node_name);
          }
        }
      }
    }
  }

  // Apply across all open tree tabs — log names may resolve in the main tree.
  bool matched = false;
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i));
    if (view == nullptr) {
      continue;
    }
    const int uid = uidForNodeNameInTree(view->tree(), node_name);
    if (uid < 0) {
      continue;
    }
    // Defer full-path relight until the monitor batch finishes.
    view->setNodeStatus(uid, effective, previous, /*refresh_flow=*/false);
    matched = true;
  }
  if (!matched && mode_ == BtPanelMode::kMonitor) {
    // Soft hint only; high-frequency unmatched leaves would spam the status bar.
  }
}

bool BehaviorTreePanel::handleHookOnStatus(const QString&, BtNodeStatus) {
  // Kept for ABI stability of private helpers; logic lives in applyStatusChange.
  return false;
}

void BehaviorTreePanel::openHookDialogForUid(int uid) {
  BtGraphView* view = currentGraphView();
  if (view == nullptr || !view->tree().nodes.contains(uid)) {
    return;
  }
  const BtAbsNode& node = view->tree().nodes.value(uid);
  if (node.kind == BtNodeKind::kRoot || node.expanded_inline) {
    return;
  }
  const QString node_name =
      node.instance_name.isEmpty() ? node.registration_id : node.instance_name;

  BtHook current;
  if (hooks_.contains(node_name)) {
    current = hooks_.value(node_name);
  } else {
    current.node_name = node_name;
    current.node_uid = uid;
    current.mode = BtHookMode::kNone;
  }
  current.node_uid = uid;
  current.node_name = node_name;

  BtHookDialog dialog(node_name, current, this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const BtHook result = dialog.result();
  if (result.mode == BtHookMode::kNone) {
    removeHookByName(node_name);
  } else {
    upsertHook(result);
  }
}

void BehaviorTreePanel::onNodeEditorRequested(int uid) {
  BtGraphView* view = currentGraphView();
  if (view == nullptr || !view->tree().nodes.contains(uid)) {
    return;
  }
  const BtAbsNode node = view->tree().nodes.value(uid);
  if (node.kind == BtNodeKind::kRoot || node.expanded_inline) {
    return;
  }

  const bool read_only = mode_ != BtPanelMode::kEditor || view->isReadOnly();
  BtNodeEditorDialog dialog(node, read_only, this);
  if (dialog.exec() != QDialog::Accepted || read_only) {
    return;
  }
  const BtNodeEditorResult edited = dialog.result();
  if (view->applyNodeEditor(uid, edited.instance_name, edited.skip_if, edited.success_if,
                            edited.failure_if, edited.while_script, edited.on_success,
                            edited.on_failure, edited.on_halted, edited.post_script,
                            edited.description)) {
    if (selected_uid_ == uid) {
      updatePropertyPanel(uid);
    }
    markDirty(true);
    emit configChanged();
    updateStatusText(tr("Updated node %1").arg(edited.instance_name.isEmpty()
                                                   ? node.registration_id
                                                   : edited.instance_name));
  }
}

void BehaviorTreePanel::upsertHook(const BtHook& hook) {
  if (hook.node_name.isEmpty() || hook.mode == BtHookMode::kNone) {
    return;
  }
  hooks_.insert(hook.node_name, hook);
  rebuildBreakpointList();
  syncHookMarkersToViews();
  updateStatusText(tr("Hook set: %1").arg(HookSummary(hook)));
}

void BehaviorTreePanel::removeHookByName(const QString& node_name) {
  if (!hooks_.contains(node_name)) {
    return;
  }
  hooks_.remove(node_name);
  rebuildBreakpointList();
  syncHookMarkersToViews();
}

void BehaviorTreePanel::rebuildBreakpointList() {
  if (breakpoints_list_ == nullptr) {
    return;
  }
  breakpoints_list_->clear();
  QStringList names = hooks_.keys();
  names.sort(Qt::CaseInsensitive);
  for (const QString& name : names) {
    const BtHook& hook = hooks_.value(name);
    if (hook.mode == BtHookMode::kNone) {
      continue;
    }
    auto* item = new QTreeWidgetItem(breakpoints_list_);
    item->setText(0, HookSummary(hook));
    item->setData(0, Qt::UserRole, name);
    item->setToolTip(0, tr("%1\nMode: %2\nPosition: %3\nDesired: %4\nOnce: %5")
                            .arg(name, HookModeToString(hook.mode),
                                 HookPositionToString(hook.position),
                                 StatusToString(hook.desired_status),
                                 hook.once ? tr("yes") : tr("no")));
    if (hook.mode == BtHookMode::kBreakpoint) {
      item->setForeground(0, QColor(185, 28, 28));
    } else {
      item->setForeground(0, QColor(30, 64, 175));
    }
  }
}

void BehaviorTreePanel::syncHookMarkersToViews() {
  QSet<int> uids;
  for (auto it = hooks_.constBegin(); it != hooks_.constEnd(); ++it) {
    if (it->mode == BtHookMode::kNone) {
      continue;
    }
    int uid = it->node_uid;
    if (uid < 0) {
      uid = uidForNodeName(it.key());
    }
    if (uid >= 0) {
      uids.insert(uid);
    }
  }
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
      view->setHookNodeUids(uids);
    }
  }
}

void BehaviorTreePanel::onConfigureHookRequested(int uid) {
  openHookDialogForUid(uid);
}

void BehaviorTreePanel::onBreakpointListDoubleClicked(QTreeWidgetItem* item, int) {
  if (item == nullptr) {
    return;
  }
  const QString name = item->data(0, Qt::UserRole).toString();
  int uid = uidForNodeName(name);
  if (uid < 0 && hooks_.contains(name)) {
    uid = hooks_.value(name).node_uid;
  }
  if (uid >= 0) {
    openHookDialogForUid(uid);
  }
}

void BehaviorTreePanel::onBreakpointDeleteClicked() {
  QTreeWidgetItem* item = breakpoints_list_->currentItem();
  if (item == nullptr) {
    return;
  }
  removeHookByName(item->data(0, Qt::UserRole).toString());
  updateStatusText(tr("Breakpoint removed"));
}

void BehaviorTreePanel::onBreakpointClearClicked() {
  if (hooks_.isEmpty()) {
    return;
  }
  hooks_.clear();
  rebuildBreakpointList();
  syncHookMarkersToViews();
  updateStatusText(tr("All breakpoints cleared"));
}

void BehaviorTreePanel::onBreakpointSaveClicked() {
  if (hooks_.isEmpty()) {
    QMessageBox::information(this, tr("Save Breakpoints"), tr("No breakpoints to save."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Breakpoints"), QStringLiteral("bt_breakpoints.json"),
      tr("JSON (*.json);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  if (!SaveHooksToFile(hooks_, path)) {
    QMessageBox::warning(this, tr("Save Breakpoints"), tr("Cannot write %1").arg(path));
    return;
  }
  updateStatusText(tr("Saved %1 breakpoints").arg(hooks_.size()));
}

void BehaviorTreePanel::onBreakpointLoadClicked() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Load Breakpoints"), {}, tr("JSON (*.json);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  BtHookMap loaded;
  if (!LoadHooksFromFile(path, &loaded)) {
    QMessageBox::warning(this, tr("Load Breakpoints"), tr("Cannot read %1").arg(path));
    return;
  }
  hooks_ = loaded;
  // Rebind uids from current tree names.
  for (auto it = hooks_.begin(); it != hooks_.end(); ++it) {
    const int uid = uidForNodeName(it.key());
    if (uid >= 0) {
      it->node_uid = uid;
    }
  }
  rebuildBreakpointList();
  syncHookMarkersToViews();
  updateStatusText(tr("Loaded %1 breakpoints").arg(hooks_.size()));
}

int BehaviorTreePanel::uidForNodeName(const QString& node_name) const {
  BtGraphView* view = currentGraphView();
  if (view == nullptr) {
    return -1;
  }
  return uidForNodeNameInTree(view->tree(), node_name);
}

int BehaviorTreePanel::uidForNodeNameInTree(const BtAbsTree& tree,
                                           const QString& node_name) const {
  if (node_name.isEmpty() || tree.nodes.isEmpty()) {
    return -1;
  }

  // Publisher uses BT.CPP fullPath(), optionally suffixed with /RegistrationId.
  const QStringList segments =
      node_name.split(QLatin1Char('/'), Qt::SkipEmptyParts);
  const QString leaf = segments.isEmpty() ? node_name : segments.last();
  const QString leaf_parent =
      segments.size() >= 2 ? segments.at(segments.size() - 2) : QString();

  int best_uid = -1;
  int best_score = -1;
  for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
    const BtAbsNode& node = it.value();
    if (node.kind == BtNodeKind::kRoot || node.expanded_inline) {
      continue;
    }
    int score = -1;
    if (!node.instance_name.isEmpty() && node.instance_name == node_name) {
      score = 100;
    } else if (node.registration_id == node_name) {
      score = 90;
    } else if (!node.instance_name.isEmpty() && node.instance_name == leaf) {
      // Prefer instance match on last path segment (PlanFollow).
      score = 80;
    } else if (!node.instance_name.isEmpty() &&
               (node_name.endsWith(QLatin1Char('/') + node.instance_name) ||
                node_name.endsWith(QLatin1Char('/') + node.instance_name +
                                   QLatin1Char('/') + node.registration_id))) {
      score = 85;
    } else if (node.registration_id == leaf) {
      // Unnamed leaf published as .../PoseReady or .../PoseReady/PoseReady.
      score = 70;
      // Prefer the node whose parent instance matches the path segment
      // (.../NavigateWithReplanning/RateController).
      if (!leaf_parent.isEmpty()) {
        const int parent_uid = FindParentUid(tree, it.key());
        if (parent_uid >= 0 && tree.nodes.contains(parent_uid)) {
          const BtAbsNode& parent = tree.nodes.value(parent_uid);
          if (parent.instance_name == leaf_parent ||
              parent.registration_id == leaf_parent) {
            score = 92;
          }
        }
      }
    } else if (!leaf_parent.isEmpty() && node.registration_id == leaf &&
               !node.instance_name.isEmpty() && node.instance_name == leaf_parent) {
      // .../PlanFollow/Sequence
      score = 75;
    } else if (!leaf_parent.isEmpty() && !node.instance_name.isEmpty() &&
               node.instance_name == leaf_parent &&
               node.registration_id == leaf) {
      score = 88;
    }

    if (score > best_score) {
      best_score = score;
      best_uid = it.key();
    }
  }
  return best_uid;
}

void BehaviorTreePanel::appendReplayEvent(const ReplayEvent& event) {
  replay_events_.push_back(event);
  const int index = replay_events_.size() - 1;
  if (replay_slider_ != nullptr) {
    replay_slider_->setMaximum(std::max(0, index));
  }
  if (mode_ == BtPanelMode::kMonitor) {
    replay_index_ = index;
    if (replay_slider_ != nullptr) {
      replay_slider_->blockSignals(true);
      replay_slider_->setValue(replay_index_);
      replay_slider_->blockSignals(false);
    }
    appendTransitionRow(index);
    updateLogFooter();
  } else if (mode_ == BtPanelMode::kReplay) {
    rebuildTransitionsTable();
    updateLogFooter();
  }
}

void BehaviorTreePanel::appendTransitionRow(int event_index) {
  if (transitions_table_ == nullptr || event_index < 0 ||
      event_index >= replay_events_.size()) {
    return;
  }
  const ReplayEvent& event = replay_events_.at(event_index);
  const QString filter = transitions_filter_edit_ != nullptr
                             ? transitions_filter_edit_->text().trimmed()
                             : QString();
  if (!filter.isEmpty() && !event.node_name.contains(filter, Qt::CaseInsensitive)) {
    return;
  }

  updating_transitions_ = true;
  const qint64 t0 = replay_events_.first().timestamp_ns;
  const int row = transitions_table_->rowCount();
  transitions_table_->insertRow(row);
  const double sec =
      (event.timestamp_ns >= t0) ? static_cast<double>(event.timestamp_ns - t0) / 1e9 : 0.0;

  auto* time_item = new QTableWidgetItem(QString::number(sec, 'f', 3));
  bool bold_time = true;
  if (row > 0) {
    QTableWidgetItem* prev_time = transitions_table_->item(row - 1, 0);
    if (prev_time != nullptr && prev_time->text() == time_item->text()) {
      bold_time = false;
    }
  }
  if (bold_time) {
    QFont font = time_item->font();
    font.setBold(true);
    time_item->setFont(font);
  }

  auto* name_item = new QTableWidgetItem(event.node_name);
  auto* prev_item = new QTableWidgetItem(StatusToString(event.previous));
  auto* status_item = new QTableWidgetItem(StatusToString(event.status));
  prev_item->setBackground(MonitorStatusChipColor(event.previous));
  status_item->setBackground(MonitorStatusChipColor(event.status));
  prev_item->setTextAlignment(Qt::AlignCenter);
  status_item->setTextAlignment(Qt::AlignCenter);
  prev_item->setForeground(QColor(30, 30, 30));
  status_item->setForeground(QColor(30, 30, 30));

  time_item->setData(Qt::UserRole, event_index);
  name_item->setData(Qt::UserRole, event_index);
  prev_item->setData(Qt::UserRole, event_index);
  status_item->setData(Qt::UserRole, event_index);
  transitions_table_->setItem(row, 0, time_item);
  transitions_table_->setItem(row, 1, name_item);
  transitions_table_->setItem(row, 2, prev_item);
  transitions_table_->setItem(row, 3, status_item);
  updating_transitions_ = false;

  if (monitor_follow_button_ != nullptr && monitor_follow_button_->isChecked()) {
    transitions_table_->scrollToItem(time_item, QAbstractItemView::PositionAtBottom);
    transitions_table_->selectRow(row);
  }
}

void BehaviorTreePanel::applyReplayUpTo(int index) {
  BtGraphView* view = currentGraphView();
  if (view == nullptr) {
    return;
  }
  view->clearStatuses();
  const int last = qBound(0, index, replay_events_.size() - 1);
  for (int i = 0; i <= last; ++i) {
    const ReplayEvent& event = replay_events_.at(i);
    applyStatusChange(event.node_name, event.status, event.previous);
  }
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (auto* tab_view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
      tab_view->refreshSignalFlow();
    }
  }
  syncTransitionsSelection(last);
  updateReplayTimeLabel();
  updateStatusText(tr("Replay %1 / %2").arg(last + 1).arg(replay_events_.size()));
}

void BehaviorTreePanel::setReplayPlaying(bool playing) {
  replay_playing_ = playing;
  if (replay_play_button_ != nullptr) {
    replay_play_button_->blockSignals(true);
    replay_play_button_->setChecked(playing);
    replay_play_button_->setToolTip(playing ? tr("Pause replay") : tr("Play replay"));
    replay_play_button_->blockSignals(false);
  }
  if (log_play_button_ != nullptr) {
    log_play_button_->blockSignals(true);
    log_play_button_->setChecked(playing);
    log_play_button_->setToolTip(playing ? tr("Pause log replay") : tr("Play log replay"));
    log_play_button_->blockSignals(false);
  }
  if (playing) {
    updateReplayTimerInterval();
    if (replay_timer_ != nullptr) {
      replay_timer_->start();
    }
  } else if (replay_timer_ != nullptr) {
    replay_timer_->stop();
  }
}

void BehaviorTreePanel::updateReplayTimerInterval() {
  int base_ms = 200;
  if (replay_index_ + 1 < replay_events_.size()) {
    const qint64 t0 = replay_events_.at(replay_index_).timestamp_ns;
    const qint64 t1 = replay_events_.at(replay_index_ + 1).timestamp_ns;
    if (t1 > t0) {
      base_ms = static_cast<int>((t1 - t0) / 1000000);
    }
  }
  const double speed = std::max(0.01, replay_speed_);
  const int ms = qBound(8, static_cast<int>(base_ms / speed), 4000);
  if (replay_timer_ != nullptr) {
    replay_timer_->setInterval(ms);
  }
}

void BehaviorTreePanel::updateReplayTimeLabel() {
  if (replay_time_label_ != nullptr) {
    if (replay_events_.isEmpty() || replay_index_ < 0) {
      replay_time_label_->setText(QStringLiteral("00.000"));
    } else {
      const int idx = qBound(0, replay_index_, replay_events_.size() - 1);
      const qint64 t0 = replay_events_.first().timestamp_ns;
      const qint64 t = replay_events_.at(idx).timestamp_ns;
      const double sec = (t >= t0) ? static_cast<double>(t - t0) / 1e9 : 0.0;
      replay_time_label_->setText(QString::number(sec, 'f', 3).rightJustified(6, QChar('0')));
    }
  }
  updateLogFooter();
}

void BehaviorTreePanel::updateLogFooter() {
  if (log_event_spin_ == nullptr || log_event_count_label_ == nullptr) {
    return;
  }
  const int total = replay_events_.size();
  // Groot2 uses 1-based display: "9 of 64".
  log_event_spin_->blockSignals(true);
  if (total <= 0) {
    log_event_spin_->setRange(0, 0);
    log_event_spin_->setValue(0);
    log_event_count_label_->setText(tr("of 0"));
  } else {
    log_event_spin_->setRange(1, total);
    const int display = qBound(1, replay_index_ + 1, total);
    log_event_spin_->setValue(display);
    log_event_count_label_->setText(tr("of %1").arg(total));
  }
  log_event_spin_->blockSignals(false);
  if (log_play_button_ != nullptr) {
    log_play_button_->setEnabled(total > 0);
  }
}

void BehaviorTreePanel::onLogEventSpinChanged(int value) {
  if (replay_events_.isEmpty() || value < 1) {
    return;
  }
  const int index = qBound(0, value - 1, replay_events_.size() - 1);
  if (index == replay_index_) {
    return;
  }
  setReplayPlaying(false);
  replay_index_ = index;
  if (replay_slider_ != nullptr) {
    replay_slider_->blockSignals(true);
    replay_slider_->setValue(index);
    replay_slider_->blockSignals(false);
  }
  applyReplayUpTo(index);
}

namespace {

QColor StatusTextColor(BtNodeStatus status) {
  switch (status) {
    case BtNodeStatus::kSuccess:
      return QColor(34, 160, 34);
    case BtNodeStatus::kRunning:
      return QColor(0, 170, 70);
    case BtNodeStatus::kFailure:
      return QColor(220, 50, 50);
    case BtNodeStatus::kIdle:
    default:
      return QColor(100, 116, 139);
  }
}

}  // namespace

void BehaviorTreePanel::rebuildTransitionsTable() {
  if (transitions_table_ == nullptr) {
    return;
  }
  updating_transitions_ = true;
  const QString filter = transitions_filter_edit_ != nullptr
                             ? transitions_filter_edit_->text().trimmed()
                             : QString();
  transitions_table_->setRowCount(0);
  if (replay_events_.isEmpty()) {
    updating_transitions_ = false;
    return;
  }
  const qint64 t0 = replay_events_.first().timestamp_ns;
  QString last_time_text;
  for (int i = 0; i < replay_events_.size(); ++i) {
    const ReplayEvent& event = replay_events_.at(i);
    if (!filter.isEmpty() &&
        !event.node_name.contains(filter, Qt::CaseInsensitive)) {
      continue;
    }
    const int row = transitions_table_->rowCount();
    transitions_table_->insertRow(row);
    const double sec =
        (event.timestamp_ns >= t0) ? static_cast<double>(event.timestamp_ns - t0) / 1e9 : 0.0;
    const QString time_text = QString::number(sec, 'f', 3);
    auto* time_item = new QTableWidgetItem(time_text);
    if (time_text != last_time_text) {
      QFont font = time_item->font();
      font.setBold(true);
      time_item->setFont(font);
      last_time_text = time_text;
    }
    auto* name_item = new QTableWidgetItem(event.node_name);
    auto* prev_item = new QTableWidgetItem(StatusToString(event.previous));
    auto* status_item = new QTableWidgetItem(StatusToString(event.status));
    prev_item->setBackground(MonitorStatusChipColor(event.previous));
    status_item->setBackground(MonitorStatusChipColor(event.status));
    prev_item->setTextAlignment(Qt::AlignCenter);
    status_item->setTextAlignment(Qt::AlignCenter);
    prev_item->setForeground(QColor(30, 30, 30));
    status_item->setForeground(QColor(30, 30, 30));
    time_item->setData(Qt::UserRole, i);
    name_item->setData(Qt::UserRole, i);
    prev_item->setData(Qt::UserRole, i);
    status_item->setData(Qt::UserRole, i);
    transitions_table_->setItem(row, 0, time_item);
    transitions_table_->setItem(row, 1, name_item);
    transitions_table_->setItem(row, 2, prev_item);
    transitions_table_->setItem(row, 3, status_item);
  }
  updating_transitions_ = false;
  syncTransitionsSelection(replay_index_);
}

void BehaviorTreePanel::syncTransitionsSelection(int index) {
  if (transitions_table_ == nullptr || updating_transitions_) {
    return;
  }
  for (int row = 0; row < transitions_table_->rowCount(); ++row) {
    QTableWidgetItem* item = transitions_table_->item(row, 0);
    if (item != nullptr && item->data(Qt::UserRole).toInt() == index) {
      transitions_table_->blockSignals(true);
      transitions_table_->selectRow(row);
      transitions_table_->scrollToItem(item, QAbstractItemView::PositionAtCenter);
      transitions_table_->blockSignals(false);
      return;
    }
  }
}

void BehaviorTreePanel::onReplaySpeedChanged(int index) {
  if (replay_speed_combo_ == nullptr || index < 0) {
    return;
  }
  replay_speed_ = replay_speed_combo_->itemData(index).toDouble();
  if (replay_speed_ <= 0.0) {
    replay_speed_ = 1.0;
  }
  updateReplayTimerInterval();
}

void BehaviorTreePanel::onTransitionRowClicked(int row, int) {
  if (transitions_table_ == nullptr || row < 0) {
    return;
  }
  QTableWidgetItem* item = transitions_table_->item(row, 0);
  if (item == nullptr) {
    return;
  }
  const int index = item->data(Qt::UserRole).toInt();
  if (index < 0 || index >= replay_events_.size()) {
    return;
  }
  setReplayPlaying(false);
  replay_index_ = index;
  replay_slider_->blockSignals(true);
  replay_slider_->setValue(index);
  replay_slider_->blockSignals(false);
  applyReplayUpTo(index);
}

void BehaviorTreePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void BehaviorTreePanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (mode_ == BtPanelMode::kMonitor && monitor_connect_button_ != nullptr &&
      monitor_connect_button_->isChecked()) {
    subscribeMonitor();  // no-op if already subscribed to the same channel
  }
}

void BehaviorTreePanel::hideEvent(QHideEvent* event) {
  // Do not unsubscribe here: tearing down and recreating the Autolink reader for
  // /behavior_tree/log on transient dock hide/show can segfault in CreateReader.
  // Subscription is released on leaving Monitor, disconnect toggle, or dtor.
  setReplayPlaying(false);
  QWidget::hideEvent(event);
}

void BehaviorTreePanel::onModeChanged(int mode_id) {
  const BtPanelMode next_mode = static_cast<BtPanelMode>(mode_id);
  if (next_mode != BtPanelMode::kEditor && dirty_) {
    if (!confirmDiscardChanges()) {
      mode_group_->blockSignals(true);
      mode_group_->button(static_cast<int>(mode_))->setChecked(true);
      mode_group_->blockSignals(false);
      return;
    }
    markDirty(false);
  }
  mode_ = next_mode;
  applyModeUi();
  emit configChanged();
}

void BehaviorTreePanel::onOpenClicked() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open Behavior Tree"), document_.source_path,
      tr("BehaviorTree XML (*.xml)"));
  if (!path.isEmpty()) {
    loadFile(path);
  }
}

void BehaviorTreePanel::onSaveClicked() { saveFile(); }

void BehaviorTreePanel::onSaveAsClicked() { saveAs(); }

void BehaviorTreePanel::onFitClicked() {
  if (BtGraphView* view = currentGraphView()) {
    view->fitToView();
  }
}

void BehaviorTreePanel::onLayoutClicked() {
  if (BtGraphView* view = currentGraphView()) {
    view->applyAutoLayout(false);
    markDirty(true);
    emit configChanged();
  }
}

void BehaviorTreePanel::onLayoutHorizontalClicked() {
  if (BtGraphView* view = currentGraphView()) {
    view->applyAutoLayout(true);
    markDirty(true);
    emit configChanged();
  }
}

void BehaviorTreePanel::onExportSvgClicked() {
  BtGraphView* view = currentGraphView();
  if (view == nullptr) {
    return;
  }
  const QString default_name =
      currentTreeId().isEmpty() ? QStringLiteral("behavior_tree.svg")
                                : currentTreeId() + QStringLiteral(".svg");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export SVG"), default_name, tr("SVG Files (*.svg)"));
  if (path.isEmpty()) {
    return;
  }
  if (view->exportSvg(path)) {
    updateStatusText(tr("Exported SVG to %1").arg(path));
  } else {
    QMessageBox::warning(this, tr("Export SVG"), tr("Failed to export %1").arg(path));
  }
}

void BehaviorTreePanel::onImportModelsClicked() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Import TreeNodesModel"), document_.source_path,
      tr("BehaviorTree XML (*.xml)"));
  if (path.isEmpty()) {
    return;
  }
  const auto imported = LoadImportedCustomModels(path);
  if (!imported.has_value()) {
    QMessageBox::warning(this, tr("Import Models"), tr("Failed to load %1").arg(path));
    return;
  }
  if (imported->isEmpty()) {
    QMessageBox::warning(this, tr("Import Models"), tr("No custom node models found in %1").arg(path));
    return;
  }

  const QSet<QString> stale =
      CustomModelsToRemoveOnImport(this, document_.models, *imported);
  for (const QString& model_id : stale) {
    removeCustomModel(model_id);
  }
  for (auto it = imported->constBegin(); it != imported->constEnd(); ++it) {
    addCustomModel(it.value());
  }

  markDirty(true);
  updateStatusText(tr("Imported %1 custom node models").arg(imported->size()));
  emit configChanged();
}

void BehaviorTreePanel::onExportModelsClicked() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export TreeNodesModel"),
      QStringLiteral("tree_nodes_model.xml"), tr("BehaviorTree XML (*.xml)"));
  if (path.isEmpty()) {
    return;
  }
  if (SaveTreeNodesModelFile(document_.models, path)) {
    updateStatusText(tr("Exported models to %1").arg(path));
  } else {
    QMessageBox::warning(this, tr("Export Models"), tr("Failed to save %1").arg(path));
  }
}

void BehaviorTreePanel::wireGraphView(BtGraphView* view) {
  if (view == nullptr) {
    return;
  }
  view->setSubTreeLookup([this](const QString& tree_id) -> const BtAbsTree* {
    if (!document_.trees.contains(tree_id)) {
      return nullptr;
    }
    return &document_.trees[tree_id];
  });
  connect(view, &BtGraphView::selectionChanged, this,
          &BehaviorTreePanel::onGraphSelectionChanged);
  connect(view, &BtGraphView::treeChanged, this, &BehaviorTreePanel::onGraphTreeChanged);
  connect(view, &BtGraphView::createSubtreeRequested, this,
          &BehaviorTreePanel::onCreateSubtreeRequested);
  connect(view, &BtGraphView::subtreeExpandRequested, this,
          &BehaviorTreePanel::onSubtreeExpandRequested);
  connect(view, &BtGraphView::configureHookRequested, this,
          &BehaviorTreePanel::onConfigureHookRequested);
  connect(view, &BtGraphView::nodeDoubleClicked, this, [this](int uid) {
    if (mode_ == BtPanelMode::kMonitor || mode_ == BtPanelMode::kReplay) {
      onConfigureHookRequested(uid);
    } else if (mode_ == BtPanelMode::kEditor) {
      onNodeEditorRequested(uid);
    }
  });
  syncHookMarkersToViews();
}

void BehaviorTreePanel::onCreateSubtreeRequested(int uid) {
  BtGraphView* view = currentGraphView();
  if (view == nullptr || uid < 0) {
    return;
  }

  bool ok = false;
  const QString subtree_id = QInputDialog::getText(
      this, tr("Create SubTree"), tr("SubTree ID"), QLineEdit::Normal, {}, &ok);
  if (!ok || subtree_id.trimmed().isEmpty()) {
    return;
  }
  const QString trimmed = subtree_id.trimmed();
  if (document_.trees.contains(trimmed)) {
    QMessageBox::warning(this, tr("Create SubTree"),
                         tr("A BehaviorTree named [%1] already exists.").arg(trimmed));
    return;
  }

  const auto extracted = view->extractSubtree(uid, trimmed);
  if (!extracted.has_value()) {
    QMessageBox::warning(this, tr("Create SubTree"),
                         tr("Unable to create SubTree from the selected node."));
    return;
  }

  document_.trees.insert(trimmed, *extracted);
  if (!document_.models.contains(trimmed)) {
    document_.models.insert(trimmed, MakeSubTreeModel(trimmed));
  }
  syncModelsToViews();
  syncCurrentTreeFromView();
  markDirty(true);
  const QString current_id = currentTreeId();
  rebuildTreeTabs();
  if (!current_id.isEmpty()) {
    for (int i = 0; i < tree_tabs_->count(); ++i) {
      if (treeIdAtTab(i) == current_id) {
        tree_tabs_->setCurrentIndex(i);
        break;
      }
    }
  }
  updateStatusText(tr("Created SubTree %1").arg(trimmed));
  emit configChanged();
}

void BehaviorTreePanel::onSubtreeExpandRequested(const QString& tree_id) {
  if (tree_id.isEmpty()) {
    return;
  }
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (treeIdAtTab(i) == tree_id) {
      tree_tabs_->setCurrentIndex(i);
      updateStatusText(tr("Opened SubTree %1").arg(tree_id));
      return;
    }
  }
  updateStatusText(tr("SubTree %1 not found in document").arg(tree_id));
}

void BehaviorTreePanel::onPortRemapItemDoubleClicked(QTreeWidgetItem* item, int column) {
  if (item == nullptr || column != 1) {
    return;
  }
  const QString value = item->text(1).trimmed();
  BtGraphView* view = currentGraphView();
  if (view == nullptr) {
    return;
  }
  if (value.isEmpty()) {
    highlighted_port_value_.clear();
    view->clearPortHighlight();
  } else {
    highlighted_port_value_ = value;
    view->setHighlightedPortValue(value);
    updateStatusText(tr("Highlighting port value: %1").arg(value));
  }
  if (selected_uid_ >= 0) {
    updatePropertyPanel(selected_uid_);
  }
}

void BehaviorTreePanel::onUndoClicked() {
  if (BtGraphView* view = currentGraphView()) {
    view->undo();
    updateUndoRedoButtons();
  }
}

void BehaviorTreePanel::onRedoClicked() {
  if (BtGraphView* view = currentGraphView()) {
    view->redo();
    updateUndoRedoButtons();
  }
}

void BehaviorTreePanel::onFileTreeItemActivated(QTreeWidgetItem* item, int /*column*/) {
  if (item == nullptr) {
    return;
  }
  const QString path = item->data(0, Qt::UserRole).toString();
  if (!path.isEmpty()) {
    loadFile(path);
  }
}

void BehaviorTreePanel::onTreeTabChanged(int /*index*/) {
  updatePropertyPanel(-1);
  updateUndoRedoButtons();
}

void BehaviorTreePanel::onGraphSelectionChanged(int uid) { updatePropertyPanel(uid); }

void BehaviorTreePanel::onGraphTreeChanged() {
  const QString edited_tree_id = currentTreeId();
  syncCurrentTreeFromView();
  markDirty(true);
  emit configChanged();
  updateUndoRedoButtons();
  updateValidTreeIndicator();
  if (selected_uid_ >= 0) {
    updatePropertyPanel(selected_uid_);
  }

  if (!edited_tree_id.isEmpty()) {
    const BtNodeModel model = document_.models.value(edited_tree_id);
    if (model.kind == BtNodeKind::kSubTree) {
      for (int i = 0; i < tree_tabs_->count(); ++i) {
        if (treeIdAtTab(i) == edited_tree_id) {
          continue;
        }
        if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
          view->refreshExpandedSubTree(edited_tree_id);
        }
      }
    }
  }
}

void BehaviorTreePanel::onInstanceNameEdited() {
  if (updating_property_panel_ || mode_ != BtPanelMode::kEditor) {
    return;
  }
  BtGraphView* view = currentGraphView();
  if (view == nullptr || selected_uid_ < 0) {
    return;
  }
  BtAbsTree tree = view->tree();
  if (!tree.nodes.contains(selected_uid_)) {
    return;
  }
  tree.nodes[selected_uid_].instance_name = instance_name_edit_->text().trimmed();
  view->setTree(tree, document_.models);
  markDirty(true);
  emit configChanged();
}

void BehaviorTreePanel::onPortRemapChanged() {
  if (updating_property_panel_ || mode_ != BtPanelMode::kEditor) {
    return;
  }
  BtGraphView* view = currentGraphView();
  if (view == nullptr || selected_uid_ < 0) {
    return;
  }
  BtAbsTree tree = view->tree();
  if (!tree.nodes.contains(selected_uid_)) {
    return;
  }
  QHash<QString, QString> remap;
  for (int i = 0; i < port_remap_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = port_remap_tree_->topLevelItem(i);
    remap.insert(item->text(0), item->text(1));
  }
  tree.nodes[selected_uid_].port_remap = remap;
  view->setTree(tree, document_.models);
  markDirty(true);
  emit configChanged();
}

void BehaviorTreePanel::onMonitorConnectToggled(bool enabled) {
  if (enabled && mode_ == BtPanelMode::kMonitor) {
    subscribeMonitor();
  } else {
    unsubscribeMonitor();
  }
  emit configChanged();
}

void BehaviorTreePanel::onMonitorChannelEdited() {
  if (mode_ == BtPanelMode::kMonitor && monitor_connect_button_ != nullptr &&
      monitor_connect_button_->isChecked()) {
    // Channel string changed → force reader recreate via channel mismatch.
    subscribeMonitor();
  }
  emit configChanged();
}

void BehaviorTreePanel::onReplayPlayPause() {
  setReplayPlaying(!replay_playing_);
}

void BehaviorTreePanel::onReplaySliderChanged(int value) {
  replay_index_ = value;
  applyReplayUpTo(replay_index_);
}

void BehaviorTreePanel::onReplayStepClicked() {
  if (replay_events_.isEmpty()) {
    return;
  }
  replay_index_ = qMin(replay_index_ + 1, replay_events_.size() - 1);
  replay_slider_->setValue(replay_index_);
  applyReplayUpTo(replay_index_);
}

void BehaviorTreePanel::onReplayStepBackClicked() {
  if (replay_events_.isEmpty()) {
    return;
  }
  replay_index_ = qMax(0, replay_index_ - 1);
  replay_slider_->setValue(replay_index_);
  applyReplayUpTo(replay_index_);
}

void BehaviorTreePanel::onReplayClearClicked() {
  setReplayPlaying(false);
  replay_events_.clear();
  replay_index_ = 0;
  replay_slider_->setMaximum(0);
  replay_slider_->setValue(0);
  if (BtGraphView* view = currentGraphView()) {
    view->clearStatuses();
  }
  rebuildTransitionsTable();
  updateReplayTimeLabel();
  updateStatusText(tr("Replay buffer cleared"));
}

void BehaviorTreePanel::onReplaySaveClicked() {
  if (replay_events_.isEmpty()) {
    QMessageBox::information(this, tr("Save Replay Log"), tr("Replay buffer is empty."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Replay Log"), QStringLiteral("behavior_tree_replay.jsonl"),
      tr("JSON Lines (*.jsonl *.json);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Save Replay Log"), tr("Cannot write %1").arg(path));
    return;
  }
  for (const ReplayEvent& event : replay_events_) {
    QJsonObject obj;
    obj.insert(QStringLiteral("node_name"), event.node_name);
    obj.insert(QStringLiteral("previous"), StatusToString(event.previous));
    obj.insert(QStringLiteral("status"), StatusToString(event.status));
    obj.insert(QStringLiteral("timestamp"), static_cast<double>(event.timestamp_ns));
    file.write(QJsonDocument(obj).toJson(QJsonDocument::Compact));
    file.write("\n");
  }
  updateStatusText(tr("Saved %1 replay events").arg(replay_events_.size()));
}

void BehaviorTreePanel::onOpenReplayLogClicked() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open Replay Log"), {},
      tr("JSON Lines (*.jsonl *.json);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Open Replay Log"), tr("Cannot open %1").arg(path));
    return;
  }
  replay_events_.clear();
  while (!file.atEnd()) {
    const QByteArray line = file.readLine().trimmed();
    if (line.isEmpty()) {
      continue;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(line);
    if (!doc.isObject()) {
      continue;
    }
    const QJsonObject obj = doc.object();
    ReplayEvent event;
    event.node_name = obj.value(QStringLiteral("node_name")).toString();
    const auto status = StatusFromString(obj.value(QStringLiteral("status")).toString());
    if (!status.has_value()) {
      continue;
    }
    event.status = *status;
    if (obj.contains(QStringLiteral("previous"))) {
      event.previous =
          StatusFromString(obj.value(QStringLiteral("previous")).toString())
              .value_or(BtNodeStatus::kIdle);
    }
    event.timestamp_ns = static_cast<qint64>(obj.value(QStringLiteral("timestamp")).toDouble());
    replay_events_.push_back(event);
  }
  replay_slider_->setMaximum(
      std::max(0, static_cast<int>(replay_events_.size()) - 1));
  replay_index_ = 0;
  replay_slider_->setValue(0);
  rebuildTransitionsTable();
  applyReplayUpTo(0);
  updateStatusText(tr("Loaded %1 replay events").arg(replay_events_.size()));
}

void BehaviorTreePanel::syncModelsToViews() {
  palette_->setModels(document_.models);
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
      view->setTree(view->tree(), document_.models);
    }
  }
}

void BehaviorTreePanel::addCustomModel(const BtNodeModel& model) {
  if (model.registration_id.isEmpty() || IsBuiltinModel(model.registration_id)) {
    return;
  }
  document_.models.insert(model.registration_id, model);
  if (model.kind == BtNodeKind::kSubTree && !document_.trees.contains(model.registration_id)) {
    document_.trees.insert(model.registration_id, MakeEmptySubTree(model.registration_id));
    rebuildTreeTabs();
  } else {
    syncModelsToViews();
  }
  markDirty(true);
}

void BehaviorTreePanel::replaceCustomModel(const QString& old_id, const BtNodeModel& new_model) {
  if (old_id.isEmpty() || new_model.registration_id.isEmpty()) {
    return;
  }
  document_.models.remove(old_id);
  document_.models.insert(new_model.registration_id, new_model);
  if (old_id != new_model.registration_id) {
    onTreeNodeModelEdited(old_id, new_model.registration_id);
  }
  syncModelsToViews();
  markDirty(true);
}

void BehaviorTreePanel::removeCustomModel(const QString& model_id) {
  if (model_id.isEmpty() || IsBuiltinModel(model_id)) {
    return;
  }
  const QString usage_tree = FindTreeUsingModel(document_.trees, model_id);
  if (!usage_tree.isEmpty()) {
    const BtNodeModel& model = document_.models.value(model_id);
    if (model.kind != BtNodeKind::kSubTree) {
      QMessageBox::warning(
          this, tr("Can't remove this model"),
          tr("You are using this model in the tree called [%1].\n"
             "Remove all instances of [%2] before deleting the model.")
              .arg(usage_tree, model_id));
      return;
    }

    const int ret = QMessageBox::warning(
        this, tr("Delete SubTree?"),
        tr("The SubTree model will be removed and its tab will be closed.\n"
           "Are you sure? This action can't be undone."),
        QMessageBox::Cancel | QMessageBox::Yes, QMessageBox::Cancel);
    if (ret != QMessageBox::Yes) {
      return;
    }
    destroySubTreeTab(model_id);
  }

  document_.models.remove(model_id);
  syncModelsToViews();
  markDirty(true);
}

void BehaviorTreePanel::onTreeNodeModelEdited(const QString& old_id, const QString& new_id) {
  RenameModelInAllTrees(document_.trees, old_id, new_id);
  if (document_.trees.contains(old_id) && old_id != new_id) {
    BtAbsTree tree = document_.trees.take(old_id);
    tree.tree_id = new_id;
    document_.trees.insert(new_id, tree);
  }
  syncCurrentTreeFromView();
  rebuildTreeTabs();
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (treeIdAtTab(i) == new_id) {
      tree_tabs_->setCurrentIndex(i);
      break;
    }
  }
}

void BehaviorTreePanel::destroySubTreeTab(const QString& tree_id) {
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (auto* view = qobject_cast<BtGraphView*>(tree_tabs_->widget(i))) {
      view->collapseExpandedSubTreesById(tree_id);
    }
  }
  if (!document_.trees.contains(tree_id)) {
    return;
  }
  document_.trees.remove(tree_id);
  if (document_.main_tree_id == tree_id) {
    document_.main_tree_id.clear();
  }
  rebuildTreeTabs();
  if (document_.main_tree_id.isEmpty() && tree_tabs_->count() > 0) {
    setMainTreeTab(0);
  }
}

void BehaviorTreePanel::onAddCustomNodeClicked() {
  BtCustomNodeDialog dialog(document_.models, {}, this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto model = dialog.model();
  if (!model.has_value()) {
    return;
  }
  addCustomModel(*model);
  updateStatusText(tr("Added custom node %1").arg(model->registration_id));
  emit configChanged();
}

void BehaviorTreePanel::onEditCustomNode(const QString& registration_id) {
  if (registration_id.isEmpty() || IsBuiltinModel(registration_id)) {
    return;
  }
  BtCustomNodeDialog dialog(document_.models, registration_id, this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto model = dialog.model();
  if (!model.has_value()) {
    return;
  }
  replaceCustomModel(registration_id, *model);
  updateStatusText(tr("Updated custom node %1").arg(model->registration_id));
  emit configChanged();
}

void BehaviorTreePanel::onRemoveCustomNode(const QString& registration_id) {
  if (registration_id.isEmpty() || IsBuiltinModel(registration_id)) {
    return;
  }
  const BtNodeModel& model = document_.models.value(registration_id);
  int ret = QMessageBox::Cancel;
  if (model.kind != BtNodeKind::kSubTree) {
    ret = QMessageBox::warning(this, tr("Delete TreeNode Model?"),
                               tr("Are you sure? This action can't be undone."),
                               QMessageBox::Cancel | QMessageBox::Yes, QMessageBox::Cancel);
  } else {
    ret = QMessageBox::warning(
        this, tr("Delete SubTree?"),
        tr("The SubTree model will be removed and its tab will be closed.\n"
           "Are you sure? This action can't be undone."),
        QMessageBox::Cancel | QMessageBox::Yes, QMessageBox::Cancel);
  }
  if (ret != QMessageBox::Yes) {
    return;
  }
  removeCustomModel(registration_id);
  updateStatusText(tr("Removed custom node %1").arg(registration_id));
  emit configChanged();
}

void BehaviorTreePanel::onTreeTabContextMenu(const QPoint& pos) {
  const int tab_index = tree_tabs_->tabBar()->tabAt(pos);
  if (tab_index < 0) {
    return;
  }

  QMenu menu(this);
  QAction* rename_action = menu.addAction(tr("Rename"));
  QAction* set_main_action = menu.addAction(tr("Set as main tree"));
  connect(rename_action, &QAction::triggered, this, [this, tab_index]() {
    bool ok = false;
    const QString old_name = treeIdAtTab(tab_index);
    const QString new_name = QInputDialog::getText(
        this, tr("Change name"), tr("Insert the new name of this BehaviorTree"),
        QLineEdit::Normal, old_name, &ok);
    if (!ok || new_name.trimmed().isEmpty()) {
      return;
    }
    renameTreeTab(tab_index, new_name.trimmed());
  });
  connect(set_main_action, &QAction::triggered, this,
          [this, tab_index]() { setMainTreeTab(tab_index); });
  menu.exec(tree_tabs_->tabBar()->mapToGlobal(pos));
}

void BehaviorTreePanel::renameTreeTab(int tab_index, const QString& new_name) {
  if (tab_index < 0 || tab_index >= tree_tabs_->count()) {
    return;
  }
  const QString old_name = treeIdAtTab(tab_index);
  if (new_name == old_name) {
    return;
  }
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (i != tab_index && treeIdAtTab(i) == new_name) {
      QMessageBox::warning(
          this, tr("Tab name already in use"),
          tr("There is already a BehaviorTree called [%1].\nUse another name.").arg(new_name));
      return;
    }
  }

  syncCurrentTreeFromView();
  BtAbsTree tree = document_.trees.take(old_name);
  tree.tree_id = new_name;
  document_.trees.insert(new_name, tree);
  if (document_.main_tree_id == old_name) {
    document_.main_tree_id = new_name;
  }

  if (document_.models.contains(old_name) &&
      document_.models.value(old_name).kind == BtNodeKind::kSubTree) {
    BtNodeModel model = document_.models.take(old_name);
    model.registration_id = new_name;
    document_.models.insert(new_name, model);
    onTreeNodeModelEdited(old_name, new_name);
  } else {
    rebuildTreeTabs();
    for (int i = 0; i < tree_tabs_->count(); ++i) {
      if (treeIdAtTab(i) == new_name) {
        tree_tabs_->setCurrentIndex(i);
        break;
      }
    }
  }

  markDirty(true);
  updateStatusText(tr("Renamed tree %1 → %2").arg(old_name, new_name));
  emit configChanged();
}

void BehaviorTreePanel::setMainTreeTab(int tab_index) {
  if (tab_index < 0 || tab_index >= tree_tabs_->count()) {
    return;
  }
  document_.main_tree_id = treeIdAtTab(tab_index);
  for (int i = 0; i < tree_tabs_->count(); ++i) {
    if (i == tab_index) {
      tree_tabs_->setTabIcon(i, GrootIcon(QStringLiteral("svg/star.svg")));
    } else {
      tree_tabs_->setTabIcon(i, GrootIcon(QStringLiteral("svg/tree.svg")));
    }
  }
  markDirty(true);
  emit configChanged();
}

}  // namespace behavior_tree
}  // namespace autoviz
