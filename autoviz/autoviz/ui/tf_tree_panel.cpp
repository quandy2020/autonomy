/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/tf_tree_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QFocusEvent>
#include <QShowEvent>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <functional>
#include <vector>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/tf_tree_graph_view.hpp"

namespace autoviz {
namespace {

constexpr int kRoleFrameId = Qt::UserRole;

QLabel* MakeDetailLabel(const QString& text, QWidget* parent) {
  auto* label = new QLabel(text, parent);
  label->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 12px;"));
  label->setWordWrap(true);
  return label;
}

QLabel* MakeDetailValue(QWidget* parent) {
  auto* label = new QLabel(QStringLiteral("—"), parent);
  label->setStyleSheet(QStringLiteral("color: palette(text); font-size: 12px;"));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setWordWrap(true);
  return label;
}

QString NormalizeParent(const QString& parent) {
  if (parent.isEmpty() || parent == QLatin1String("NO_PARENT")) {
    return {};
  }
  return parent;
}

}  // namespace

TfTreePanel::TfTreePanel(transform::Buffer* tf_buffer,
                         common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), tf_buffer_(tf_buffer), manager_(manager) {
  setFocusPolicy(Qt::StrongFocus);
  setupUi();

  // Cap tree rebuilds: transforms-changed fires per setTransform (often >> 30Hz).
  refresh_timer_ = new QTimer(this);
  refresh_timer_->setTimerType(Qt::PreciseTimer);
  refresh_timer_->setSingleShot(true);
  refresh_timer_->setInterval(250);
  connect(refresh_timer_, &QTimer::timeout, this,
          &TfTreePanel::onCoalescedRefresh);

  if (tf_buffer_ != nullptr) {
    transforms_changed_connection_ =
        tf_buffer_->_addTransformsChangedListener([this]() { scheduleRefresh(); });
  }
  force_rebuild_ = true;
  onCoalescedRefresh();
}

TfTreePanel::~TfTreePanel() { transforms_changed_connection_.disconnect(); }

void TfTreePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("TfTreeDock");
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

void TfTreePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

void TfTreePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void TfTreePanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  // App activation can emit show for docks; do not force a full tree rebuild
  // (combined with GL restore + TF catch-up that freezes the UI).
  scheduleRefresh();
}

void TfTreePanel::setupUi() {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* toolbar_layout = new QVBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(10, 8, 10, 6);
  toolbar_layout->setSpacing(4);

  auto* filter_row = new QHBoxLayout();
  auto* filter_icon = new QLabel(QStringLiteral("⌕"), toolbar);
  filter_icon->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 14px;"));
  filter_icon->setFixedWidth(16);
  filter_row->addWidget(filter_icon);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setPlaceholderText(tr("Filter frames"));
  filter_edit_->setClearButtonEnabled(true);
  StyleFilterLineEdit(filter_edit_);
  filter_row->addWidget(filter_edit_, 1);
  auto* fit_button = new QToolButton(toolbar);
  fit_button->setText(tr("Fit"));
  fit_button->setToolTip(tr("Zoom the TF graph to fit"));
  filter_row->addWidget(fit_button);
  toolbar_layout->addLayout(filter_row);

  summary_label_ = new QLabel(toolbar);
  StyleHintLabel(summary_label_);
  toolbar_layout->addWidget(summary_label_);
  root->addWidget(toolbar);

  view_tabs_ = new QTabWidget(this);
  view_tabs_->setDocumentMode(true);
  view_tabs_->setTabPosition(QTabWidget::North);
  root->addWidget(view_tabs_, 1);

  auto* tree_page = new QWidget(view_tabs_);
  auto* tree_page_layout = new QVBoxLayout(tree_page);
  tree_page_layout->setContentsMargins(0, 0, 0, 0);
  tree_page_layout->setSpacing(0);

  auto* splitter = new QSplitter(Qt::Horizontal, tree_page);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(1);
  splitter->setStyleSheet(PanelSplitterStyle());

  tree_ = new QTreeWidget(splitter);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setUniformRowHeights(true);
  // Animated expand/collapse is expensive when the tree is rebuilt often.
  tree_->setAnimated(false);
  tree_->setIndentation(18);
  StylePanelTree(tree_);

  auto* detail_panel = new QFrame(splitter);
  detail_panel->setMinimumWidth(220);
  detail_panel->setStyleSheet(
      QStringLiteral(
          "QFrame { background: palette(window); border-left: 1px solid palette(midlight); }"));
  auto* detail_layout = new QVBoxLayout(detail_panel);
  detail_layout->setContentsMargins(14, 14, 14, 14);
  detail_layout->setSpacing(10);

  detail_title_ = new QLabel(tr("Frame details"), detail_panel);
  detail_title_->setStyleSheet(
      QStringLiteral("color: palette(text); font-size: 14px; font-weight: 600;"));
  detail_layout->addWidget(detail_title_);

  detail_hint_ = new QLabel(tr("Select a frame to inspect transform metadata."),
                            detail_panel);
  detail_hint_->setWordWrap(true);
  StyleHintLabel(detail_hint_);
  detail_layout->addWidget(detail_hint_);

  detail_body_ = new QWidget(detail_panel);
  auto* form = new QFormLayout(detail_body_);
  form->setContentsMargins(0, 0, 0, 0);
  form->setSpacing(8);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignTop);
  detail_parent_value_ = MakeDetailValue(detail_body_);
  detail_type_value_ = MakeDetailValue(detail_body_);
  detail_authority_value_ = MakeDetailValue(detail_body_);
  detail_last_time_value_ = MakeDetailValue(detail_body_);
  detail_age_value_ = MakeDetailValue(detail_body_);
  detail_count_value_ = MakeDetailValue(detail_body_);
  form->addRow(MakeDetailLabel(tr("Parent"), detail_body_), detail_parent_value_);
  form->addRow(MakeDetailLabel(tr("Type"), detail_body_), detail_type_value_);
  form->addRow(MakeDetailLabel(tr("Authority"), detail_body_), detail_authority_value_);
  form->addRow(MakeDetailLabel(tr("Last transform"), detail_body_),
               detail_last_time_value_);
  form->addRow(MakeDetailLabel(tr("Time since update"), detail_body_), detail_age_value_);
  form->addRow(MakeDetailLabel(tr("Transforms received"), detail_body_),
               detail_count_value_);
  detail_body_->hide();
  detail_layout->addWidget(detail_body_);
  detail_layout->addStretch();

  splitter->addWidget(tree_);
  splitter->addWidget(detail_panel);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  tree_page_layout->addWidget(splitter, 1);
  view_tabs_->addTab(tree_page, tr("Tree"));

  graph_view_ = new TfTreeGraphView(view_tabs_);
  view_tabs_->addTab(graph_view_, tr("Graph"));

  connect(filter_edit_, &QLineEdit::textChanged, this, &TfTreePanel::onFilterChanged);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &TfTreePanel::onFrameSelectionChanged);
  connect(graph_view_, &TfTreeGraphView::frameActivated, this, [this](const QString& frame_id) {
    if (tree_ == nullptr || frame_id.isEmpty()) {
      return;
    }
    const auto node_it = frame_nodes_.find(frame_id);
    if (node_it != frame_nodes_.end() && node_it->item != nullptr) {
      tree_->setCurrentItem(node_it->item);
      updateDetailsForItem(node_it->item);
    }
  });
  connect(fit_button, &QToolButton::clicked, this, [this]() {
    if (graph_view_ != nullptr) {
      graph_view_->zoomToFit();
      if (view_tabs_ != nullptr) {
        view_tabs_->setCurrentWidget(graph_view_);
      }
    }
  });
  connect(view_tabs_, &QTabWidget::currentChanged, this, [this](int /*index*/) {
    force_rebuild_ = true;
    refresh_pending_ = true;
    onCoalescedRefresh();
  });
}

double TfTreePanel::currentTimeSec() const {
  return manager_ != nullptr ? manager_->simTimeSec() : 0.0;
}

QString TfTreePanel::formatTimestampSec(double sec) const {
  if (!(sec > 0.0) || !std::isfinite(sec)) {
    return tr("Unknown");
  }
  const QDateTime date_time = QDateTime::fromSecsSinceEpoch(
      static_cast<qint64>(sec), Qt::UTC);
  return date_time.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
}

QString TfTreePanel::formatAgeSec(double age_sec) const {
  if (age_sec < 0.0 || !std::isfinite(age_sec)) {
    return tr("Unknown");
  }
  if (age_sec < 0.05) {
    return tr("just now");
  }
  if (age_sec < 60.0) {
    return tr("%1 s ago").arg(QString::number(age_sec, 'f', 2));
  }
  if (age_sec < 3600.0) {
    return tr("%1 min ago").arg(QString::number(age_sec / 60.0, 'f', 1));
  }
  return tr("%1 h ago").arg(QString::number(age_sec / 3600.0, 'f', 1));
}

void TfTreePanel::setPaused(bool paused) {
  paused_ = paused;
  if (paused) {
    if (refresh_timer_ != nullptr) {
      refresh_timer_->stop();
    }
    refresh_pending_ = false;
    return;
  }
  scheduleRefresh();
}

void TfTreePanel::scheduleRefresh() {
  if (paused_) {
    return;
  }
  refresh_pending_ = true;
  if (refresh_timer_ != nullptr && !refresh_timer_->isActive()) {
    refresh_timer_->start();
  }
}

void TfTreePanel::refresh() {
  // Periodic tick / external callers: rate-limited like transforms-changed.
  scheduleRefresh();
}

QString TfTreePanel::structureFingerprint(
    const std::vector<transform::TfFrameStats>& frames) const {
  const QString filter =
      filter_edit_ != nullptr ? filter_edit_->text().trimmed() : QString();
  QStringList parts;
  parts.reserve(static_cast<int>(frames.size()) + 1);
  parts.push_back(filter);
  for (const transform::TfFrameStats& stats : frames) {
    parts.push_back(QString::fromStdString(stats.frame_id) + QLatin1Char('>') +
                    NormalizeParent(QString::fromStdString(stats.parent_id)) +
                    (stats.is_static ? QLatin1Char('S') : QLatin1Char('D')));
  }
  parts.sort(Qt::CaseSensitive);
  return parts.join(QLatin1Char('|'));
}

void TfTreePanel::updateStatsInPlace(
    const std::vector<transform::TfFrameStats>& frames) {
  int root_count = 0;
  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    auto it = frame_nodes_.find(frame_id);
    if (it == frame_nodes_.end()) {
      continue;
    }
    it->stats = stats;
    if (NormalizeParent(QString::fromStdString(stats.parent_id)).isEmpty()) {
      ++root_count;
    }
  }
  // root_count from stats may under-count filtered trees; reuse summary size.
  updateSummaryLabel(static_cast<int>(frames.size()),
                     std::max(1, tree_ != nullptr ? tree_->topLevelItemCount()
                                                  : root_count));
  if (graph_view_ != nullptr && view_tabs_ != nullptr &&
      view_tabs_->currentWidget() == graph_view_) {
    graph_view_->setFrames(frames, currentTimeSec(),
                           filter_edit_ != nullptr ? filter_edit_->text().trimmed()
                                                   : QString());
  }
  if (tree_ != nullptr) {
    updateDetailsForItem(tree_->currentItem());
  }
}

void TfTreePanel::onCoalescedRefresh() {
  if (!refresh_pending_ && !force_rebuild_) {
    return;
  }
  refresh_pending_ = false;

  // Skip work while the dock is hidden; showEvent will force a rebuild.
  if (!isVisible() && !force_rebuild_) {
    return;
  }

  if (tf_buffer_ == nullptr) {
    force_rebuild_ = false;
    structure_fingerprint_.clear();
    tree_->clear();
    frame_nodes_.clear();
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("TF buffer unavailable."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("TF buffer unavailable."));
    return;
  }

  const std::vector<transform::TfFrameStats> frames = tf_buffer_->frameStats();
  const QString fingerprint = structureFingerprint(frames);
  if (!force_rebuild_ && fingerprint == structure_fingerprint_ &&
      !frame_nodes_.isEmpty()) {
    updateStatsInPlace(frames);
    return;
  }
  force_rebuild_ = false;
  structure_fingerprint_ = fingerprint;
  rebuildTree();
}

void TfTreePanel::rebuildTree() {
  selected_frame_id_.clear();
  if (tree_->currentItem() != nullptr) {
    selected_frame_id_ = tree_->currentItem()->data(0, kRoleFrameId).toString();
  }

  const QSignalBlocker blocker(tree_);
  tree_->clear();
  frame_nodes_.clear();

  if (tf_buffer_ == nullptr) {
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("TF buffer unavailable."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("TF buffer unavailable."));
    return;
  }

  const std::vector<transform::TfFrameStats> frames = tf_buffer_->frameStats();
  structure_fingerprint_ = structureFingerprint(frames);
  if (frames.empty()) {
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("No transforms received yet."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("No transforms received yet."));
    return;
  }

  QHash<QString, QString> child_to_parent;
  QHash<QString, transform::TfFrameStats> stats_by_frame;
  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    const QString parent = NormalizeParent(QString::fromStdString(stats.parent_id));
    stats_by_frame.insert(frame_id, stats);
    if (!parent.isEmpty()) {
      child_to_parent.insert(frame_id, parent);
    }
  }

  const QString filter = filter_edit_->text().trimmed();
  auto frameMatchesFilter = [&](const QString& frame_id) {
    return filter.isEmpty() ||
           frame_id.contains(filter, Qt::CaseInsensitive) ||
           child_to_parent.value(frame_id).contains(filter, Qt::CaseInsensitive);
  };

  QHash<QString, QTreeWidgetItem*> item_by_frame;
  int root_count = 0;

  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    if (!frameMatchesFilter(frame_id)) {
      continue;
    }
    auto* item = new QTreeWidgetItem({frame_id});
    item->setData(0, kRoleFrameId, frame_id);
    item->setToolTip(0, frame_id);
    if (stats.is_static) {
      item->setForeground(0, QColor(136, 192, 208));
    }
    frame_nodes_.insert(frame_id, FrameNode{stats, item});
    item_by_frame.insert(frame_id, item);
  }

  for (auto it = item_by_frame.begin(); it != item_by_frame.end(); ++it) {
    const QString frame_id = it.key();
    const transform::TfFrameStats stats = stats_by_frame.value(frame_id);
    QTreeWidgetItem* item = it.value();
    const QString parent = NormalizeParent(QString::fromStdString(stats.parent_id));
    if (!parent.isEmpty() && item_by_frame.contains(parent)) {
      item_by_frame.value(parent)->addChild(item);
    } else {
      tree_->addTopLevelItem(item);
      ++root_count;
    }
  }

  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* root = tree_->topLevelItem(i);
    std::function<void(QTreeWidgetItem*)> expand_all = [&](QTreeWidgetItem* node) {
      node->setExpanded(true);
      for (int c = 0; c < node->childCount(); ++c) {
        expand_all(node->child(c));
      }
    };
    expand_all(root);
  }

  tree_->sortItems(0, Qt::AscendingOrder);
  updateSummaryLabel(static_cast<int>(frames.size()), root_count);
  if (graph_view_ != nullptr) {
    graph_view_->requestFit();
    graph_view_->setFrames(frames, currentTimeSec(), filter);
  }

  if (!selected_frame_id_.isEmpty()) {
    const auto nodes = frame_nodes_.values();
    for (const FrameNode& node : nodes) {
      if (node.item != nullptr &&
          node.item->data(0, kRoleFrameId).toString() == selected_frame_id_) {
        tree_->setCurrentItem(node.item);
        break;
      }
    }
  }

  if (tree_->currentItem() == nullptr && tree_->topLevelItemCount() > 0) {
    tree_->setCurrentItem(tree_->topLevelItem(0));
  }
  onFrameSelectionChanged();
}

void TfTreePanel::updateSummaryLabel(int frame_count, int tree_count) {
  QString text;
  if (frame_count == 0) {
    text = tr("No frames");
  } else if (tree_count <= 1) {
    text = tr("%1 frames").arg(frame_count);
  } else {
    text = tr("%1 frames · %2 disconnected trees")
               .arg(frame_count)
               .arg(tree_count);
  }
  if (summary_label_->text() != text) {
    summary_label_->setText(text);
  }
}

void TfTreePanel::onFilterChanged(const QString& /*text*/) {
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
}

void TfTreePanel::onFrameSelectionChanged() {
  if (graph_view_ != nullptr) {
    const QTreeWidgetItem* item = tree_ != nullptr ? tree_->currentItem() : nullptr;
    graph_view_->setCurrentFrame(item != nullptr
                                     ? item->data(0, kRoleFrameId).toString()
                                     : QString());
  }
  updateDetailsForItem(tree_->currentItem());
}

void TfTreePanel::updateDetailsForItem(QTreeWidgetItem* item) {
  auto set_if_changed = [](QLabel* label, const QString& text) {
    if (label != nullptr && label->text() != text) {
      label->setText(text);
    }
  };

  if (item == nullptr) {
    detail_body_->hide();
    detail_hint_->show();
    set_if_changed(detail_hint_,
                   tr("Select a frame to inspect transform metadata."));
    set_if_changed(detail_title_, tr("Frame details"));
    return;
  }

  const QString frame_id = item->data(0, kRoleFrameId).toString();
  const FrameNode node = frame_nodes_.value(frame_id);
  set_if_changed(detail_title_, frame_id);
  detail_hint_->hide();
  detail_body_->show();

  const QString parent = NormalizeParent(QString::fromStdString(node.stats.parent_id));
  set_if_changed(detail_parent_value_,
                 parent.isEmpty() ? tr("(root)") : parent);
  set_if_changed(detail_type_value_,
                 node.stats.is_static ? tr("Static") : tr("Dynamic"));
  set_if_changed(
      detail_authority_value_,
      node.stats.authority.empty() ? tr("Unknown")
                                   : QString::fromStdString(node.stats.authority));

  const double stamp_sec =
      static_cast<double>(node.stats.last_stamp_ns) / 1e9;
  set_if_changed(detail_last_time_value_, formatTimestampSec(stamp_sec));

  if (node.stats.is_static) {
    set_if_changed(detail_age_value_, tr("Static transform"));
  } else if (node.stats.last_stamp_ns <= 0) {
    set_if_changed(detail_age_value_, tr("Unknown"));
  } else {
    const double age = currentTimeSec() - stamp_sec;
    set_if_changed(detail_age_value_, formatAgeSec(age));
  }

  set_if_changed(detail_count_value_,
                 node.stats.transforms_received > 0
                     ? QString::number(node.stats.transforms_received)
                     : tr("0"));
}

}  // namespace autoviz
