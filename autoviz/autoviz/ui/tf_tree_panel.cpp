/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/tf_tree_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QFocusEvent>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QSplitter>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <functional>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

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
  if (tf_buffer_ != nullptr) {
    transforms_changed_connection_ =
        tf_buffer_->_addTransformsChangedListener([this]() {
          QMetaObject::invokeMethod(this, "refresh", Qt::QueuedConnection);
        });
  }
  refresh();
}

TfTreePanel::~TfTreePanel() { transforms_changed_connection_.disconnect(); }

void TfTreePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  auto* tools = new QWidget(dock);
  tools->setStyleSheet(PlotTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 4, 0);
  layout->setSpacing(0);
  layout->addWidget(CreateTitleSeparator(tools));

  auto* split_right = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_right.svg")),
      tr("Split right"));
  layout->addWidget(split_right);
  connect(split_right, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Horizontal);
  });

  auto* split_down = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_down.svg")),
      tr("Split down"));
  layout->addWidget(split_down);
  connect(split_down, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Vertical);
  });

  auto* expand = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand"), true);
  expand_button_ = expand;
  layout->addWidget(expand);
  connect(expand, &QToolButton::clicked, this, [this]() { emit panelExpandRequested(); });

  auto* more_button = CreatePlotTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
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
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  dock->setTitleBarTools(tools);
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

void TfTreePanel::setupUi() {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("TfTreePanel { background: palette(window); }"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  toolbar->setStyleSheet(
      QStringLiteral(
          "QFrame {"
          "  background: palette(base);"
          "  border-bottom: 1px solid palette(midlight);"
          "}"));
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
  filter_edit_->setStyleSheet(
      QStringLiteral(
          "QLineEdit {"
          "  background: palette(base);"
          "  border: 1px solid palette(mid);"
          "  border-radius: 4px;"
          "  padding: 5px 8px;"
          "  color: palette(text);"
          "}"
          "QLineEdit:focus { border-color: palette(highlight); }"));
  filter_row->addWidget(filter_edit_, 1);
  toolbar_layout->addLayout(filter_row);

  summary_label_ = new QLabel(toolbar);
  summary_label_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 11px;"));
  toolbar_layout->addWidget(summary_label_);
  root->addWidget(toolbar);

  auto* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(1);
  splitter->setStyleSheet(
      QStringLiteral("QSplitter::handle { background: palette(midlight); }"));

  tree_ = new QTreeWidget(splitter);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setUniformRowHeights(true);
  tree_->setAnimated(true);
  tree_->setIndentation(18);
  tree_->setStyleSheet(
      QStringLiteral(
          "QTreeWidget {"
          "  background: palette(base);"
          "  border: none;"
          "  color: palette(text);"
          "  outline: none;"
          "}"
          "QTreeWidget::item {"
          "  padding: 4px 2px;"
          "  border-radius: 4px;"
          "}"
          "QTreeWidget::item:hover {"
          "  background: palette(alternate-base);"
          "}"
          "QTreeWidget::item:selected {"
          "  background: palette(highlight);"
          "  color: palette(highlighted-text);"
          "}"));

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
  detail_hint_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 12px;"));
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
  root->addWidget(splitter, 1);

  connect(filter_edit_, &QLineEdit::textChanged, this, &TfTreePanel::onFilterChanged);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &TfTreePanel::onFrameSelectionChanged);
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

void TfTreePanel::refresh() {
  rebuildTree();
}

void TfTreePanel::rebuildTree() {
  selected_frame_id_.clear();
  if (tree_->currentItem() != nullptr) {
    selected_frame_id_ = tree_->currentItem()->data(0, kRoleFrameId).toString();
  }

  tree_->clear();
  frame_nodes_.clear();

  if (tf_buffer_ == nullptr) {
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("TF buffer unavailable."));
    return;
  }

  const std::vector<transform::TfFrameStats> frames = tf_buffer_->frameStats();
  if (frames.empty()) {
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
    root->setExpanded(true);
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
  if (frame_count == 0) {
    summary_label_->setText(tr("No frames"));
    return;
  }
  if (tree_count <= 1) {
    summary_label_->setText(tr("%1 frames").arg(frame_count));
    return;
  }
  summary_label_->setText(tr("%1 frames · %2 disconnected trees")
                              .arg(frame_count)
                              .arg(tree_count));
}

void TfTreePanel::onFilterChanged(const QString& /*text*/) { rebuildTree(); }

void TfTreePanel::onFrameSelectionChanged() {
  updateDetailsForItem(tree_->currentItem());
}

void TfTreePanel::updateDetailsForItem(QTreeWidgetItem* item) {
  if (item == nullptr) {
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("Select a frame to inspect transform metadata."));
    detail_title_->setText(tr("Frame details"));
    return;
  }

  const QString frame_id = item->data(0, kRoleFrameId).toString();
  const FrameNode node = frame_nodes_.value(frame_id);
  detail_title_->setText(frame_id);
  detail_hint_->hide();
  detail_body_->show();

  const QString parent = NormalizeParent(QString::fromStdString(node.stats.parent_id));
  detail_parent_value_->setText(parent.isEmpty() ? tr("(root)") : parent);
  detail_type_value_->setText(node.stats.is_static ? tr("Static") : tr("Dynamic"));
  detail_authority_value_->setText(
      node.stats.authority.empty() ? tr("Unknown")
                                   : QString::fromStdString(node.stats.authority));

  const double stamp_sec =
      static_cast<double>(node.stats.last_stamp_ns) / 1e9;
  detail_last_time_value_->setText(formatTimestampSec(stamp_sec));

  if (node.stats.is_static) {
    detail_age_value_->setText(tr("Static transform"));
  } else if (node.stats.last_stamp_ns <= 0) {
    detail_age_value_->setText(tr("Unknown"));
  } else {
    const double age = currentTimeSec() - stamp_sec;
    detail_age_value_->setText(formatAgeSec(age));
  }

  detail_count_value_->setText(
      node.stats.transforms_received > 0
          ? QString::number(node.stats.transforms_received)
          : tr("0"));
}

}  // namespace autoviz
