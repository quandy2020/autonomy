/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/displays_panel.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <QBrush>
#include <QDialog>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QQuaternion>
#include <QSet>
#include <QSignalBlocker>
#include <QTextBrowser>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/display_status.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/display/tf_display.hpp"
#include "autoviz/ui/add_display_dialog.hpp"
#include "autoviz/ui/display_tree_delegate.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

QStringList CompatibleChannelsForDisplay(
    const common::VisualizationManager& manager, const display::Display& display) {
  const common::DisplayTypeInfo type_info =
      common::DisplayCatalog::infoForType(display.typeId());
  QStringList channels;
  if (type_info.message_types.empty()) {
    for (const auto& name : manager.channelNames()) {
      channels.push_back(QString::fromStdString(name));
    }
  } else {
    for (const auto& info : manager.channels()) {
      for (const auto& expected : type_info.message_types) {
        if (commsgs::MessageTypesCompatible(info.message_type, expected)) {
          channels.push_back(QString::fromStdString(info.channel_name));
          break;
        }
      }
    }
  }
  const QString current = QString::fromStdString(display.channel());
  if (!current.isEmpty() && channels.indexOf(current) < 0) {
    channels.push_front(current);
  }
  channels.removeDuplicates();
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

void SetTreeItemMeta(QTreeWidgetItem* item, DisplayTreeItemKind kind,
                     int display_index = -1,
                     const QString& property_key = {},
                     const QString& description = {},
                     int child_index = -1) {
  const auto write_col = [&](int col) {
    item->setData(col, kDisplayTreeRoleKind, static_cast<int>(kind));
    item->setData(col, kDisplayTreeRoleDisplayIndex, display_index);
    item->setData(col, kDisplayTreeRolePropertyKey, property_key);
    item->setData(col, kDisplayTreeRoleDescription, description);
    item->setData(col, kDisplayTreeRoleChildIndex, child_index);
  };
  write_col(kDisplayTreeColName);
  write_col(kDisplayTreeColValue);
}

Qt::ItemFlags NameColumnFlags(bool editable_value) {
  Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  if (editable_value) {
    flags |= Qt::ItemIsEditable;
  }
  return flags;
}

Qt::ItemFlags ValueColumnFlags(bool editable, bool checkable = false) {
  Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  if (editable) {
    flags |= Qt::ItemIsEditable;
  }
  if (checkable) {
    flags |= Qt::ItemIsUserCheckable;
  }
  return flags;
}

QString FormatTfVector(const QVector3D& v) {
  return QStringLiteral("%1; %2; %3")
      .arg(v.x(), 0, 'f', 3)
      .arg(v.y(), 0, 'f', 3)
      .arg(v.z(), 0, 'f', 3);
}

QString FormatTfQuat(const QQuaternion& q) {
  return QStringLiteral("%1; %2; %3; %4")
      .arg(q.x(), 0, 'f', 3)
      .arg(q.y(), 0, 'f', 3)
      .arg(q.z(), 0, 'f', 3)
      .arg(q.scalar(), 0, 'f', 3);
}

QTreeWidgetItem* FindChildByPropertyKey(QTreeWidgetItem* parent,
                                        const QString& key) {
  if (parent == nullptr) {
    return nullptr;
  }
  for (int i = 0; i < parent->childCount(); ++i) {
    QTreeWidgetItem* child = parent->child(i);
    if (child->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey)
            .toString() == key) {
      return child;
    }
  }
  return nullptr;
}

void EnsureReadonlyField(QTreeWidgetItem* parent, int display_index,
                         int child_index, const QString& key,
                         const QString& label, const QString& value) {
  QTreeWidgetItem* field = FindChildByPropertyKey(parent, key);
  if (field == nullptr) {
    field = new QTreeWidgetItem(parent);
    field->setText(kDisplayTreeColName, label);
    field->setFlags(NameColumnFlags(false));
    SetTreeItemMeta(field, DisplayTreeItemKind::kDisplayFrameField,
                    display_index, key, {}, child_index);
  }
  if (field->text(kDisplayTreeColValue) != value) {
    field->setText(kDisplayTreeColValue, value);
  }
}

void SyncTfFramePoseFields(QTreeWidgetItem* frame_item, int display_index,
                           int child_index, const QString& key,
                           const display::TfFrameSnapshot& snap,
                           const QString& position_label,
                           const QString& orientation_label,
                           const QString& rel_position_label,
                           const QString& rel_orientation_label,
                           const QString& unavailable) {
  EnsureReadonlyField(
      frame_item, display_index, child_index, key + ".position", position_label,
      snap.have_fixed_pose ? FormatTfVector(snap.position) : unavailable);
  EnsureReadonlyField(
      frame_item, display_index, child_index, key + ".orientation",
      orientation_label,
      snap.have_fixed_pose ? FormatTfQuat(snap.orientation) : unavailable);
  EnsureReadonlyField(frame_item, display_index, child_index,
                      key + ".rel_position", rel_position_label,
                      FormatTfVector(snap.rel_position));
  EnsureReadonlyField(frame_item, display_index, child_index,
                      key + ".rel_orientation", rel_orientation_label,
                      FormatTfQuat(snap.rel_orientation));
}

bool TfDisplayHasExpandedPoseRows(QTreeWidgetItem* display_item) {
  if (display_item == nullptr || !display_item->isExpanded()) {
    return false;
  }
  QTreeWidgetItem* frames_cat =
      FindChildByPropertyKey(display_item, QStringLiteral("frames"));
  if (frames_cat == nullptr || !frames_cat->isExpanded()) {
    return false;
  }
  for (int i = 0; i < frames_cat->childCount(); ++i) {
    QTreeWidgetItem* child = frames_cat->child(i);
    const auto kind = static_cast<DisplayTreeItemKind>(
        child->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplayFrameEnabled &&
        child->isExpanded()) {
      return true;
    }
  }
  return false;
}

/** UserRole scratch: last synced TF Tree edge fingerprint (avoid rebuild flicker). */
constexpr int kDisplayTreeRoleTfTreeFingerprint = Qt::UserRole + 20;

QString TfTreeEdgesFingerprint(
    const std::vector<std::pair<std::string, std::string>>& edges) {
  QStringList parts;
  parts.reserve(static_cast<int>(edges.size()));
  for (const auto& edge : edges) {
    parts.push_back(QString::fromStdString(edge.first) + QLatin1Char('>') +
                    QString::fromStdString(edge.second));
  }
  parts.sort(Qt::CaseSensitive);
  return parts.join(QLatin1Char('|'));
}

void CollectExpandedTreeKeys(QTreeWidgetItem* item, QSet<QString>* keys) {
  if (item == nullptr || keys == nullptr) {
    return;
  }
  if (item->isExpanded()) {
    const QString key =
        item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey).toString();
    if (!key.isEmpty()) {
      keys->insert(key);
    }
  }
  for (int i = 0; i < item->childCount(); ++i) {
    CollectExpandedTreeKeys(item->child(i), keys);
  }
}

void RestoreExpandedTreeKeys(QTreeWidgetItem* item, const QSet<QString>& keys) {
  if (item == nullptr) {
    return;
  }
  const QString key =
      item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey).toString();
  if (!key.isEmpty() && keys.contains(key)) {
    item->setExpanded(true);
  }
  for (int i = 0; i < item->childCount(); ++i) {
    RestoreExpandedTreeKeys(item->child(i), keys);
  }
}

void RebuildTfTreeCategory(QTreeWidgetItem* tree_cat, int display_index,
                           int child_index,
                           const std::vector<std::pair<std::string, std::string>>&
                               edges) {
  const QString fingerprint = TfTreeEdgesFingerprint(edges);
  const QString previous =
      tree_cat->data(kDisplayTreeColName, kDisplayTreeRoleTfTreeFingerprint)
          .toString();
  if (fingerprint == previous) {
    return;
  }

  // Deleting children collapses the parent in Qt — preserve expand state.
  QSet<QString> expanded_keys;
  CollectExpandedTreeKeys(tree_cat, &expanded_keys);
  const bool tree_expanded = tree_cat->isExpanded();

  while (tree_cat->childCount() > 0) {
    delete tree_cat->takeChild(0);
  }

  std::map<std::string, std::vector<std::string>> children;
  std::vector<std::string> roots;
  for (const auto& edge : edges) {
    if (edge.second.empty()) {
      roots.push_back(edge.first);
    } else {
      children[edge.second].push_back(edge.first);
    }
  }
  std::sort(roots.begin(), roots.end());
  for (auto& entry : children) {
    std::sort(entry.second.begin(), entry.second.end());
  }

  std::function<void(QTreeWidgetItem*, const std::string&)> add_node =
      [&](QTreeWidgetItem* parent, const std::string& name) {
        auto* node = new QTreeWidgetItem(parent);
        node->setText(kDisplayTreeColName, QString::fromStdString(name));
        node->setFlags(NameColumnFlags(false));
        SetTreeItemMeta(node, DisplayTreeItemKind::kDisplayTreeNode,
                        display_index,
                        QStringLiteral("tree.") + QString::fromStdString(name),
                        {}, child_index);
        const auto it = children.find(name);
        if (it == children.end()) {
          return;
        }
        for (const std::string& child_name : it->second) {
          add_node(node, child_name);
        }
      };

  for (const std::string& root : roots) {
    add_node(tree_cat, root);
  }
  // Orphans whose parent is filtered out still appear as roots.
  for (const auto& edge : edges) {
    if (edge.second.empty()) {
      continue;
    }
    bool parent_present = false;
    for (const auto& other : edges) {
      if (other.first == edge.second) {
        parent_present = true;
        break;
      }
    }
    if (!parent_present &&
        FindChildByPropertyKey(
            tree_cat, QStringLiteral("tree.") + QString::fromStdString(edge.first)) ==
            nullptr) {
      add_node(tree_cat, edge.first);
    }
  }

  tree_cat->setData(kDisplayTreeColName, kDisplayTreeRoleTfTreeFingerprint,
                    fingerprint);
  tree_cat->setData(kDisplayTreeColValue, kDisplayTreeRoleTfTreeFingerprint,
                    fingerprint);
  tree_cat->setExpanded(tree_expanded);
  RestoreExpandedTreeKeys(tree_cat, expanded_keys);
}

QString StatusSummaryText(int errors, int warns) {
  if (errors > 0) {
    return QObject::tr("%1 Error(s)").arg(errors);
  }
  if (warns > 0) {
    return QObject::tr("%1 Warn(s)").arg(warns);
  }
  return QObject::tr("Ok");
}

Qt::ItemFlags DisplayRowFlags() {
  return NameColumnFlags(false) | ValueColumnFlags(false, true) |
         Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled;
}

}  // namespace

DisplaysPanel::DisplaysPanel(
    std::shared_ptr<common::VisualizationManager> manager, QWidget* parent)
    : QWidget(parent), manager_(std::move(manager)) {
  ApplyPanelShell(this);
  setupUi();
  tf_pose_timer_ = new QTimer(this);
  tf_pose_timer_->setTimerType(Qt::PreciseTimer);
  tf_pose_timer_->setInterval(50);  // 20Hz for expanded Frames pose fields
  connect(tf_pose_timer_, &QTimer::timeout, this, &DisplaysPanel::onTfPoseTick);
  refresh();
}

void DisplaysPanel::setupUi() {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  splitter_ = new QSplitter(Qt::Vertical, this);
  splitter_->setObjectName(QStringLiteral("DisplayPanel/TreeWithHelp"));
  splitter_->setStyleSheet(PanelSplitterStyle());

  tree_ = new DisplayTreeWidget(manager_, splitter_);
  tree_->setObjectName(QStringLiteral("DisplayPanel/PropertyTree"));
  tree_->setColumnCount(2);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setIndentation(12);
  tree_->setIconSize(QSize(28, 28));
  tree_->setAnimated(true);
  tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->header()->setStretchLastSection(true);
  tree_->header()->setSectionResizeMode(kDisplayTreeColName,
                                        QHeaderView::ResizeToContents);
  tree_->setItemDelegateForColumn(kDisplayTreeColName,
                                  new DisplayNameTreeDelegate(tree_));
  StylePanelTree(tree_);

  help_ = new QTextBrowser(splitter_);
  help_->setObjectName(QStringLiteral("DisplayPanel/Help"));
  help_->setOpenExternalLinks(true);
  help_->setStyleSheet(PanelHelpBrowserStyle());

  splitter_->addWidget(tree_);
  splitter_->addWidget(help_);
  splitter_->setStretchFactor(0, 1000);
  splitter_->setCollapsible(0, false);
  splitter_->setSizes({1000, 1});

  layout->addWidget(splitter_, 1);

  auto* button_row = new QHBoxLayout();
  button_row->setContentsMargins(PanelChromeLayout::kToolbarMarginH, 4,
                                 PanelChromeLayout::kToolbarMarginH, 4);
  button_row->setSpacing(PanelChromeLayout::kToolbarSpacing);
  auto* add_button = new QPushButton(tr("Add"), this);
  add_button->setStyleSheet(PanelCompactButtonStyle());
  add_button->setShortcut(QKeySequence(QStringLiteral("Ctrl+N")));
  duplicate_button_ = new QPushButton(tr("Duplicate"), this);
  duplicate_button_->setStyleSheet(PanelCompactButtonStyle());
  duplicate_button_->setShortcut(QKeySequence(QStringLiteral("Ctrl+D")));
  duplicate_button_->setEnabled(false);
  remove_button_ = new QPushButton(tr("Remove"), this);
  remove_button_->setStyleSheet(PanelCompactButtonStyle());
  remove_button_->setShortcut(QKeySequence(QStringLiteral("Ctrl+X")));
  remove_button_->setEnabled(false);
  rename_button_ = new QPushButton(tr("Rename"), this);
  rename_button_->setStyleSheet(PanelCompactButtonStyle());
  rename_button_->setShortcut(QKeySequence(QStringLiteral("Ctrl+R")));
  rename_button_->setEnabled(false);
  button_row->addWidget(add_button);
  button_row->addWidget(duplicate_button_);
  button_row->addWidget(remove_button_);
  button_row->addWidget(rename_button_);
  layout->addLayout(button_row);

  value_delegate_ = new DisplayTreeDelegate(tree_);
  tree_->setItemDelegateForColumn(kDisplayTreeColValue, value_delegate_);
  // Avoid CurrentChanged/AnyKeyPressed: every selection change was opening an
  // editor, and closing it rewrote values (e.g. float formatting) → full scene
  // update on every property click.
  tree_->setEditTriggers(QAbstractItemView::DoubleClicked |
                         QAbstractItemView::SelectedClicked |
                         QAbstractItemView::EditKeyPressed);

  connect(add_button, &QPushButton::clicked, this, &DisplaysPanel::onAddDisplay);
  connect(duplicate_button_, &QPushButton::clicked, this,
          &DisplaysPanel::onDuplicateDisplay);
  connect(remove_button_, &QPushButton::clicked, this,
          &DisplaysPanel::onRemoveDisplay);
  connect(rename_button_, &QPushButton::clicked, this,
          &DisplaysPanel::onRenameDisplay);
  connect(tree_, &QTreeWidget::itemChanged, this,
          &DisplaysPanel::onDisplayItemChanged);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &DisplaysPanel::onDisplaySelectionChanged);
  // Expanding Frames / a frame row should fill pose fields immediately.
  connect(tree_, &QTreeWidget::itemExpanded, this, [this](QTreeWidgetItem* item) {
    if (updating_ || item == nullptr || manager_ == nullptr) {
      return;
    }
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind != DisplayTreeItemKind::kDisplay &&
        kind != DisplayTreeItemKind::kDisplayChild &&
        kind != DisplayTreeItemKind::kDisplayPropertyCategory &&
        kind != DisplayTreeItemKind::kDisplayFrameEnabled) {
      updateTfPoseTimerState();
      return;
    }
    QTreeWidgetItem* display_item = item;
    while (display_item != nullptr) {
      const auto k = static_cast<DisplayTreeItemKind>(
          display_item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
      if (k == DisplayTreeItemKind::kDisplay ||
          k == DisplayTreeItemKind::kDisplayChild) {
        break;
      }
      display_item = display_item->parent();
    }
    if (display_item == nullptr) {
      updateTfPoseTimerState();
      return;
    }
    const int index =
        display_item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex)
            .toInt();
    const int child_index =
        display_item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex)
            .toInt();
    display::Display* display =
        manager_->displayAt(static_cast<std::size_t>(index), child_index);
    if (auto* tf = dynamic_cast<display::TfDisplay*>(display)) {
      updating_ = true;
      syncTfDisplayProperties(display_item, tf);
      updating_ = false;
    }
    updateTfPoseTimerState();
  });
  connect(tree_, &QTreeWidget::itemCollapsed, this,
          [this](QTreeWidgetItem*) { updateTfPoseTimerState(); });
  connect(tree_, &DisplayTreeWidget::displaysReorganized, this, [this]() {
    refresh();
    emit displaysChanged();
  });
}

void DisplaysPanel::populateGlobalOptions() {
  auto* global = new QTreeWidgetItem(tree_);
  global->setText(kDisplayTreeColName, tr("Global Options"));
  global->setIcon(kDisplayTreeColName,
                  IconLoader::load(QStringLiteral(":/autoviz/icons/options.png")));
  global->setExpanded(true);
  global->setFlags(NameColumnFlags(false));
  SetTreeItemMeta(global, DisplayTreeItemKind::kGlobalOptions, -1, {},
                  tr("Configure the display frame, background, and other "
                     "global options."));

  auto* fixed = new QTreeWidgetItem(global);
  fixed->setText(kDisplayTreeColName, tr("Fixed Frame"));
  fixed->setText(kDisplayTreeColValue,
                 QString::fromStdString(manager_->fixedFrame()));
  fixed->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
  SetTreeItemMeta(fixed, DisplayTreeItemKind::kGlobalFixedFrame, -1, {},
                  tr("Frame into which all data is transformed before being "
                     "displayed."));

  auto* background = new QTreeWidgetItem(global);
  background->setText(kDisplayTreeColName, tr("Background Color"));
  background->setText(kDisplayTreeColValue,
                      QString::fromStdString(manager_->backgroundColor()));
  background->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
  SetTreeItemMeta(background, DisplayTreeItemKind::kGlobalBackgroundColor, -1,
                  {},
                  tr("Background color for the 3D view."));

  auto* show_grid = new QTreeWidgetItem(global);
  show_grid->setText(kDisplayTreeColName, tr("Show Grid"));
  show_grid->setCheckState(kDisplayTreeColValue,
                           manager_->showGrid() ? Qt::Checked : Qt::Unchecked);
  show_grid->setFlags(NameColumnFlags(false) | ValueColumnFlags(false, true));
  SetTreeItemMeta(show_grid, DisplayTreeItemKind::kGlobalShowGrid, -1, {},
                  tr("Show or hide the reference grid in the 3D view."));

  auto* frame_rate = new QTreeWidgetItem(global);
  frame_rate->setText(kDisplayTreeColName, tr("Frame Rate"));
  frame_rate->setText(kDisplayTreeColValue,
                      QString::number(manager_->targetFrameRate()));
  frame_rate->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
  SetTreeItemMeta(frame_rate, DisplayTreeItemKind::kGlobalFrameRate, -1, {},
                  tr("Target number of frames to try to render each second."));
}

void DisplaysPanel::updateGlobalStatus() {
  QTreeWidgetItem* status = findItemByKind(DisplayTreeItemKind::kGlobalStatus);
  if (status == nullptr) {
    return;
  }

  const bool was_expanded = status->isExpanded();

  int errors = 0;
  int warns = 0;
  QString issues_fingerprint;
  struct IssueRow {
    int index = -1;
    display::DisplayStatusLevel level = display::DisplayStatusLevel::kOk;
    QString name;
    QString message;
  };
  std::vector<IssueRow> issues;
  for (std::size_t i = 0; i < manager_->displays().size(); ++i) {
    const auto* display = manager_->displays()[i];
    if (!display->enabled()) {
      continue;
    }
    const display::DisplayStatus display_status = display->status();
    if (display_status.level != display::DisplayStatusLevel::kError &&
        display_status.level != display::DisplayStatusLevel::kWarn) {
      continue;
    }
    if (display_status.level == display::DisplayStatusLevel::kError) {
      ++errors;
    } else {
      ++warns;
    }
    IssueRow row;
    row.index = static_cast<int>(i);
    row.level = display_status.level;
    row.name = QString::fromStdString(display->name());
    row.message = QString::fromStdString(display_status.message);
    issues_fingerprint += row.name;
    issues_fingerprint += QLatin1Char('\t');
    issues_fingerprint += QString::number(static_cast<int>(row.level));
    issues_fingerprint += QLatin1Char('\t');
    issues_fingerprint += row.message;
    issues_fingerprint += QLatin1Char('\n');
    issues.push_back(std::move(row));
  }

  if (issues_fingerprint != global_status_fingerprint_) {
    global_status_fingerprint_ = issues_fingerprint;
    while (status->childCount() > 0) {
      delete status->takeChild(0);
    }
    for (const IssueRow& row : issues) {
      auto* child = new QTreeWidgetItem(status);
      child->setText(kDisplayTreeColName, row.name);
      child->setText(kDisplayTreeColValue, row.message);
      child->setIcon(kDisplayTreeColValue,
                     IconLoader::statusIcon(row.level, true));
      child->setFlags(NameColumnFlags(false));
      child->setForeground(
          kDisplayTreeColValue,
          QBrush(row.level == display::DisplayStatusLevel::kError
                     ? Qt::red
                     : QColor(200, 140, 0)));
      SetTreeItemMeta(child, DisplayTreeItemKind::kGlobalStatusMessage,
                      row.index);
    }
  }

  const QString summary = StatusSummaryText(errors, warns);
  if (status->text(kDisplayTreeColValue) != summary) {
    status->setText(kDisplayTreeColValue, summary);
  }
  if (errors > 0) {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kError,
                                           true));
    status->setForeground(kDisplayTreeColValue, QBrush(Qt::red));
  } else if (warns > 0) {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kWarn,
                                           true));
    status->setForeground(kDisplayTreeColValue, QBrush(QColor(200, 140, 0)));
  } else {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kOk,
                                           true));
    status->setForeground(kDisplayTreeColValue,
                          tree_->palette().brush(QPalette::Text));
  }

  const int issue_count = errors + warns;
  if (issue_count > global_status_issue_count_) {
    status->setExpanded(true);
  } else {
    status->setExpanded(was_expanded);
  }
  global_status_issue_count_ = issue_count;
}

void DisplaysPanel::populateGlobalStatus() {
  auto* status = new QTreeWidgetItem(tree_);
  status->setText(kDisplayTreeColName, tr("Global Status"));
  status->setIcon(
      kDisplayTreeColName,
      IconLoader::load(QStringLiteral(":/autoviz/icons/global_status.svg")));
  status->setFlags(NameColumnFlags(false));
  SetTreeItemMeta(status, DisplayTreeItemKind::kGlobalStatus, -1, {},
                  tr("Shows aggregated errors and warnings from displays."));
  updateGlobalStatus();
}

void DisplaysPanel::populateDisplayProperties(QTreeWidgetItem* display_item,
                                              display::Display* display,
                                              std::size_t index,
                                              int child_index) {
  auto* status = new QTreeWidgetItem(display_item);
  status->setText(kDisplayTreeColName, tr("Status"));
  status->setFlags(NameColumnFlags(false));
  const display::DisplayStatus display_status = display->status();
  status->setIcon(kDisplayTreeColValue,
                  IconLoader::statusIcon(display_status.level, display->enabled()));
  status->setText(kDisplayTreeColValue,
                  display_status.message.empty()
                      ? tr("Ok")
                      : QString::fromStdString(display_status.message));
  SetTreeItemMeta(status, DisplayTreeItemKind::kDisplayStatus,
                  static_cast<int>(index), {}, {}, child_index);

  if (!display->channel().empty()) {
    auto* channel = new QTreeWidgetItem(display_item);
    const QString channel_label = tr("Channel");
    channel->setText(kDisplayTreeColName, channel_label);
    channel->setText(kDisplayTreeColValue,
                     QString::fromStdString(display->channel()));
    channel->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
    SetTreeItemMeta(channel, DisplayTreeItemKind::kDisplayChannel,
                    static_cast<int>(index), QStringLiteral("channel"),
                    tr("Topic or channel name for incoming data."), child_index);
    const QString joined =
        CompatibleChannelsForDisplay(*manager_, *display).join(QLatin1Char('\n'));
    channel->setData(kDisplayTreeColName, kDisplayTreeRoleChannelOptions, joined);
    channel->setData(kDisplayTreeColValue, kDisplayTreeRoleChannelOptions, joined);
  }

  for (const auto& spec : display->propertySpecs()) {
    const QString current_value = QString::fromStdString(
        display->propertyValue(spec.key, spec.default_value));
    auto* prop = new QTreeWidgetItem(display_item);
    prop->setText(kDisplayTreeColName, QString::fromStdString(spec.label));
    const QString default_lower =
        QString::fromStdString(spec.default_value).trimmed().toLower();
    const bool is_bool_property =
        default_lower == QLatin1String("true") ||
        default_lower == QLatin1String("false");
    if (is_bool_property) {
      // RViz-style: Show Names / Show Axes / … as value-column checkboxes.
      prop->setText(kDisplayTreeColValue, QString());
      prop->setFlags(NameColumnFlags(false) | ValueColumnFlags(false, true));
      const bool checked = common::ParseBoolProperty(
          current_value.toStdString(), default_lower == QLatin1String("true"));
      prop->setCheckState(kDisplayTreeColValue,
                          checked ? Qt::Checked : Qt::Unchecked);
    } else {
      prop->setText(kDisplayTreeColValue, current_value);
      prop->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
    }
    prop->setData(kDisplayTreeColName, Qt::UserRole + 10, default_lower);
    prop->setData(kDisplayTreeColValue, Qt::UserRole + 10, default_lower);
    prop->setData(kDisplayTreeColName, kDisplayTreeRolePropertyKind,
                  static_cast<int>(spec.kind));
    prop->setData(kDisplayTreeColValue, kDisplayTreeRolePropertyKind,
                  static_cast<int>(spec.kind));
    if (!spec.options.empty()) {
      QStringList options;
      for (const auto& option : spec.options) {
        options.push_back(QString::fromStdString(option));
      }
      const QString joined = options.join('\n');
      prop->setData(kDisplayTreeColName, Qt::UserRole + 11, joined);
      prop->setData(kDisplayTreeColValue, Qt::UserRole + 11, joined);
    }
    SetTreeItemMeta(prop, DisplayTreeItemKind::kDisplayProperty,
                    static_cast<int>(index),
                    QString::fromStdString(spec.key), {}, child_index);
  }

  if (display->typeId() == "TF") {
    auto* frames = new QTreeWidgetItem(display_item);
    frames->setText(kDisplayTreeColName, tr("Frames"));
    frames->setFlags(NameColumnFlags(false));
    SetTreeItemMeta(frames, DisplayTreeItemKind::kDisplayPropertyCategory,
                    static_cast<int>(index), QStringLiteral("frames"),
                    tr("Per-frame enable and pose (relative to Fixed Frame)."),
                    child_index);

    auto* all_enabled = new QTreeWidgetItem(frames);
    all_enabled->setText(kDisplayTreeColName, tr("All Enabled"));
    all_enabled->setFlags(NameColumnFlags(false) |
                          ValueColumnFlags(false, true));
    all_enabled->setCheckState(kDisplayTreeColValue, Qt::Checked);
    SetTreeItemMeta(all_enabled, DisplayTreeItemKind::kDisplayAllFramesEnabled,
                    static_cast<int>(index), QStringLiteral("frames.all_enabled"),
                    tr("Enable or disable all TF frames."), child_index);

    auto* tree = new QTreeWidgetItem(display_item);
    tree->setText(kDisplayTreeColName, tr("Tree"));
    tree->setFlags(NameColumnFlags(false));
    SetTreeItemMeta(tree, DisplayTreeItemKind::kDisplayPropertyCategory,
                    static_cast<int>(index), QStringLiteral("tree"),
                    tr("TF parent hierarchy (read-only)."), child_index);

    if (auto* tf = dynamic_cast<display::TfDisplay*>(display)) {
      syncTfDisplayProperties(display_item, tf);
    }
  }
}

void DisplaysPanel::populateDisplays() {
  const auto& displays = manager_->displays();
  for (std::size_t i = 0; i < displays.size(); ++i) {
    display::Display* display = displays[i];
    auto* item = new QTreeWidgetItem(tree_);
    item->setText(kDisplayTreeColName, QString::fromStdString(display->name()));
    item->setIcon(kDisplayTreeColName,
                  IconLoader::displayIcon(QString::fromStdString(display->typeId())));
    item->setFlags(DisplayRowFlags());
    item->setCheckState(kDisplayTreeColValue,
                        display->enabled() ? Qt::Checked : Qt::Unchecked);
    item->setExpanded(display->typeId() == "Grid" || display->typeId() == "Group");
    SetTreeItemMeta(item, DisplayTreeItemKind::kDisplay, static_cast<int>(i),
                    {}, tr("Drag onto a Group to nest, or above/below to "
                           "reorder. Use the grip on the left."));
    populateDisplayProperties(item, display, i, -1);

    if (auto* group = dynamic_cast<display::DisplayGroup*>(display)) {
      for (std::size_t ci = 0; ci < group->children().size(); ++ci) {
        display::Display* child = group->child(ci);
        if (child == nullptr) {
          continue;
        }
        auto* child_item = new QTreeWidgetItem(item);
        child_item->setText(kDisplayTreeColName,
                            QString::fromStdString(child->name()));
        child_item->setIcon(
            kDisplayTreeColName,
            IconLoader::displayIcon(QString::fromStdString(child->typeId())));
        child_item->setFlags(DisplayRowFlags());
        child_item->setCheckState(kDisplayTreeColValue,
                                  child->enabled() ? Qt::Checked : Qt::Unchecked);
        child_item->setExpanded(false);
        SetTreeItemMeta(child_item, DisplayTreeItemKind::kDisplayChild,
                        static_cast<int>(i), {},
                        tr("Drag to reorder or move into another Group."),
                        static_cast<int>(ci));
        populateDisplayProperties(child_item, child, i, static_cast<int>(ci));
      }
    }
  }
}

void DisplaysPanel::populateTree() {
  tree_->clear();
  populateGlobalOptions();
  populateGlobalStatus();
  populateDisplays();
}

void DisplaysPanel::updateChannelDelegate() {
  QStringList channels;
  for (const auto& name : manager_->channelNames()) {
    channels.push_back(QString::fromStdString(name));
  }
  channels.sort(Qt::CaseInsensitive);
  value_delegate_->setChannels(channels);
}

void DisplaysPanel::refresh() {
  updating_ = true;
  global_status_issue_count_ = 0;
  channel_list_fingerprint_.clear();
  global_status_fingerprint_.clear();
  updateChannelDelegate();
  for (const auto& name : manager_->channelNames()) {
    channel_list_fingerprint_ += QString::fromStdString(name);
    channel_list_fingerprint_ += QLatin1Char('\n');
  }
  populateTree();
  if (QTreeWidgetItem* global = findItemByKind(DisplayTreeItemKind::kGlobalOptions)) {
    global->setExpanded(true);
  }
  updating_ = false;
}

void DisplaysPanel::syncTfDisplayProperties(QTreeWidgetItem* display_item,
                                            display::TfDisplay* tf) {
  if (display_item == nullptr || tf == nullptr) {
    return;
  }
  // Collapsed TF row: skip Frames/Tree churn (major UI hitch with moving poses).
  if (!display_item->isExpanded()) {
    return;
  }

  QTreeWidgetItem* frames_cat =
      FindChildByPropertyKey(display_item, QStringLiteral("frames"));
  QTreeWidgetItem* tree_cat =
      FindChildByPropertyKey(display_item, QStringLiteral("tree"));
  if (frames_cat == nullptr || tree_cat == nullptr) {
    return;
  }

  // Creating checkable items emits itemChanged with Unchecked by default;
  // block so we never persist a spurious All Enabled / per-frame disable.
  const QSignalBlocker blocker(tree_);

  const int display_index =
      display_item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex)
          .toInt();
  const int child_index =
      display_item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();

  QTreeWidgetItem* all_enabled =
      FindChildByPropertyKey(frames_cat, QStringLiteral("frames.all_enabled"));
  if (all_enabled != nullptr) {
    const Qt::CheckState want =
        tf->allFramesEnabled() ? Qt::Checked : Qt::Unchecked;
    if (all_enabled->checkState(kDisplayTreeColValue) != want) {
      all_enabled->setCheckState(kDisplayTreeColValue, want);
    }
  }

  const auto snapshots = tf->frameSnapshots();
  std::map<std::string, const display::TfFrameSnapshot*> by_name;
  for (const auto& snap : snapshots) {
    by_name.emplace(snap.name, &snap);
  }

  // Frame list / checkboxes only need work when Frames is open (or first sync).
  const bool sync_frame_rows = frames_cat->isExpanded() ||
                               frames_cat->childCount() <= 1;

  if (sync_frame_rows) {
    for (int i = frames_cat->childCount() - 1; i >= 0; --i) {
      QTreeWidgetItem* child = frames_cat->child(i);
      const auto kind = static_cast<DisplayTreeItemKind>(
          child->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
      if (kind != DisplayTreeItemKind::kDisplayFrameEnabled) {
        continue;
      }
      const QString key =
          child->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey)
              .toString();
      static const QString kPrefix = QStringLiteral("frames.");
      if (!key.startsWith(kPrefix) ||
          key == QStringLiteral("frames.all_enabled")) {
        continue;
      }
      const std::string frame_id = key.mid(kPrefix.size()).toStdString();
      if (by_name.find(frame_id) == by_name.end()) {
        delete frames_cat->takeChild(i);
      }
    }

    for (const auto& snap : snapshots) {
      const QString key =
          QStringLiteral("frames.") + QString::fromStdString(snap.name);
      QTreeWidgetItem* frame_item = FindChildByPropertyKey(frames_cat, key);
      if (frame_item == nullptr) {
        frame_item = new QTreeWidgetItem(frames_cat);
        frame_item->setText(kDisplayTreeColName,
                            QString::fromStdString(snap.name));
        frame_item->setFlags(NameColumnFlags(false) |
                             ValueColumnFlags(false, true));
        SetTreeItemMeta(frame_item, DisplayTreeItemKind::kDisplayFrameEnabled,
                        display_index, key, {}, child_index);
      }
      const Qt::CheckState want = snap.enabled ? Qt::Checked : Qt::Unchecked;
      if (frame_item->checkState(kDisplayTreeColValue) != want) {
        frame_item->setCheckState(kDisplayTreeColValue, want);
      }

      // Parent rarely changes; always cheap to keep in sync when row exists.
      EnsureReadonlyField(frame_item, display_index, child_index,
                          key + ".parent", tr("Parent"),
                          QString::fromStdString(snap.parent));

      // Pose numbers: initial fill when expanded; realtime via tf_pose_timer_.
      if (!frame_item->isExpanded()) {
        continue;
      }
      SyncTfFramePoseFields(frame_item, display_index, child_index, key, snap,
                            tr("Position"), tr("Orientation"),
                            tr("Relative Position"), tr("Relative Orientation"),
                            tr("<unavailable>"));
    }
  }

  if (tree_cat->isExpanded() || tree_cat->childCount() == 0) {
    RebuildTfTreeCategory(tree_cat, display_index, child_index, tf->treeEdges());
  }
  updateTfPoseTimerState();
}

void DisplaysPanel::setLiveUpdatesPaused(bool paused) {
  live_updates_paused_ = paused;
  if (paused) {
    if (tf_pose_timer_ != nullptr) {
      tf_pose_timer_->stop();
    }
    return;
  }
  updateTfPoseTimerState();
}

void DisplaysPanel::updateTfPoseTimerState() {
  if (tf_pose_timer_ == nullptr || tree_ == nullptr || live_updates_paused_) {
    if (tf_pose_timer_ != nullptr && live_updates_paused_) {
      tf_pose_timer_->stop();
    }
    return;
  }
  bool need = false;
  std::function<void(QTreeWidgetItem*)> walk = [&](QTreeWidgetItem* item) {
    if (item == nullptr || need) {
      return;
    }
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay ||
        kind == DisplayTreeItemKind::kDisplayChild) {
      if (TfDisplayHasExpandedPoseRows(item)) {
        need = true;
        return;
      }
    }
    for (int i = 0; i < item->childCount(); ++i) {
      walk(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    walk(tree_->topLevelItem(i));
  }
  if (need) {
    if (!tf_pose_timer_->isActive()) {
      tf_pose_timer_->start();
    }
  } else if (tf_pose_timer_->isActive()) {
    tf_pose_timer_->stop();
  }
}

void DisplaysPanel::onTfPoseTick() { syncTfExpandedPoseFields(); }

void DisplaysPanel::syncTfExpandedPoseFields() {
  if (live_updates_paused_ || updating_ || tree_ == nullptr ||
      manager_ == nullptr || tree_->isEditing()) {
    return;
  }

  const QString position_label = tr("Position");
  const QString orientation_label = tr("Orientation");
  const QString rel_position_label = tr("Relative Position");
  const QString rel_orientation_label = tr("Relative Orientation");
  const QString unavailable = tr("<unavailable>");

  std::function<void(QTreeWidgetItem*)> walk = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay ||
        kind == DisplayTreeItemKind::kDisplayChild) {
      if (!TfDisplayHasExpandedPoseRows(item)) {
        for (int i = 0; i < item->childCount(); ++i) {
          walk(item->child(i));
        }
        return;
      }
      const int index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
      const int child_index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
      display::Display* display =
          manager_->displayAt(static_cast<std::size_t>(index), child_index);
      auto* tf = dynamic_cast<display::TfDisplay*>(display);
      if (tf == nullptr) {
        return;
      }
      QTreeWidgetItem* frames_cat =
          FindChildByPropertyKey(item, QStringLiteral("frames"));
      if (frames_cat == nullptr) {
        return;
      }
      const auto snapshots = tf->frameSnapshots();
      std::map<std::string, const display::TfFrameSnapshot*> by_name;
      for (const auto& snap : snapshots) {
        by_name.emplace(snap.name, &snap);
      }
      for (int i = 0; i < frames_cat->childCount(); ++i) {
        QTreeWidgetItem* frame_item = frames_cat->child(i);
        const auto frame_kind = static_cast<DisplayTreeItemKind>(
            frame_item->data(kDisplayTreeColName, kDisplayTreeRoleKind)
                .toInt());
        if (frame_kind != DisplayTreeItemKind::kDisplayFrameEnabled ||
            !frame_item->isExpanded()) {
          continue;
        }
        const QString key =
            frame_item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey)
                .toString();
        static const QString kPrefix = QStringLiteral("frames.");
        if (!key.startsWith(kPrefix)) {
          continue;
        }
        const std::string frame_id = key.mid(kPrefix.size()).toStdString();
        const auto it = by_name.find(frame_id);
        if (it == by_name.end() || it->second == nullptr) {
          continue;
        }
        SyncTfFramePoseFields(frame_item, index, child_index, key, *it->second,
                              position_label, orientation_label,
                              rel_position_label, rel_orientation_label,
                              unavailable);
      }
      return;
    }
    for (int i = 0; i < item->childCount(); ++i) {
      walk(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    walk(tree_->topLevelItem(i));
  }
}

void DisplaysPanel::refreshStatus() {
  // Do not touch the tree while an in-place editor is open — setText/setData
  // would close/commit the editor and make property clicks feel frozen.
  if (tree_ != nullptr && tree_->isEditing()) {
    return;
  }

  updating_ = true;

  QString channel_fingerprint;
  for (const auto& name : manager_->channelNames()) {
    channel_fingerprint += QString::fromStdString(name);
    channel_fingerprint += QLatin1Char('\n');
  }
  const bool channels_changed = channel_fingerprint != channel_list_fingerprint_;
  if (channels_changed) {
    channel_list_fingerprint_ = channel_fingerprint;
    updateChannelDelegate();
  }

  int display_items = 0;
  std::function<void(QTreeWidgetItem*)> walk = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay) {
      ++display_items;
    }
    if (kind == DisplayTreeItemKind::kDisplay ||
        kind == DisplayTreeItemKind::kDisplayChild) {
      const int index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
      const int child_index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
      display::Display* display =
          manager_->displayAt(static_cast<std::size_t>(index), child_index);
      if (auto* tf = dynamic_cast<display::TfDisplay*>(display)) {
        syncTfDisplayProperties(item, tf);
      }
    }
    if (kind == DisplayTreeItemKind::kDisplayStatus ||
        kind == DisplayTreeItemKind::kDisplayChannel) {
      const int index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
      const int child_index =
          item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
      display::Display* display =
          manager_->displayAt(static_cast<std::size_t>(index), child_index);
      if (display != nullptr) {
        if (kind == DisplayTreeItemKind::kDisplayStatus) {
          const display::DisplayStatus display_status = display->status();
          const QString text = display_status.message.empty()
                                   ? tr("Ok")
                                   : QString::fromStdString(display_status.message);
          if (item->text(kDisplayTreeColValue) != text) {
            item->setText(kDisplayTreeColValue, text);
          }
          item->setIcon(kDisplayTreeColValue,
                        IconLoader::statusIcon(display_status.level,
                                               display->enabled()));
        } else if (channels_changed) {
          const QString joined =
              CompatibleChannelsForDisplay(*manager_, *display)
                  .join(QLatin1Char('\n'));
          if (item->data(kDisplayTreeColValue, kDisplayTreeRoleChannelOptions)
                  .toString() != joined) {
            item->setData(kDisplayTreeColName, kDisplayTreeRoleChannelOptions,
                          joined);
            item->setData(kDisplayTreeColValue, kDisplayTreeRoleChannelOptions,
                          joined);
          }
        }
      }
    }
    for (int i = 0; i < item->childCount(); ++i) {
      walk(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    walk(tree_->topLevelItem(i));
  }

  if (display_items != static_cast<int>(manager_->displays().size())) {
    updating_ = false;
    refresh();
    return;
  }
  updateGlobalStatus();
  updating_ = false;
}

QTreeWidgetItem* DisplaysPanel::findItemByKind(
    DisplayTreeItemKind kind, int display_index,
    const QString& property_key) const {
  std::function<QTreeWidgetItem*(QTreeWidgetItem*)> search =
      [&](QTreeWidgetItem* item) -> QTreeWidgetItem* {
    if (item == nullptr) {
      return nullptr;
    }
    const auto item_kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    const int item_index =
        item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
    const QString item_key =
        item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey).toString();
    if (item_kind == kind && item_index == display_index &&
        (property_key.isEmpty() || item_key == property_key)) {
      return item;
    }
    for (int i = 0; i < item->childCount(); ++i) {
      if (QTreeWidgetItem* found = search(item->child(i))) {
        return found;
      }
    }
    return nullptr;
  };

  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    if (QTreeWidgetItem* found = search(tree_->topLevelItem(i))) {
      return found;
    }
  }
  return nullptr;
}

void DisplaysPanel::updateHelp(QTreeWidgetItem* item) {
  if (item == nullptr || help_ == nullptr) {
    help_->clear();
    return;
  }
  const QString heading = item->text(kDisplayTreeColName);
  const QString body =
      item->data(kDisplayTreeColName, kDisplayTreeRoleDescription).toString();
  if (body.isEmpty()) {
    help_->setHtml(QString());
    return;
  }
  help_->setHtml(QStringLiteral("<html><body><strong>%1</strong><br>%2</body></html>")
                     .arg(heading.toHtmlEscaped(), body.toHtmlEscaped()));
}

void DisplaysPanel::updateActionButtons() {
  QTreeWidgetItem* item = tree_->currentItem();
  const auto kind = item == nullptr
                        ? DisplayTreeItemKind::kGlobalOptions
                        : static_cast<DisplayTreeItemKind>(
                              item->data(kDisplayTreeColName, kDisplayTreeRoleKind)
                                  .toInt());
  const bool display_selected =
      kind == DisplayTreeItemKind::kDisplay ||
      kind == DisplayTreeItemKind::kDisplayChild;
  duplicate_button_->setEnabled(display_selected);
  remove_button_->setEnabled(display_selected);
  rename_button_->setEnabled(display_selected);
}

void DisplaysPanel::onDisplaySelectionChanged() {
  updateHelp(tree_->currentItem());
  updateActionButtons();
}

void DisplaysPanel::applyPropertyValue(QTreeWidgetItem* item,
                                         const QString& value) {
  const auto kind = static_cast<DisplayTreeItemKind>(
      item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
  const int display_index =
      item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
  const QString property_key =
      item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey).toString();

  if (kind == DisplayTreeItemKind::kGlobalFixedFrame) {
    const QString frame = value.trimmed();
    if (frame.isEmpty() ||
        frame.toStdString() == manager_->fixedFrame()) {
      return;
    }
    manager_->setFixedFrame(frame.toStdString());
    emit fixedFrameChanged(frame);
    return;
  }

  if (kind == DisplayTreeItemKind::kGlobalBackgroundColor) {
    if (value.toStdString() == manager_->backgroundColor()) {
      return;
    }
    manager_->setBackgroundColor(value.toStdString());
    emit backgroundColorChanged(
        common::ParseColorProperty(value.toStdString(), QColor(48, 48, 48)));
    return;
  }

  if (kind == DisplayTreeItemKind::kGlobalFrameRate) {
    bool ok = false;
    const int rate = value.trimmed().toInt(&ok);
    if (!ok || rate < 1 || rate == manager_->targetFrameRate()) {
      return;
    }
    manager_->setTargetFrameRate(rate);
    return;
  }

  if (display_index < 0) {
    return;
  }

  const int child_index =
      item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
  display::Display* display =
      manager_->displayAt(static_cast<std::size_t>(display_index), child_index);
  if (display == nullptr) {
    return;
  }

  if (kind == DisplayTreeItemKind::kDisplayChannel) {
    if (value.toStdString() == display->channel()) {
      return;
    }
    manager_->setDisplayChannel(static_cast<std::size_t>(display_index),
                                value.toStdString(), child_index);
    emit displaysChanged();
    return;
  }

  if (kind == DisplayTreeItemKind::kDisplayProperty) {
    const std::string key = property_key.toStdString();
    const std::string new_value = value.toStdString();
    if (new_value == display->propertyValue(key, {})) {
      return;
    }
    manager_->setDisplayProperty(static_cast<std::size_t>(display_index), key,
                                 new_value, child_index);
    emit displaysChanged();
  }
}

void DisplaysPanel::onDisplayItemChanged(QTreeWidgetItem* item, int column) {
  if (updating_ || item == nullptr) {
    return;
  }

  const auto kind = static_cast<DisplayTreeItemKind>(
      item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
  const int display_index =
      item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();

  if (kind == DisplayTreeItemKind::kGlobalShowGrid &&
      column == kDisplayTreeColValue) {
    const bool enabled = item->checkState(kDisplayTreeColValue) == Qt::Checked;
    manager_->setShowGrid(enabled);
    for (std::size_t i = 0; i < manager_->displays().size(); ++i) {
      if (manager_->displays()[i]->typeId() == "Grid") {
        manager_->setDisplayEnabled(i, enabled);
        for (int row = 0; row < tree_->topLevelItemCount(); ++row) {
          QTreeWidgetItem* top = tree_->topLevelItem(row);
          if (top == nullptr) {
            continue;
          }
          const auto top_kind = static_cast<DisplayTreeItemKind>(
              top->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
          if (top_kind != DisplayTreeItemKind::kDisplay) {
            continue;
          }
          if (top->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex)
                  .toInt() == static_cast<int>(i)) {
            top->setCheckState(kDisplayTreeColValue,
                               enabled ? Qt::Checked : Qt::Unchecked);
            break;
          }
        }
        break;
      }
    }
    emit displaysChanged();
    return;
  }

  if (kind == DisplayTreeItemKind::kDisplayProperty &&
      column == kDisplayTreeColValue &&
      (item->flags() & Qt::ItemIsUserCheckable)) {
    const QString default_lower =
        item->data(kDisplayTreeColName, Qt::UserRole + 10).toString();
    if (default_lower == QLatin1String("true") ||
        default_lower == QLatin1String("false")) {
      const bool checked =
          item->checkState(kDisplayTreeColValue) == Qt::Checked;
      applyPropertyValue(item, checked ? QStringLiteral("true")
                                       : QStringLiteral("false"));
      return;
    }
  }

  if ((kind == DisplayTreeItemKind::kDisplayAllFramesEnabled ||
       kind == DisplayTreeItemKind::kDisplayFrameEnabled) &&
      column == kDisplayTreeColValue) {
    const int child_index =
        item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
    display::Display* display =
        manager_->displayAt(static_cast<std::size_t>(display_index), child_index);
    auto* tf = dynamic_cast<display::TfDisplay*>(display);
    if (tf == nullptr) {
      return;
    }
    const bool enabled = item->checkState(kDisplayTreeColValue) == Qt::Checked;
    if (kind == DisplayTreeItemKind::kDisplayAllFramesEnabled) {
      if (tf->allFramesEnabled() == enabled) {
        return;
      }
      tf->setAllFramesEnabled(enabled);
    } else {
      const QString key =
          item->data(kDisplayTreeColName, kDisplayTreeRolePropertyKey).toString();
      static const QString kPrefix = QStringLiteral("frames.");
      if (!key.startsWith(kPrefix)) {
        return;
      }
      const std::string frame_id = key.mid(kPrefix.size()).toStdString();
      tf->setFrameEnabled(frame_id, enabled);
    }
    emit displaysChanged();
    return;
  }

  if ((kind == DisplayTreeItemKind::kDisplay ||
       kind == DisplayTreeItemKind::kDisplayChild) &&
      column == kDisplayTreeColValue) {
    const int child_index =
        item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
    const bool enabled = item->checkState(kDisplayTreeColValue) == Qt::Checked;
    if (const display::Display* display =
            manager_->displayAt(static_cast<std::size_t>(display_index),
                                child_index);
        display != nullptr && display->enabled() == enabled) {
      return;
    }
    manager_->setDisplayEnabled(static_cast<std::size_t>(display_index), enabled,
                                child_index);
    if (kind == DisplayTreeItemKind::kDisplay && display_index >= 0 &&
        static_cast<std::size_t>(display_index) < manager_->displays().size()) {
      const auto* display =
          manager_->displays()[static_cast<std::size_t>(display_index)];
      if (display->typeId() == "Grid") {
        manager_->setShowGrid(enabled);
        if (QTreeWidgetItem* show_grid =
                findItemByKind(DisplayTreeItemKind::kGlobalShowGrid)) {
          show_grid->setCheckState(kDisplayTreeColValue,
                                   enabled ? Qt::Checked : Qt::Unchecked);
        }
      }
    }
    refreshStatus();
    emit displaysChanged();
    return;
  }

  if (column == kDisplayTreeColValue) {
    applyPropertyValue(item, item->text(kDisplayTreeColValue));
  }
}

std::string DisplaysPanel::uniqueDisplayName(const std::string& base) const {
  std::vector<std::string> names;
  for (const auto* display : manager_->displays()) {
    names.push_back(display->name());
  }
  const auto exists = [&names](const std::string& candidate) {
    return std::find(names.begin(), names.end(), candidate) != names.end();
  };
  if (!exists(base)) {
    return base;
  }
  for (int suffix = 2; suffix < 1000; ++suffix) {
    const std::string candidate = base + " " + std::to_string(suffix);
    if (!exists(candidate)) {
      return candidate;
    }
  }
  return base + " copy";
}

void DisplaysPanel::onAddDisplay() {
  QStringList existing_names;
  for (const auto* display : manager_->displays()) {
    existing_names.push_back(QString::fromStdString(display->name()));
  }
  AddDisplayDialog dialog(manager_, existing_names, this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const QString type_q = dialog.selectedType();
  if (type_q.isEmpty()) {
    return;
  }
  auto config = common::DisplayFactory::defaultForType(type_q.toStdString());
  config.name = dialog.selectedName().toStdString();
  if (!dialog.selectedChannel().isEmpty()) {
    config.channel = dialog.selectedChannel().toStdString();
  }
  config.name = uniqueDisplayName(config.name);
  if (!manager_->addDisplay(config)) {
    return;
  }
  refresh();
  emit displaysChanged();
}

void DisplaysPanel::onDuplicateDisplay() {
  QTreeWidgetItem* item = tree_->currentItem();
  while (item != nullptr) {
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay) {
      break;
    }
    item = item->parent();
  }
  if (item == nullptr) {
    return;
  }
  const int index = item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
  if (index < 0) {
    return;
  }
  if (!manager_->duplicateDisplay(static_cast<std::size_t>(index))) {
    return;
  }
  const std::size_t new_index = manager_->displays().size() - 1;
  const std::string unique_name = uniqueDisplayName(
      manager_->displays()[new_index]->name());
  manager_->setDisplayName(new_index, unique_name);
  refresh();
  emit displaysChanged();
}

void DisplaysPanel::onRenameDisplay() {
  QTreeWidgetItem* item = tree_->currentItem();
  while (item != nullptr) {
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay) {
      break;
    }
    item = item->parent();
  }
  if (item == nullptr) {
    return;
  }
  const int index = item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
  if (index < 0 ||
      static_cast<std::size_t>(index) >= manager_->displays().size()) {
    return;
  }
  const QString current =
      QString::fromStdString(manager_->displays()[static_cast<std::size_t>(index)]->name());
  bool ok = false;
  const QString next = QInputDialog::getText(
      this, tr("Rename Display"), tr("Display name:"), QLineEdit::Normal,
      current, &ok);
  if (!ok || next.trimmed().isEmpty()) {
    return;
  }
  manager_->setDisplayName(static_cast<std::size_t>(index),
                           next.trimmed().toStdString());
  refresh();
  emit displaysChanged();
}

void DisplaysPanel::onRemoveDisplay() {
  QTreeWidgetItem* item = tree_->currentItem();
  while (item != nullptr) {
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kDisplay ||
        kind == DisplayTreeItemKind::kDisplayChild) {
      break;
    }
    item = item->parent();
  }
  if (item == nullptr) {
    return;
  }
  const int index = item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
  const int child_index =
      item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
  if (index < 0) {
    return;
  }
  if (!manager_->removeDisplay(static_cast<std::size_t>(index), child_index)) {
    return;
  }
  refresh();
  emit displaysChanged();
}

}  // namespace autoviz
