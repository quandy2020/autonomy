/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/displays_panel.hpp"

#include <algorithm>

#include <QBrush>
#include <QDialog>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QVBoxLayout>

#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/display_status.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/ui/add_display_dialog.hpp"
#include "autoviz/ui/display_tree_delegate.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

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
  tree_->setEditTriggers(QAbstractItemView::AllEditTriggers);

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

  while (status->childCount() > 0) {
    delete status->takeChild(0);
  }

  int errors = 0;
  int warns = 0;
  for (std::size_t i = 0; i < manager_->displays().size(); ++i) {
    const auto* display = manager_->displays()[i];
    if (!display->enabled()) {
      continue;
    }
    const display::DisplayStatus display_status = display->status();
    if (display_status.level == display::DisplayStatusLevel::kError) {
      ++errors;
      auto* child = new QTreeWidgetItem(status);
      child->setText(kDisplayTreeColName,
                     QString::fromStdString(display->name()));
      child->setText(kDisplayTreeColValue,
                     QString::fromStdString(display_status.message));
      child->setIcon(kDisplayTreeColValue,
                     IconLoader::statusIcon(display_status.level, true));
      child->setFlags(NameColumnFlags(false));
      child->setForeground(kDisplayTreeColValue, QBrush(Qt::red));
      SetTreeItemMeta(child, DisplayTreeItemKind::kGlobalStatusMessage,
                      static_cast<int>(i));
    } else if (display_status.level == display::DisplayStatusLevel::kWarn) {
      ++warns;
      auto* child = new QTreeWidgetItem(status);
      child->setText(kDisplayTreeColName,
                     QString::fromStdString(display->name()));
      child->setText(kDisplayTreeColValue,
                     QString::fromStdString(display_status.message));
      child->setIcon(kDisplayTreeColValue,
                     IconLoader::statusIcon(display_status.level, true));
      child->setFlags(NameColumnFlags(false));
      child->setForeground(kDisplayTreeColValue, QBrush(QColor(200, 140, 0)));
      SetTreeItemMeta(child, DisplayTreeItemKind::kGlobalStatusMessage,
                      static_cast<int>(i));
    }
  }

  if (errors > 0) {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kError,
                                           true));
    status->setText(kDisplayTreeColValue, StatusSummaryText(errors, warns));
    status->setForeground(kDisplayTreeColValue, QBrush(Qt::red));
  } else if (warns > 0) {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kWarn,
                                           true));
    status->setText(kDisplayTreeColValue, StatusSummaryText(errors, warns));
    status->setForeground(kDisplayTreeColValue, QBrush(QColor(200, 140, 0)));
  } else {
    status->setIcon(kDisplayTreeColValue,
                    IconLoader::statusIcon(display::DisplayStatusLevel::kOk,
                                           true));
    status->setText(kDisplayTreeColValue, StatusSummaryText(errors, warns));
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
  }

  for (const auto& spec : display->propertySpecs()) {
    const QString current_value = QString::fromStdString(
        display->propertyValue(spec.key, spec.default_value));
    auto* prop = new QTreeWidgetItem(display_item);
    prop->setText(kDisplayTreeColName, QString::fromStdString(spec.label));
    prop->setText(kDisplayTreeColValue, current_value);
    prop->setFlags(NameColumnFlags(false) | ValueColumnFlags(true));
    const QString default_lower =
        QString::fromStdString(spec.default_value).trimmed().toLower();
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
  updateChannelDelegate();
  populateTree();
  if (QTreeWidgetItem* global = findItemByKind(DisplayTreeItemKind::kGlobalOptions)) {
    global->setExpanded(true);
  }
  updating_ = false;
}

void DisplaysPanel::refreshStatus() {
  updating_ = true;
  updateChannelDelegate();

  int display_items = 0;
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = tree_->topLevelItem(i);
    if (item == nullptr) {
      continue;
    }
    const auto kind = static_cast<DisplayTreeItemKind>(
        item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
    if (kind != DisplayTreeItemKind::kDisplay) {
      continue;
    }
    ++display_items;
    const int index = item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex).toInt();
    if (index < 0 ||
        static_cast<std::size_t>(index) >= manager_->displays().size()) {
      continue;
    }
    const auto* display = manager_->displays()[static_cast<std::size_t>(index)];
    if (QTreeWidgetItem* status = findItemByKind(
            DisplayTreeItemKind::kDisplayStatus, index)) {
      const display::DisplayStatus display_status = display->status();
      status->setIcon(kDisplayTreeColValue,
                      IconLoader::statusIcon(display_status.level,
                                             display->enabled()));
      status->setText(kDisplayTreeColValue,
                      display_status.message.empty()
                          ? tr("Ok")
                          : QString::fromStdString(display_status.message));
    }
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
    if (frame.isEmpty()) {
      return;
    }
    manager_->setFixedFrame(frame.toStdString());
    emit fixedFrameChanged(frame);
    return;
  }

  if (kind == DisplayTreeItemKind::kGlobalBackgroundColor) {
    manager_->setBackgroundColor(value.toStdString());
    emit backgroundColorChanged(
        common::ParseColorProperty(value.toStdString(), QColor(48, 48, 48)));
    return;
  }

  if (kind == DisplayTreeItemKind::kGlobalFrameRate) {
    bool ok = false;
    const int rate = value.trimmed().toInt(&ok);
    if (!ok || rate < 1) {
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

  if (kind == DisplayTreeItemKind::kDisplayChannel) {
    manager_->setDisplayChannel(static_cast<std::size_t>(display_index),
                                value.toStdString(), child_index);
    emit displaysChanged();
    return;
  }

  if (kind == DisplayTreeItemKind::kDisplayProperty) {
    manager_->setDisplayProperty(static_cast<std::size_t>(display_index),
                                 property_key.toStdString(),
                                 value.toStdString(), child_index);
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

  if ((kind == DisplayTreeItemKind::kDisplay ||
       kind == DisplayTreeItemKind::kDisplayChild) &&
      column == kDisplayTreeColValue) {
    const int child_index =
        item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
    const bool enabled = item->checkState(kDisplayTreeColValue) == Qt::Checked;
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
