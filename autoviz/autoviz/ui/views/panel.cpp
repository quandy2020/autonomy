/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.cpp
 * @brief Implementation of @ref autoviz::ViewsPanel.
 *
 * Responsibilities covered here:
 * - frosted UI construction (@c setupUi);
 * - property-tree population and ViewController ↔ tree synchronization;
 * - saved-view CRUD and session-facing accessors;
 * - type-dependent property visibility (Orbit / FPS / TopDownOrtho).
 *
 * @see panel.hpp
 */

#include "autoviz/ui/views/panel.hpp"

#include <QAbstractItemView>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPaintEvent>
#include <QPalette>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/common/view_state_io.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

/**
 * @brief Attaches @ref ViewTreeItemKind (and optional saved-view index) to an
 *        item's name-column user roles.
 *
 * @param item Tree row to annotate (must not be @c nullptr).
 * @param kind Property / row discriminator for the delegate and change handler.
 * @param saved_index Index into @c saved_views_ for bookmark rows; −1 otherwise.
 */
void SetTreeItemMeta(QTreeWidgetItem* item, ViewTreeItemKind kind,
                     int saved_index = -1) {
  item->setData(kViewTreeColName, kViewTreeRoleKind, static_cast<int>(kind));
  item->setData(kViewTreeColName, kViewTreeRoleSavedIndex, saved_index);
}

/**
 * @brief Flags for the name column: selectable but not editable.
 */
Qt::ItemFlags NameFlags() {
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

/**
 * @brief Flags for the value column.
 *
 * @param editable When @c true, adds @c Qt::ItemIsEditable (line/combo edit).
 * @param checkable When @c true, adds @c Qt::ItemIsUserCheckable (checkbox).
 */
Qt::ItemFlags ValueFlags(bool editable, bool checkable = false) {
  Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  if (editable) {
    flags |= Qt::ItemIsEditable;
  }
  if (checkable) {
    flags |= Qt::ItemIsUserCheckable;
  }
  return flags;
}

/**
 * @brief Formats a float for the value column using general format (8 significant digits).
 *
 * @param value Numeric camera property.
 * @return Display / edit string written into the tree cell.
 */
QString FormatFloat(float value) {
  return QString::number(value, 'g', 8);
}

}  // namespace

ViewsPanel::ViewsPanel(rendering::ViewController* view_controller,
                       common::VisualizationManager* manager,
                       QWidget* parent)
    : QWidget(parent),
      manager_(manager),
      view_controller_(view_controller) {
  setAttribute(Qt::WA_StyledBackground, false);
  setAutoFillBackground(false);
  setupUi();
  populateTree();
}

void ViewsPanel::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  PaintPanelFrostedCard(painter, rect(), 14.0);
}

void ViewsPanel::setManager(common::VisualizationManager* manager) {
  manager_ = manager;
  updateFrameDelegate();
  if (view_controller_ != nullptr && manager_ != nullptr) {
    view_controller_->setFrameManager(&manager_->frameManager());
  }
}

void ViewsPanel::setViewController(rendering::ViewController* view_controller) {
  view_controller_ = view_controller;
  if (view_controller_ != nullptr && manager_ != nullptr) {
    view_controller_->setFrameManager(&manager_->frameManager());
  }
  populateTypeSelector();
  populateTree();
}

void ViewsPanel::setupUi() {
  setObjectName(QStringLiteral("Views/ViewsPanel"));
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(0);

  QHBoxLayout* top_row = nullptr;
  auto* toolbar = MakePanelToolbar(this, &top_row);
  auto* type_label = new QLabel(tr("Type"), toolbar);
  type_label->setStyleSheet(
      style::sheet(QStringLiteral("views"), style::tokens()));
  top_row->addWidget(type_label);
  type_selector_ = new QComboBox(toolbar);
  type_selector_->setObjectName(QStringLiteral("ViewsPanel/TypeSelector"));
  type_selector_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  StyleCompactSettingsField(type_selector_);
  top_row->addWidget(type_selector_, 1);
  auto* zero_button = new QPushButton(tr("Zero"), toolbar);
  zero_button->setObjectName(QStringLiteral("ViewsPanel/ZeroButton"));
  zero_button->setCursor(Qt::PointingHandCursor);
  zero_button->setToolTip(
      tr("Jump to 0,0,0 with the current view controller. Shortcut: Z"));
  zero_button->setStyleSheet(
      style::sheet(QStringLiteral("views")));
  top_row->addWidget(zero_button);
  layout->addWidget(toolbar);

  tree_ = new QTreeWidget(this);
  tree_->setObjectName(QStringLiteral("ViewsPanel/PropertyTree"));
  tree_->setColumnCount(2);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setIndentation(12);
  tree_->setIconSize(QSize(16, 16));
  tree_->setUniformRowHeights(true);
  tree_->setAnimated(true);
  tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->setAttribute(Qt::WA_TranslucentBackground, true);
  tree_->setAutoFillBackground(false);
  if (tree_->viewport() != nullptr) {
    tree_->viewport()->setAutoFillBackground(false);
    tree_->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
  }
  tree_->setAttribute(Qt::WA_MacShowFocusRect, false);
  tree_->setAllColumnsShowFocus(false);
  tree_->header()->setStretchLastSection(true);
  tree_->header()->setSectionResizeMode(kViewTreeColName,
                                        QHeaderView::ResizeToContents);
  StyleFrostedPanelTree(tree_, kViewTreeColName);
  tree_->setObjectName(QStringLiteral("ViewsPanel/PropertyTree"));
  QPalette tree_pal = tree_->palette();
  tree_pal.setBrush(QPalette::Highlight, Qt::transparent);
  tree_pal.setBrush(QPalette::HighlightedText, QColor(0x0F, 0x76, 0x6E));
  tree_pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  tree_pal.setBrush(QPalette::Inactive, QPalette::HighlightedText,
                    QColor(0x0F, 0x76, 0x6E));
  tree_->setPalette(tree_pal);
  value_delegate_ = new ViewTreeDelegate(tree_);
  tree_->setItemDelegateForColumn(kViewTreeColValue, value_delegate_);
  tree_->setEditTriggers(QAbstractItemView::DoubleClicked |
                         QAbstractItemView::SelectedClicked |
                         QAbstractItemView::EditKeyPressed);
  updateFrameDelegate();
  layout->addWidget(tree_, 1);

  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("ViewsFrostedFooter"));
  footer->setAttribute(Qt::WA_StyledBackground, true);
  footer->setStyleSheet(
      style::sheet(QStringLiteral("views")));
  auto* button_row = new QHBoxLayout(footer);
  button_row->setContentsMargins(10, 8, 10, 8);
  button_row->setSpacing(8);

  auto* save_button = new QPushButton(tr("Save"), footer);
  save_button->setObjectName(QStringLiteral("ViewsPanel/SaveButton"));
  save_button->setCursor(Qt::PointingHandCursor);
  save_button->setToolTip(tr("Save the current camera view"));
  save_button->setStyleSheet(PanelPrimaryButtonStyle());

  remove_button_ = new QPushButton(tr("Remove"), footer);
  remove_button_->setObjectName(QStringLiteral("ViewsPanel/RemoveButton"));
  remove_button_->setCursor(Qt::PointingHandCursor);
  remove_button_->setToolTip(tr("Remove the selected saved view"));
  remove_button_->setStyleSheet(PanelGhostButtonStyle());
  remove_button_->setEnabled(false);

  rename_button_ = new QPushButton(tr("Rename"), footer);
  rename_button_->setObjectName(QStringLiteral("ViewsPanel/RenameButton"));
  rename_button_->setCursor(Qt::PointingHandCursor);
  rename_button_->setToolTip(tr("Rename the selected saved view"));
  rename_button_->setStyleSheet(PanelGhostButtonStyle());
  rename_button_->setEnabled(false);

  button_row->addWidget(save_button);
  button_row->addWidget(remove_button_);
  button_row->addWidget(rename_button_);
  button_row->addStretch();
  layout->addWidget(footer);

  populateTypeSelector();

  connect(type_selector_, qOverload<int>(&QComboBox::activated), this,
          &ViewsPanel::onTypeChanged);
  connect(zero_button, &QPushButton::clicked, this, &ViewsPanel::onZeroClicked);
  connect(save_button, &QPushButton::clicked, this, &ViewsPanel::onSaveClicked);
  connect(remove_button_, &QPushButton::clicked, this, &ViewsPanel::onRemoveClicked);
  connect(rename_button_, &QPushButton::clicked, this, &ViewsPanel::onRenameClicked);
  connect(tree_, &QTreeWidget::itemChanged, this, &ViewsPanel::onTreeItemChanged);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &ViewsPanel::onTreeSelectionChanged);
  connect(tree_, &QTreeWidget::itemClicked, this, &ViewsPanel::onTreeItemActivated);
  connect(tree_, &QTreeWidget::itemActivated, this, &ViewsPanel::onTreeItemActivated);
}

QString ViewsPanel::formattedTypeName(const QString& type) const {
  return tr("%1 (autoviz)").arg(type);
}

void ViewsPanel::populateTypeSelector() {
  updating_ = true;
  type_selector_->clear();
  // Match RViz2 declared view controllers (no Autoviz-only TopDown / FPSMotion).
  static const char* kBuiltinViewTypes[] = {"Orbit", "XYOrbit", "TopDownOrtho", "FPS",
                                     "ThirdPersonFollow"};
  for (const char* type : kBuiltinViewTypes) {
    const QString qtype = QString::fromLatin1(type);
    type_selector_->addItem(formattedTypeName(qtype), qtype);
  }
  if (view_controller_ != nullptr) {
    QString current = view_controller_->typeName();
    if (current == QLatin1String("TopDown")) {
      current = QStringLiteral("TopDownOrtho");
    } else if (current == QLatin1String("FPSMotion")) {
      current = QStringLiteral("FPS");
    }
    const int index = type_selector_->findData(current);
    if (index >= 0) {
      type_selector_->setCurrentIndex(index);
    }
  }
  updating_ = false;
}

void ViewsPanel::populateCurrentViewProperties(QTreeWidgetItem* parent) {
  auto* near_clip = new QTreeWidgetItem(parent);
  near_clip->setText(kViewTreeColName, tr("Near Clip Distance"));
  near_clip->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(near_clip, ViewTreeItemKind::kNearClip);

  auto* invert_z = new QTreeWidgetItem(parent);
  invert_z->setText(kViewTreeColName, tr("Invert Z Axis"));
  invert_z->setFlags(NameFlags() | ValueFlags(false, true));
  SetTreeItemMeta(invert_z, ViewTreeItemKind::kInvertZ);

  auto* target_frame = new QTreeWidgetItem(parent);
  target_frame->setText(kViewTreeColName, tr("Target Frame"));
  target_frame->setText(kViewTreeColValue, rendering::ViewTargetFrameFixedSentinel());
  target_frame->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(target_frame, ViewTreeItemKind::kTargetFrame);

  auto* distance = new QTreeWidgetItem(parent);
  distance->setText(kViewTreeColName, tr("Distance"));
  distance->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(distance, ViewTreeItemKind::kDistance);

  auto* focal_size = new QTreeWidgetItem(parent);
  focal_size->setText(kViewTreeColName, tr("Focal Shape Size"));
  focal_size->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(focal_size, ViewTreeItemKind::kFocalShapeSize);

  auto* focal_fixed = new QTreeWidgetItem(parent);
  focal_fixed->setText(kViewTreeColName, tr("Focal Shape Fixed Size"));
  focal_fixed->setFlags(NameFlags() | ValueFlags(false, true));
  SetTreeItemMeta(focal_fixed, ViewTreeItemKind::kFocalShapeFixedSize);

  auto* yaw = new QTreeWidgetItem(parent);
  yaw->setText(kViewTreeColName, tr("Yaw"));
  yaw->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(yaw, ViewTreeItemKind::kYaw);

  auto* pitch = new QTreeWidgetItem(parent);
  pitch->setText(kViewTreeColName, tr("Pitch"));
  pitch->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(pitch, ViewTreeItemKind::kPitch);

  auto* focal_point = new QTreeWidgetItem(parent);
  focal_point->setText(kViewTreeColName, tr("Focal Point"));
  focal_point->setExpanded(true);
  focal_point->setFlags(NameFlags());
  SetTreeItemMeta(focal_point, ViewTreeItemKind::kFocalPointGroup);

  auto* focal_x = new QTreeWidgetItem(focal_point);
  focal_x->setText(kViewTreeColName, tr("X"));
  focal_x->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(focal_x, ViewTreeItemKind::kFocalPointX);

  auto* focal_y = new QTreeWidgetItem(focal_point);
  focal_y->setText(kViewTreeColName, tr("Y"));
  focal_y->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(focal_y, ViewTreeItemKind::kFocalPointY);

  auto* focal_z = new QTreeWidgetItem(focal_point);
  focal_z->setText(kViewTreeColName, tr("Z"));
  focal_z->setFlags(NameFlags() | ValueFlags(true));
  SetTreeItemMeta(focal_z, ViewTreeItemKind::kFocalPointZ);
}

void ViewsPanel::populateTree() {
  updating_ = true;
  tree_->clear();

  auto* current = new QTreeWidgetItem(tree_);
  current->setText(kViewTreeColName, tr("Current View"));
  current->setExpanded(true);
  current->setFlags(NameFlags());
  SetTreeItemMeta(current, ViewTreeItemKind::kCurrentView);
  populateCurrentViewProperties(current);

  for (std::size_t i = 0; i < saved_views_.size(); ++i) {
    auto* saved = new QTreeWidgetItem(tree_);
    saved->setText(kViewTreeColName,
                   QString::fromStdString(saved_views_[i].name));
    saved->setText(kViewTreeColValue,
                   formattedTypeName(QString::fromStdString(saved_views_[i].type)));
    saved->setFlags(NameFlags());
    SetTreeItemMeta(saved, ViewTreeItemKind::kSavedView, static_cast<int>(i));
  }

  updateCurrentViewValues();
  updatePropertyVisibility();
  updating_ = false;
}

void ViewsPanel::updateCurrentViewValues() {
  if (view_controller_ == nullptr) {
    return;
  }
  if (QTreeWidgetItem* current = findItemByKind(ViewTreeItemKind::kCurrentView)) {
    current->setText(kViewTreeColValue, formattedTypeName(view_controller_->typeName()));
  }
  const rendering::ViewState state = view_controller_->state();
  const bool fps = view_controller_->type() == rendering::ViewControllerType::kFps;
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kNearClip)) {
    item->setText(kViewTreeColValue, FormatFloat(state.near_clip_distance));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kInvertZ)) {
    item->setCheckState(kViewTreeColValue,
                        state.invert_z_axis ? Qt::Checked : Qt::Unchecked);
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kTargetFrame)) {
    item->setText(kViewTreeColValue, view_controller_->targetFrameDisplay());
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kDistance)) {
    item->setText(kViewTreeColValue, FormatFloat(state.distance));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalShapeSize)) {
    item->setText(kViewTreeColValue, FormatFloat(state.focal_shape_size));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalShapeFixedSize)) {
    item->setCheckState(
        kViewTreeColValue,
        state.focal_shape_fixed_size ? Qt::Checked : Qt::Unchecked);
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kYaw)) {
    item->setText(kViewTreeColValue,
                  FormatFloat(fps ? state.fps_yaw : state.yaw));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kPitch)) {
    item->setText(kViewTreeColValue,
                  FormatFloat(fps ? state.fps_pitch : state.pitch));
  }
  const QVector3D point = fps ? state.fps_position : state.target;
  if (QTreeWidgetItem* group = findItemByKind(ViewTreeItemKind::kFocalPointGroup)) {
    group->setText(kViewTreeColValue,
                   QStringLiteral("%1; %2; %3")
                       .arg(FormatFloat(point.x()))
                       .arg(FormatFloat(point.y()))
                       .arg(FormatFloat(point.z())));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointX)) {
    item->setText(kViewTreeColValue, FormatFloat(point.x()));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointY)) {
    item->setText(kViewTreeColValue, FormatFloat(point.y()));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointZ)) {
    item->setText(kViewTreeColValue, FormatFloat(point.z()));
  }
  updatePropertyVisibility();
}

void ViewsPanel::updatePropertyVisibility() {
  if (view_controller_ == nullptr) {
    return;
  }
  const rendering::ViewControllerType type = view_controller_->type();
  const bool fps = type == rendering::ViewControllerType::kFps ||
                   type == rendering::ViewControllerType::kFpsMotion;
  const bool ortho = type == rendering::ViewControllerType::kTopDownOrtho;

  const auto set_hidden = [this](ViewTreeItemKind kind, bool hidden) {
    if (QTreeWidgetItem* item = findItemByKind(kind)) {
      item->setHidden(hidden);
    }
  };
  const auto set_label = [this](ViewTreeItemKind kind, const QString& label) {
    if (QTreeWidgetItem* item = findItemByKind(kind)) {
      item->setText(kViewTreeColName, label);
    }
  };

  // Labels follow RViz2 Orbit / FPS / FixedOrientationOrtho property names.
  if (ortho) {
    set_label(ViewTreeItemKind::kDistance, tr("Scale"));
    set_label(ViewTreeItemKind::kYaw, tr("Angle"));
    set_label(ViewTreeItemKind::kFocalPointGroup, tr("Position"));
    set_label(ViewTreeItemKind::kFocalPointX, tr("X"));
    set_label(ViewTreeItemKind::kFocalPointY, tr("Y"));
  } else if (fps) {
    set_label(ViewTreeItemKind::kDistance, tr("Distance"));
    set_label(ViewTreeItemKind::kYaw, tr("Yaw"));
    set_label(ViewTreeItemKind::kPitch, tr("Pitch"));
    set_label(ViewTreeItemKind::kFocalPointGroup, tr("Position"));
    set_label(ViewTreeItemKind::kFocalPointX, tr("X"));
    set_label(ViewTreeItemKind::kFocalPointY, tr("Y"));
    set_label(ViewTreeItemKind::kFocalPointZ, tr("Z"));
  } else {
    set_label(ViewTreeItemKind::kDistance, tr("Distance"));
    set_label(ViewTreeItemKind::kYaw, tr("Yaw"));
    set_label(ViewTreeItemKind::kPitch, tr("Pitch"));
    set_label(ViewTreeItemKind::kFocalPointGroup, tr("Focal Point"));
    set_label(ViewTreeItemKind::kFocalPointX, tr("X"));
    set_label(ViewTreeItemKind::kFocalPointY, tr("Y"));
    set_label(ViewTreeItemKind::kFocalPointZ, tr("Z"));
  }

  set_hidden(ViewTreeItemKind::kDistance, fps);
  set_hidden(ViewTreeItemKind::kYaw, false);
  // Ortho uses Angle (yaw); Pitch is Orbit-family only (RViz Ortho has no Pitch).
  set_hidden(ViewTreeItemKind::kPitch, fps ? false : ortho);
  set_hidden(ViewTreeItemKind::kFocalShapeSize, fps || ortho);
  set_hidden(ViewTreeItemKind::kFocalShapeFixedSize, fps || ortho);
  set_hidden(ViewTreeItemKind::kFocalPointGroup, false);
  set_hidden(ViewTreeItemKind::kFocalPointZ, ortho);
}

void ViewsPanel::refreshFromController() {
  updating_ = true;
  populateTypeSelector();
  updateCurrentViewValues();
  updating_ = false;
}

void ViewsPanel::updateFrameDelegate() {
  if (value_delegate_ == nullptr || manager_ == nullptr) {
    return;
  }
  QStringList frames;
  frames.push_back(rendering::ViewTargetFrameFixedSentinel());
  for (const auto& name : manager_->frameManager().allFrameNames()) {
    frames.push_back(QString::fromStdString(name));
  }
  frames.sort(Qt::CaseInsensitive);
  frames.removeAll(rendering::ViewTargetFrameFixedSentinel());
  frames.prepend(rendering::ViewTargetFrameFixedSentinel());
  value_delegate_->setFrameNames(frames);
}

void ViewsPanel::refreshFrameList() { updateFrameDelegate(); }

QTreeWidgetItem* ViewsPanel::findItemByKind(ViewTreeItemKind kind) const {
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* top = tree_->topLevelItem(i);
    if (top == nullptr) {
      continue;
    }
    const auto item_kind = static_cast<ViewTreeItemKind>(
        top->data(kViewTreeColName, kViewTreeRoleKind).toInt());
    if (item_kind == kind) {
      return top;
    }
    for (int j = 0; j < top->childCount(); ++j) {
      QTreeWidgetItem* child = top->child(j);
      if (child == nullptr) {
        continue;
      }
      const auto child_kind = static_cast<ViewTreeItemKind>(
          child->data(kViewTreeColName, kViewTreeRoleKind).toInt());
      if (child_kind == kind) {
        return child;
      }
      for (int k = 0; k < child->childCount(); ++k) {
        QTreeWidgetItem* grandchild = child->child(k);
        if (grandchild == nullptr) {
          continue;
        }
        const auto gc_kind = static_cast<ViewTreeItemKind>(
            grandchild->data(kViewTreeColName, kViewTreeRoleKind).toInt());
        if (gc_kind == kind) {
          return grandchild;
        }
      }
    }
  }
  return nullptr;
}

std::vector<common::SavedViewConfig> ViewsPanel::savedViews() const {
  return saved_views_;
}

void ViewsPanel::setSavedViews(
    const std::vector<common::SavedViewConfig>& views) {
  saved_views_ = views;
  populateTree();
}

void ViewsPanel::onTypeChanged(int index) {
  if (updating_ || view_controller_ == nullptr) {
    return;
  }
  view_controller_->setTypeByName(type_selector_->itemData(index).toString());
  updateCurrentViewValues();
  emit viewChanged();
}

void ViewsPanel::zeroView() {
  onZeroClicked();
}

void ViewsPanel::onZeroClicked() {
  if (view_controller_ == nullptr) {
    return;
  }
  view_controller_->reset();
  populateTypeSelector();
  updateCurrentViewValues();
  emit viewChanged();
}

void ViewsPanel::onSaveClicked() {
  if (view_controller_ == nullptr) {
    return;
  }
  const QString name =
      tr("View %1").arg(static_cast<int>(saved_views_.size()) + 1);
  saved_views_.push_back(
      common::ToSavedViewConfig(name.toStdString(), view_controller_->state()));
  populateTree();
  emit viewsChanged();
}

void ViewsPanel::onRemoveClicked() {
  QTreeWidgetItem* item = tree_->currentItem();
  if (item == nullptr) {
    return;
  }
  const auto kind = static_cast<ViewTreeItemKind>(
      item->data(kViewTreeColName, kViewTreeRoleKind).toInt());
  if (kind != ViewTreeItemKind::kSavedView) {
    return;
  }
  const int index = item->data(kViewTreeColName, kViewTreeRoleSavedIndex).toInt();
  if (index < 0 || index >= static_cast<int>(saved_views_.size())) {
    return;
  }
  saved_views_.erase(saved_views_.begin() + index);
  populateTree();
  emit viewsChanged();
}

void ViewsPanel::onRenameClicked() {
  QTreeWidgetItem* item = tree_->currentItem();
  if (item == nullptr) {
    return;
  }
  const auto kind = static_cast<ViewTreeItemKind>(
      item->data(kViewTreeColName, kViewTreeRoleKind).toInt());
  if (kind != ViewTreeItemKind::kSavedView) {
    return;
  }
  const int index = item->data(kViewTreeColName, kViewTreeRoleSavedIndex).toInt();
  if (index < 0 || index >= static_cast<int>(saved_views_.size())) {
    return;
  }
  const QString current =
      QString::fromStdString(saved_views_[static_cast<std::size_t>(index)].name);
  bool ok = false;
  const QString next =
      QInputDialog::getText(this, tr("Rename View"), tr("New Name?"),
                            QLineEdit::Normal, current, &ok);
  if (!ok || next.trimmed().isEmpty() || next == current) {
    return;
  }
  saved_views_[static_cast<std::size_t>(index)].name = next.trimmed().toStdString();
  populateTree();
  emit viewsChanged();
}

void ViewsPanel::onTreeItemChanged(QTreeWidgetItem* item, int column) {
  if (updating_ || item == nullptr || view_controller_ == nullptr ||
      column != kViewTreeColValue) {
    return;
  }
  const auto kind = static_cast<ViewTreeItemKind>(
      item->data(kViewTreeColName, kViewTreeRoleKind).toInt());
  bool changed = false;
  switch (kind) {
    case ViewTreeItemKind::kNearClip:
      view_controller_->setNearClipDistance(item->text(kViewTreeColValue).toFloat());
      changed = true;
      break;
    case ViewTreeItemKind::kInvertZ:
      view_controller_->setInvertZAxis(item->checkState(kViewTreeColValue) == Qt::Checked);
      changed = true;
      break;
    case ViewTreeItemKind::kTargetFrame:
      view_controller_->setTargetFrame(item->text(kViewTreeColValue));
      changed = true;
      break;
    case ViewTreeItemKind::kDistance: {
      rendering::ViewState state = view_controller_->state();
      state.distance = item->text(kViewTreeColValue).toFloat();
      view_controller_->setState(state);
      changed = true;
      break;
    }
    case ViewTreeItemKind::kFocalShapeSize:
      view_controller_->setFocalShapeSize(item->text(kViewTreeColValue).toFloat());
      changed = true;
      break;
    case ViewTreeItemKind::kFocalShapeFixedSize:
      view_controller_->setFocalShapeFixedSize(
          item->checkState(kViewTreeColValue) == Qt::Checked);
      changed = true;
      break;
    case ViewTreeItemKind::kYaw: {
      rendering::ViewState state = view_controller_->state();
      const float value = item->text(kViewTreeColValue).toFloat();
      if (view_controller_->type() == rendering::ViewControllerType::kFps ||
          view_controller_->type() == rendering::ViewControllerType::kFpsMotion) {
        state.fps_yaw = value;
      } else {
        state.yaw = value;
      }
      view_controller_->setState(state);
      changed = true;
      break;
    }
    case ViewTreeItemKind::kPitch: {
      rendering::ViewState state = view_controller_->state();
      const float value = item->text(kViewTreeColValue).toFloat();
      if (view_controller_->type() == rendering::ViewControllerType::kFps ||
          view_controller_->type() == rendering::ViewControllerType::kFpsMotion) {
        state.fps_pitch = value;
      } else {
        state.pitch = value;
      }
      view_controller_->setState(state);
      changed = true;
      break;
    }
    case ViewTreeItemKind::kFocalPointX:
    case ViewTreeItemKind::kFocalPointY:
    case ViewTreeItemKind::kFocalPointZ: {
      rendering::ViewState state = view_controller_->state();
      const bool fps =
          view_controller_->type() == rendering::ViewControllerType::kFps ||
          view_controller_->type() == rendering::ViewControllerType::kFpsMotion;
      QVector3D point = fps ? state.fps_position : state.target;
      const float value = item->text(kViewTreeColValue).toFloat();
      if (kind == ViewTreeItemKind::kFocalPointX) {
        point.setX(value);
      } else if (kind == ViewTreeItemKind::kFocalPointY) {
        point.setY(value);
      } else {
        point.setZ(value);
      }
      if (fps) {
        state.fps_position = point;
        view_controller_->setState(state);
      } else {
        view_controller_->setTarget(point);
      }
      changed = true;
      break;
    }
    default:
      break;
  }
  if (changed) {
    updating_ = true;
    updateCurrentViewValues();
    updating_ = false;
    emit viewChanged();
  }
}

void ViewsPanel::onTreeSelectionChanged() { updateActionButtons(); }

void ViewsPanel::onTreeItemActivated(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column);
  if (item == nullptr || view_controller_ == nullptr) {
    return;
  }
  const auto kind = static_cast<ViewTreeItemKind>(
      item->data(kViewTreeColName, kViewTreeRoleKind).toInt());
  if (kind != ViewTreeItemKind::kSavedView) {
    return;
  }
  const int index = item->data(kViewTreeColName, kViewTreeRoleSavedIndex).toInt();
  if (index < 0 || index >= static_cast<int>(saved_views_.size())) {
    return;
  }
  view_controller_->setState(
      common::ToViewState(saved_views_[static_cast<std::size_t>(index)]));
  refreshFromController();
  emit viewChanged();
}

void ViewsPanel::updateActionButtons() {
  QTreeWidgetItem* item = tree_->currentItem();
  const bool saved_selected =
      item != nullptr &&
      static_cast<ViewTreeItemKind>(
          item->data(kViewTreeColName, kViewTreeRoleKind).toInt()) ==
          ViewTreeItemKind::kSavedView;
  remove_button_->setEnabled(saved_selected);
  rename_button_->setEnabled(saved_selected);
}

}  // namespace autoviz
