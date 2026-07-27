/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/views_panel.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/common/view_state_io.hpp"
#include "autoviz/common/view_controller_registry.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace {

void SetTreeItemMeta(QTreeWidgetItem* item, ViewTreeItemKind kind,
                     int saved_index = -1) {
  item->setData(kViewTreeColName, kViewTreeRoleKind, static_cast<int>(kind));
  item->setData(kViewTreeColName, kViewTreeRoleSavedIndex, saved_index);
}

Qt::ItemFlags NameFlags() {
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

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
  setupUi();
  populateTree();
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
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* top_row = new QHBoxLayout();
  top_row->setContentsMargins(2, 6, 2, 2);
  top_row->addWidget(new QLabel(tr("Type:"), this));
  type_selector_ = new QComboBox(this);
  type_selector_->setObjectName(QStringLiteral("ViewsPanel/TypeSelector"));
  top_row->addWidget(type_selector_, 1);
  auto* zero_button = new QPushButton(tr("Zero"), this);
  zero_button->setToolTip(
      tr("Jump to 0,0,0 with the current view controller. Shortcut: Z"));
  top_row->addWidget(zero_button);
  layout->addLayout(top_row);

  tree_ = new QTreeWidget(this);
  tree_->setObjectName(QStringLiteral("ViewsPanel/PropertyTree"));
  tree_->setColumnCount(2);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setIndentation(12);
  tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->header()->setStretchLastSection(true);
  tree_->header()->setSectionResizeMode(kViewTreeColName,
                                        QHeaderView::ResizeToContents);
  tree_->setEditTriggers(QAbstractItemView::AllEditTriggers);
  value_delegate_ = new ViewTreeDelegate(tree_);
  tree_->setItemDelegateForColumn(kViewTreeColValue, value_delegate_);
  updateFrameDelegate();
  layout->addWidget(tree_, 1);

  auto* button_row = new QHBoxLayout();
  button_row->setContentsMargins(2, 0, 2, 2);
  auto* save_button = new QPushButton(tr("Save"), this);
  remove_button_ = new QPushButton(tr("Remove"), this);
  rename_button_ = new QPushButton(tr("Rename"), this);
  remove_button_->setEnabled(false);
  rename_button_->setEnabled(false);
  button_row->addWidget(save_button);
  button_row->addWidget(remove_button_);
  button_row->addWidget(rename_button_);
  layout->addLayout(button_row);

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
  connect(tree_, &QTreeWidget::itemActivated, this, &ViewsPanel::onTreeItemActivated);
}

QString ViewsPanel::formattedTypeName(const QString& type) const {
  return tr("%1 (autoviz)").arg(type);
}

void ViewsPanel::populateTypeSelector() {
  updating_ = true;
  type_selector_->clear();
  for (const std::string& type :
       common::ViewControllerRegistry::instance().typeNames()) {
    const QString qtype = QString::fromStdString(type);
    type_selector_->addItem(formattedTypeName(qtype), qtype);
  }
  if (view_controller_ != nullptr) {
    const int index = type_selector_->findData(view_controller_->typeName());
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
    item->setText(kViewTreeColValue, FormatFloat(state.yaw));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kPitch)) {
    item->setText(kViewTreeColValue, FormatFloat(state.pitch));
  }
  if (QTreeWidgetItem* group = findItemByKind(ViewTreeItemKind::kFocalPointGroup)) {
    group->setText(kViewTreeColValue,
                   QStringLiteral("%1; %2; %3")
                       .arg(FormatFloat(state.target.x()))
                       .arg(FormatFloat(state.target.y()))
                       .arg(FormatFloat(state.target.z())));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointX)) {
    item->setText(kViewTreeColValue, FormatFloat(state.target.x()));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointY)) {
    item->setText(kViewTreeColValue, FormatFloat(state.target.y()));
  }
  if (QTreeWidgetItem* item = findItemByKind(ViewTreeItemKind::kFocalPointZ)) {
    item->setText(kViewTreeColValue, FormatFloat(state.target.z()));
  }
  updatePropertyVisibility();
}

void ViewsPanel::updatePropertyVisibility() {
  if (view_controller_ == nullptr) {
    return;
  }
  const rendering::ViewControllerType type = view_controller_->type();
  const bool fps = type == rendering::ViewControllerType::kFps;
  const bool topdown_ortho = type == rendering::ViewControllerType::kTopDownOrtho;
  const bool fixed_pitch =
      type == rendering::ViewControllerType::kXyOrbit ||
      type == rendering::ViewControllerType::kTopDown ||
      topdown_ortho;

  const auto set_hidden = [this](ViewTreeItemKind kind, bool hidden) {
    if (QTreeWidgetItem* item = findItemByKind(kind)) {
      item->setHidden(hidden);
    }
  };

  set_hidden(ViewTreeItemKind::kDistance, fps);
  set_hidden(ViewTreeItemKind::kYaw, fps || topdown_ortho);
  set_hidden(ViewTreeItemKind::kPitch, fps || fixed_pitch);
  set_hidden(ViewTreeItemKind::kFocalShapeSize, fps);
  set_hidden(ViewTreeItemKind::kFocalShapeFixedSize, fps);
  set_hidden(ViewTreeItemKind::kFocalPointGroup, fps);
  set_hidden(ViewTreeItemKind::kFocalPointZ, topdown_ortho);
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
      state.yaw = item->text(kViewTreeColValue).toFloat();
      view_controller_->setState(state);
      changed = true;
      break;
    }
    case ViewTreeItemKind::kPitch: {
      rendering::ViewState state = view_controller_->state();
      state.pitch = item->text(kViewTreeColValue).toFloat();
      view_controller_->setState(state);
      changed = true;
      break;
    }
    case ViewTreeItemKind::kFocalPointX:
    case ViewTreeItemKind::kFocalPointY:
    case ViewTreeItemKind::kFocalPointZ: {
      rendering::ViewState state = view_controller_->state();
      QVector3D target = state.target;
      const float value = item->text(kViewTreeColValue).toFloat();
      if (kind == ViewTreeItemKind::kFocalPointX) {
        target.setX(value);
      } else if (kind == ViewTreeItemKind::kFocalPointY) {
        target.setY(value);
      } else {
        target.setZ(value);
      }
      view_controller_->setTarget(target);
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
