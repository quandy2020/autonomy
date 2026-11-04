/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/tree_select.hpp"

#include <QAction>
#include <QHBoxLayout>
#include <QHash>
#include <QMenu>
#include <QSignalBlocker>
#include <QToolButton>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

TreeSelect::TreeSelect(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizTreeSelect"));
  setProperty("autovizRole", QStringLiteral("treeSelect"));
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  button_ = new QToolButton(this);
  button_->setObjectName(QStringLiteral("AutovizTreeSelectButton"));
  button_->setPopupMode(QToolButton::InstantPopup);
  button_->setArrowType(Qt::DownArrow);
  button_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  menu_ = new QMenu(button_);
  button_->setMenu(menu_);
  layout->addWidget(button_);
  placeholder_text_ = tr("Select an item");
  setAccessibleName(tr("Tree selection"));
  button_->setAccessibleName(accessibleName());
  ApplyWidgetTheme(this, tone_, true);
  UpdatePresentation();
}

void TreeSelect::SetItems(const QVector<TreeSelectItem>& items) {
  items_ = items;
  if (!IsItemSelectable(FindItemById(current_id_))) {
    current_id_.clear();
  }
  RebuildMenu();
  UpdatePresentation();
}

QString TreeSelect::current_id() const { return current_id_; }

QVariant TreeSelect::current_data() const {
  const TreeSelectItem* item = FindItemById(current_id_);
  return item == nullptr ? QVariant() : item->data;
}

bool TreeSelect::SetCurrentId(const QString& id) {
  const TreeSelectItem* item = FindItemById(id);
  if (!IsItemSelectable(item)) {
    return false;
  }
  if (current_id_ == id) {
    return true;
  }
  current_id_ = id;
  UpdateCheckedActions();
  UpdatePresentation();
  emit CurrentItemChanged(current_id_, item->data);
  return true;
}

void TreeSelect::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyWidgetTheme(this, tone_, true);
}

void TreeSelect::SetPlaceholderText(const QString& text) {
  if (placeholder_text_ == text) {
    return;
  }
  placeholder_text_ = text;
  UpdatePresentation();
}

QSize TreeSelect::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QSize hint = button_->sizeHint();
  hint.setWidth(qMax(hint.width(), 180));
  hint.setHeight(qMax(hint.height(), tokens.control_height_medium));
  return hint;
}

QSize TreeSelect::minimumSizeHint() const {
  return QSize(120, GetThemeTokens(this, tone_).control_height_small);
}

void TreeSelect::RebuildMenu() {
  actions_.clear();
  menu_->clear();
  QSet<QString> ancestry;
  PopulateMenu(menu_, QString(), &ancestry);
  UpdateCheckedActions();
}

void TreeSelect::PopulateMenu(QMenu* menu, const QString& parent_id,
                              QSet<QString>* ancestry) {
  for (const TreeSelectItem& item : items_) {
    if (item.parent_id != parent_id || item.id.isEmpty() ||
        ancestry->contains(item.id)) {
      continue;
    }
    bool has_children = false;
    for (const TreeSelectItem& candidate : items_) {
      if (candidate.parent_id == item.id && candidate.id != item.id) {
        has_children = true;
        break;
      }
    }

    if (!has_children) {
      QAction* action = menu->addAction(item.text);
      action->setEnabled(item.enabled && item.selectable);
      action->setCheckable(true);
      action->setChecked(item.id == current_id_);
      connect(action, &QAction::triggered, this,
              [this, id = item.id]() { SetCurrentId(id); });
      actions_.insert(item.id, action);
      continue;
    }

    QMenu* submenu = menu->addMenu(item.text);
    submenu->setEnabled(item.enabled);
    if (item.selectable) {
      QAction* choose = submenu->addAction(tr("Select %1").arg(item.text));
      choose->setCheckable(true);
      choose->setChecked(item.id == current_id_);
      connect(choose, &QAction::triggered, this,
              [this, id = item.id]() { SetCurrentId(id); });
      actions_.insert(item.id, choose);
      submenu->addSeparator();
    }
    ancestry->insert(item.id);
    PopulateMenu(submenu, item.id, ancestry);
    ancestry->remove(item.id);
  }
}

const TreeSelectItem* TreeSelect::FindItemById(const QString& id) const {
  if (id.isEmpty()) {
    return nullptr;
  }
  for (const TreeSelectItem& item : items_) {
    if (item.id == id) {
      return &item;
    }
  }
  return nullptr;
}

bool TreeSelect::IsItemSelectable(const TreeSelectItem* item) const {
  if (item == nullptr || !item->enabled || !item->selectable) {
    return false;
  }
  QSet<QString> visited;
  QString parent_id = item->parent_id;
  while (!parent_id.isEmpty()) {
    if (visited.contains(parent_id)) {
      return false;
    }
    visited.insert(parent_id);
    const TreeSelectItem* parent = FindItemById(parent_id);
    if (parent == nullptr || !parent->enabled) {
      return false;
    }
    parent_id = parent->parent_id;
  }
  return true;
}

void TreeSelect::UpdateCheckedActions() {
  for (auto iterator = actions_.begin(); iterator != actions_.end();
       ++iterator) {
    QAction* action = iterator.value();
    if (action == nullptr) {
      continue;
    }
    const QSignalBlocker blocker(action);
    action->setChecked(iterator.key() == current_id_);
  }
}

void TreeSelect::UpdatePresentation() {
  const TreeSelectItem* item = FindItemById(current_id_);
  const QString text = item == nullptr ? placeholder_text_ : item->text;
  button_->setText(text);
  button_->setToolTip(item == nullptr ? QString() : item->text);
  const QString description = item == nullptr
                                  ? tr("No item selected")
                                  : tr("Selected item %1").arg(item->text);
  setAccessibleDescription(description);
  button_->setAccessibleDescription(description);
  updateGeometry();
}

}  // namespace autoviz::widget
