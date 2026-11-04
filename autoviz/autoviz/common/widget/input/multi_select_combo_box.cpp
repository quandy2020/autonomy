/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/multi_select_combo_box.hpp"

#include <QAction>
#include <QHBoxLayout>
#include <QMenu>
#include <QSet>
#include <QToolButton>
#include <algorithm>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

MultiSelectComboBox::MultiSelectComboBox(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizMultiSelectComboBox"));
  setProperty("autovizRole", QStringLiteral("multiSelectComboBox"));
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  button_ = new QToolButton(this);
  button_->setObjectName(QStringLiteral("AutovizMultiSelectButton"));
  button_->setPopupMode(QToolButton::InstantPopup);
  button_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  button_->setArrowType(Qt::DownArrow);
  button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  menu_ = new QMenu(button_);
  button_->setMenu(menu_);
  layout->addWidget(button_);

  placeholder_text_ = tr("Select options");
  setAccessibleName(tr("Multiple selection"));
  button_->setAccessibleName(accessibleName());
  ApplyWidgetTheme(this, tone_, true);
  UpdatePresentation();
}

void MultiSelectComboBox::SetItems(QVector<MultiSelectItem> items) {
  const bool selection_changed = !selected_indices().isEmpty();
  items_ = std::move(items);
  RebuildMenu();
  UpdatePresentation();
  if (selection_changed) {
    emit SelectionChanged({});
  }
}

QVector<int> MultiSelectComboBox::selected_indices() const {
  QVector<int> selected;
  selected.reserve(actions_.size());
  for (int index = 0; index < actions_.size(); ++index) {
    if (actions_[index]->isChecked()) {
      selected.push_back(index);
    }
  }
  return selected;
}

QVariantList MultiSelectComboBox::selected_data() const {
  QVariantList selected;
  for (const int index : selected_indices()) {
    selected.push_back(items_[index].data);
  }
  return selected;
}

void MultiSelectComboBox::SetSelectedIndices(const QVector<int>& indices) {
  QVector<int> requested;
  QSet<int> seen;
  for (const int index : indices) {
    if (index >= 0 && index < items_.size() && items_[index].enabled &&
        !seen.contains(index) &&
        (maximum_selection_ <= 0 || requested.size() < maximum_selection_)) {
      requested.push_back(index);
      seen.insert(index);
    }
  }
  std::sort(requested.begin(), requested.end());
  if (selected_indices() == requested) {
    return;
  }
  updating_actions_ = true;
  for (int index = 0; index < actions_.size(); ++index) {
    actions_[index]->setChecked(seen.contains(index));
  }
  updating_actions_ = false;
  UpdatePresentation();
  emit SelectionChanged(selected_indices());
}

void MultiSelectComboBox::SetMaximumSelectionCount(int count) {
  const int bounded = qMax(0, count);
  if (maximum_selection_ == bounded) {
    return;
  }
  maximum_selection_ = bounded;
  QVector<int> selected = selected_indices();
  if (maximum_selection_ > 0 && selected.size() > maximum_selection_) {
    selected.resize(maximum_selection_);
    SetSelectedIndices(selected);
  } else {
    UpdatePresentation();
  }
}

void MultiSelectComboBox::SetPlaceholderText(const QString& text) {
  if (placeholder_text_ == text) {
    return;
  }
  placeholder_text_ = text;
  UpdatePresentation();
}

void MultiSelectComboBox::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyWidgetTheme(this, tone_, true);
}

QSize MultiSelectComboBox::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QSize hint = button_->sizeHint();
  hint.setWidth(qMax(hint.width(), 180));
  hint.setHeight(qMax(hint.height(), tokens.control_height_medium));
  return hint;
}

QSize MultiSelectComboBox::minimumSizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  return QSize(120, tokens.control_height_small);
}

void MultiSelectComboBox::RebuildMenu() {
  menu_->clear();
  actions_.clear();
  actions_.reserve(items_.size());
  for (int index = 0; index < items_.size(); ++index) {
    auto* action = new QAction(items_[index].text, menu_);
    action->setCheckable(true);
    action->setEnabled(items_[index].enabled);
    action->setData(index);
    connect(action, &QAction::toggled, this, [this, index](bool checked) {
      HandleActionToggled(index, checked);
    });
    menu_->addAction(action);
    actions_.push_back(action);
  }
}

void MultiSelectComboBox::HandleActionToggled(int index, bool checked) {
  if (updating_actions_ || index < 0 || index >= actions_.size()) {
    return;
  }
  if (checked && maximum_selection_ > 0 &&
      selected_indices().size() > maximum_selection_) {
    updating_actions_ = true;
    actions_[index]->setChecked(false);
    updating_actions_ = false;
    return;
  }
  UpdatePresentation();
  emit SelectionChanged(selected_indices());
}

void MultiSelectComboBox::UpdatePresentation() {
  const QVector<int> selected = selected_indices();
  QStringList labels;
  labels.reserve(selected.size());
  for (const int index : selected) {
    labels.push_back(items_[index].text);
  }
  const QString text =
      labels.isEmpty() ? placeholder_text_ : labels.join(QStringLiteral(", "));
  button_->setText(text);
  button_->setToolTip(labels.join(QStringLiteral("\n")));
  const QString description =
      labels.isEmpty()
          ? tr("No options selected")
          : tr("%1 options selected: %2").arg(labels.size()).arg(text);
  setAccessibleDescription(description);
  button_->setAccessibleDescription(description);

  const bool limit_reached =
      maximum_selection_ > 0 && selected.size() >= maximum_selection_;
  for (int index = 0; index < actions_.size(); ++index) {
    actions_[index]->setEnabled(
        items_[index].enabled &&
        (!limit_reached || actions_[index]->isChecked()));
  }
  updateGeometry();
}

}  // namespace autoviz::widget
