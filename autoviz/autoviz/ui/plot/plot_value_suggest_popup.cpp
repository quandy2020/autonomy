/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_value_suggest_popup.hpp"

#include <QAction>
#include <QLineEdit>
#include <QMenu>
#include <QPoint>

namespace autoviz {
namespace plot {

PlotValueSuggestList::PlotValueSuggestList(QLineEdit* anchor, QObject* parent)
    : QObject(parent), anchor_(anchor), menu_(new QMenu(anchor)) {
  menu_->setToolTipsVisible(false);
}

void PlotValueSuggestList::showSuggestions(const QStringList& suggestions) {
  if (selecting_ || anchor_ == nullptr || menu_ == nullptr) {
    return;
  }
  if (suggestions.isEmpty()) {
    hideSuggestions();
    return;
  }

  menu_->clear();
  for (const QString& suggestion : suggestions) {
    QAction* action = menu_->addAction(suggestion);
    connect(action, &QAction::triggered, this, [this, suggestion]() {
      emitSelection(suggestion);
    });
  }

  menu_->setMinimumWidth(qMax(anchor_->width(), 260));
  menu_->popup(anchor_->mapToGlobal(QPoint(0, anchor_->height() + 1)));
}

void PlotValueSuggestList::hideSuggestions() {
  if (menu_ != nullptr) {
    menu_->hide();
  }
}

bool PlotValueSuggestList::isVisible() const {
  return menu_ != nullptr && menu_->isVisible();
}

void PlotValueSuggestList::emitSelection(const QString& value) {
  if (selecting_) {
    return;
  }
  selecting_ = true;
  hideSuggestions();
  if (on_selected) {
    on_selected(value);
  }
  selecting_ = false;
}

}  // namespace plot
}  // namespace autoviz
