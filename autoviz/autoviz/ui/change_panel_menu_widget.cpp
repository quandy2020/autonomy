/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/change_panel_menu_widget.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_catalog.hpp"

namespace autoviz {

ChangePanelMenuWidget::ChangePanelMenuWidget(QWidget* parent) : QWidget(parent) {
  setMinimumWidth(240);
  setMaximumWidth(320);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  search_ = new QLineEdit(this);
  search_->setPlaceholderText(tr("Search panels"));
  search_->setClearButtonEnabled(true);
  layout->addWidget(search_);

  list_ = new QListWidget(this);
  list_->setFrameShape(QFrame::NoFrame);
  list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list_->setMinimumHeight(220);
  layout->addWidget(list_, 1);

  connect(search_, &QLineEdit::textChanged, this,
          &ChangePanelMenuWidget::onFilterChanged);
  connect(list_, &QListWidget::itemActivated, this,
          &ChangePanelMenuWidget::onItemActivated);

  populate();
}

void ChangePanelMenuWidget::populate() {
  list_->clear();
  for (const PanelCatalogEntry& entry : PanelCatalog()) {
    if (!entry.isImplemented()) {
      continue;
    }
    const QString label = tr(entry.label);
    auto* item = new QListWidgetItem(
        IconLoader::panelIcon(QString::fromLatin1(entry.icon_id)), label);
    item->setData(Qt::UserRole, QString::fromLatin1(entry.object_name));
    item->setToolTip(tr(entry.description));
    list_->addItem(item);
  }
  onFilterChanged(search_->text());
}

void ChangePanelMenuWidget::onFilterChanged(const QString& text) {
  const QString needle = text.trimmed();
  for (int row = 0; row < list_->count(); ++row) {
    QListWidgetItem* item = list_->item(row);
    if (item == nullptr) {
      continue;
    }
    const bool match =
        needle.isEmpty() ||
        item->text().contains(needle, Qt::CaseInsensitive);
    item->setHidden(!match);
  }
}

void ChangePanelMenuWidget::onItemActivated() {
  const QListWidgetItem* item = list_->currentItem();
  if (item == nullptr || item->isHidden()) {
    return;
  }
  emit panelSelected(item->data(Qt::UserRole).toString());
}

}  // namespace autoviz
