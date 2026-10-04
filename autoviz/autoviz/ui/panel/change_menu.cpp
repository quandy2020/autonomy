/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/change_menu.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QVBoxLayout>

#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

}  // namespace

ChangePanelMenuWidget::ChangePanelMenuWidget(QWidget* parent) : QWidget(parent) {
  setMinimumWidth(240);
  setMaximumWidth(320);
  setAttribute(Qt::WA_StyledBackground, true);
  setAutoFillBackground(false);
  setStyleSheet(style::sheet(QStringLiteral("change_panel")));

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  search_ = new QLineEdit(this);
  search_->setPlaceholderText(tr("Search panels"));
  search_->setClearButtonEnabled(true);
  layout->addWidget(search_);

  list_ = new QListWidget(this);
  // Match Panels menu / Displays tree (16px), not the bulky Add Panel tiles.
  list_->setIconSize(QSize(16, 16));
  list_->setFrameShape(QFrame::NoFrame);
  list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list_->setMinimumHeight(220);
  layout->addWidget(list_, 1);

  connect(search_, &QLineEdit::textChanged, this,
          &ChangePanelMenuWidget::onFilterChanged);
  // Single click selects (macOS itemActivated usually needs a double-click).
  connect(list_, &QListWidget::itemClicked, this,
          &ChangePanelMenuWidget::onItemActivated);
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
    // Teleop is sidebar-only — add via Panels / Add Panel, not Change Panel.
    if (QLatin1String(entry.object_name) == QLatin1String("TeleopDock")) {
      continue;
    }
    const QString label = tr(entry.label);
    const QString object_name = QString::fromLatin1(entry.object_name);
    auto* item = new QListWidgetItem(
        IconLoader::panelsMenuDockIcon(object_name), label);
    item->setData(Qt::UserRole, object_name);
    // No per-item tooltip — keep the picker quiet while browsing.
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
  if (selecting_) {
    return;
  }
  const QListWidgetItem* item = list_->currentItem();
  if (item == nullptr || item->isHidden()) {
    return;
  }
  selecting_ = true;
  emit panelSelected(item->data(Qt::UserRole).toString());
  selecting_ = false;
}

}  // namespace autoviz
