/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/add_panel_dialog.hpp"
#include "autoviz/ui/panel_catalog.hpp"

#include <QDialogButtonBox>
#include <QListWidget>
#include <QVBoxLayout>

#include "autoviz/ui/icon_loader.hpp"

namespace autoviz {

AddPanelDialog::AddPanelDialog(const QStringList& available_panels,
                               QWidget* parent)
    : QDialog(parent) {
  setWindowTitle(tr("Add New Panel"));
  resize(360, 320);

  auto* layout = new QVBoxLayout(this);
  list_ = new QListWidget(this);
  layout->addWidget(list_);

  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                           this);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(list_, &QListWidget::itemDoubleClicked, this, &QDialog::accept);

  populate(available_panels);
}

void AddPanelDialog::populate(const QStringList& available_panels) {
  list_->clear();
  for (const auto& entry : PanelCatalog()) {
    if (!entry.isImplemented()) {
      continue;
    }
    if (!available_panels.contains(QLatin1String(entry.object_name))) {
      continue;
    }
    const QString label = tr(entry.label);
    auto* item = new QListWidgetItem(
        IconLoader::panelIcon(QString::fromLatin1(entry.icon_id)), label);
    item->setData(Qt::UserRole, QString::fromLatin1(entry.object_name));
    item->setToolTip(tr(entry.description));
    list_->addItem(item);
  }
  if (list_->count() > 0) {
    list_->setCurrentRow(0);
  }
}

QString AddPanelDialog::selectedPanelObjectName() const {
  const QListWidgetItem* item = list_->currentItem();
  if (item == nullptr) {
    return {};
  }
  return item->data(Qt::UserRole).toString();
}

}  // namespace autoviz
