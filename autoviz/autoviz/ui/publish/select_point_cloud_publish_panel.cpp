/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/publish/select_point_cloud_publish_panel.hpp"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

SelectPointCloudPublishPanel::SelectPointCloudPublishPanel(
    common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  (void)manager_;
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/selected_points"), this);
  stub_label_ = new QLabel(
      tr("Point-cloud selection and publish are not implemented yet."), this);
  stub_label_->setWordWrap(true);

  auto* form = new QFormLayout();
  form->addRow(tr("Channel:"), channel_edit_);

  auto* layout = new QVBoxLayout(this);
  layout->addWidget(stub_label_);
  layout->addLayout(form);
}

}  // namespace autoviz
