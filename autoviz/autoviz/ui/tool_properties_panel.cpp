/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/tool.hpp"
#include "autoviz/ui/tool_properties_panel.hpp"

#include <QVBoxLayout>

namespace autoviz {

ToolPropertiesPanel::ToolPropertiesPanel(
    std::shared_ptr<common::VisualizationManager> manager, QWidget* parent)
    : QWidget(parent), manager_(std::move(manager)) {
  setupUi();
  refresh();
}

void ToolPropertiesPanel::setupUi() {
  auto* layout = new QVBoxLayout(this);
  tool_label_ = new QLabel(this);
  layout->addWidget(tool_label_);

  property_container_ = new QWidget(this);
  property_form_ = new QFormLayout(property_container_);
  layout->addWidget(property_container_);
  layout->addStretch();
}

void ToolPropertiesPanel::refresh() {
  updating_ = true;
  populateProperties();
  updating_ = false;
}

void ToolPropertiesPanel::populateProperties() {
  while (property_form_->rowCount() > 0) {
    property_form_->removeRow(0);
  }
  property_edits_.clear();

  const std::string tool_id = manager_->tools().activeToolId();
  tool_label_->setText(tr("Active Tool: %1")
                           .arg(manager_->tools().toolLabel(tool_id)));

  const auto specs = manager_->tools().activeToolPropertySpecs();
  if (specs.empty()) {
    property_form_->addRow(new QLabel(tr("No configurable properties"), this));
    return;
  }

  const common::Tool* tool = manager_->tools().toolById(tool_id);
  for (const auto& spec : specs) {
    auto* edit = new QLineEdit(this);
    edit->setProperty("property_key", QString::fromStdString(spec.key));
    if (tool != nullptr) {
      edit->setText(QString::fromStdString(
          tool->propertyValue(spec.key, spec.default_value)));
    } else {
      edit->setText(QString::fromStdString(spec.default_value));
    }
    connect(edit, &QLineEdit::editingFinished, this,
            &ToolPropertiesPanel::onPropertyEdited);
    property_form_->addRow(QString::fromStdString(spec.label), edit);
    property_edits_.push_back(edit);
  }
}

void ToolPropertiesPanel::onPropertyEdited() {
  if (updating_) {
    return;
  }
  auto* edit = qobject_cast<QLineEdit*>(sender());
  if (edit == nullptr) {
    return;
  }
  const QString key = edit->property("property_key").toString();
  if (key.isEmpty()) {
    return;
  }
  manager_->tools().setActiveToolProperty(key.toStdString(),
                                          edit->text().toStdString());
  emit propertiesChanged();
}

}  // namespace autoviz
