/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_node_editor_dialog.hpp"

#include "autoviz/ui/behavior_tree/bt_icon_loader.hpp"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextEdit>
#include <QVBoxLayout>

namespace autoviz {
namespace behavior_tree {
namespace {

QWidget* MakeScriptForm(QWidget* parent, const QStringList& labels,
                        const QVector<QLineEdit*>& edits) {
  auto* page = new QWidget(parent);
  auto* form = new QFormLayout(page);
  form->setContentsMargins(12, 12, 12, 12);
  form->setHorizontalSpacing(12);
  form->setVerticalSpacing(10);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
  for (int i = 0; i < labels.size() && i < edits.size(); ++i) {
    form->addRow(labels.at(i), edits.at(i));
  }
  return page;
}

}  // namespace

QLineEdit* BtNodeEditorDialog::MakeScriptEdit(QWidget* parent, const QString& value,
                                              bool read_only) {
  auto* edit = new QLineEdit(parent);
  edit->setText(value);
  edit->setReadOnly(read_only);
  edit->setMinimumHeight(28);
  edit->setStyleSheet(QStringLiteral(
      "QLineEdit {"
      "  background: #ffffff; color: #1e293b;"
      "  border: 1px solid #334155; border-radius: 2px;"
      "  padding: 4px 8px;"
      "}"
      "QLineEdit:focus { border-color: #0891b2; }"
      "QLineEdit:read-only { background: #f1f5f9; }"));
  return edit;
}

BtNodeEditorDialog::BtNodeEditorDialog(const BtAbsNode& node, bool read_only,
                                       QWidget* parent)
    : QDialog(parent) {
  setWindowTitle(tr("Node Editor"));
  setModal(true);
  resize(520, 360);
  setStyleSheet(QStringLiteral("QDialog { background: #e8eaed; }"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(14, 14, 14, 12);
  root->setSpacing(10);

  auto* header = new QFormLayout();
  header->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  header->setHorizontalSpacing(10);
  header->setVerticalSpacing(8);

  auto* type_value = new QLabel(KindDisplayName(node.kind), this);
  type_value->setStyleSheet(QStringLiteral("font-weight: 600; color: #0f172a;"));
  header->addRow(tr("NodeType:"), type_value);

  auto* model_value = new QLabel(node.registration_id, this);
  model_value->setStyleSheet(QStringLiteral("font-weight: 600; color: #0f172a;"));
  header->addRow(tr("Model Name:"), model_value);

  instance_edit_ = MakeScriptEdit(this, node.instance_name, read_only);
  if (instance_edit_->text().isEmpty()) {
    instance_edit_->setText(node.registration_id);
  }
  header->addRow(tr("Instance name:"), instance_edit_);
  root->addLayout(header);

  auto* tabs = new QTabWidget(this);
  tabs->setDocumentMode(false);
  tabs->setStyleSheet(QStringLiteral(
      "QTabWidget::pane {"
      "  background: #ffffff; border: 1px solid #cbd5e1; border-radius: 0px;"
      "  top: -1px;"
      "}"
      "QTabBar::tab {"
      "  background: #d8dce2; color: #0f172a;"
      "  padding: 8px 14px; margin-right: 2px;"
      "  border-top-left-radius: 6px; border-top-right-radius: 6px;"
      "}"
      "QTabBar::tab:selected { background: #ffffff; font-weight: 600; }"));

  skip_if_edit_ = MakeScriptEdit(tabs, node.skip_if, read_only);
  success_if_edit_ = MakeScriptEdit(tabs, node.success_if, read_only);
  failure_if_edit_ = MakeScriptEdit(tabs, node.failure_if, read_only);
  while_edit_ = MakeScriptEdit(tabs, node.while_script, read_only);
  tabs->addTab(MakeScriptForm(tabs,
                              {QStringLiteral("_skipIf"), QStringLiteral("_successIf"),
                               QStringLiteral("_failureIf"), QStringLiteral("_while")},
                              {skip_if_edit_, success_if_edit_, failure_if_edit_, while_edit_}),
               tr("Pre Conditions"));

  on_success_edit_ = MakeScriptEdit(tabs, node.on_success, read_only);
  on_failure_edit_ = MakeScriptEdit(tabs, node.on_failure, read_only);
  on_halted_edit_ = MakeScriptEdit(tabs, node.on_halted, read_only);
  post_edit_ = MakeScriptEdit(tabs, node.post_script, read_only);
  tabs->addTab(MakeScriptForm(tabs,
                              {QStringLiteral("_onSuccess"), QStringLiteral("_onFailure"),
                               QStringLiteral("_onHalted"), QStringLiteral("_post")},
                              {on_success_edit_, on_failure_edit_, on_halted_edit_, post_edit_}),
               tr("Post Conditions"));

  auto* desc_page = new QWidget(tabs);
  auto* desc_layout = new QVBoxLayout(desc_page);
  desc_layout->setContentsMargins(12, 12, 12, 12);
  desc_layout->setSpacing(8);
  auto* desc_label = new QLabel(tr("Add a Description/Comments to this node:"), desc_page);
  desc_label->setStyleSheet(QStringLiteral("color: #0f172a;"));
  desc_layout->addWidget(desc_label);
  description_edit_ = new QTextEdit(desc_page);
  description_edit_->setPlainText(node.description);
  description_edit_->setReadOnly(read_only);
  description_edit_->setMinimumHeight(140);
  description_edit_->setStyleSheet(QStringLiteral(
      "QTextEdit {"
      "  background: #ffffff; color: #1e293b;"
      "  border: 1px solid #334155; border-radius: 2px;"
      "  padding: 6px;"
      "}"
      "QTextEdit:read-only { background: #f1f5f9; }"));
  desc_layout->addWidget(description_edit_, 1);
  tabs->addTab(desc_page, tr("Description"));

  root->addWidget(tabs, 1);

  auto* buttons = new QDialogButtonBox(this);
  auto* cancel_btn = buttons->addButton(QDialogButtonBox::Cancel);
  auto* ok_btn = buttons->addButton(QDialogButtonBox::Ok);
  cancel_btn->setText(tr("Cancel"));
  ok_btn->setText(tr("OK"));
  cancel_btn->setIcon(BtIconLoader::toolbarIcon(QStringLiteral("close_x.png"), 16));
  ok_btn->setIcon(BtIconLoader::toolbarIcon(QStringLiteral("svg/arrow_right.svg"), 16));
  ok_btn->setStyleSheet(QStringLiteral(
      "QPushButton {"
      "  background: #dcfce7; border: 1px solid #86efac;"
      "  border-radius: 4px; padding: 6px 14px; font-weight: 600;"
      "}"
      "QPushButton:hover { background: #bbf7d0; }"));
  if (read_only) {
    ok_btn->setEnabled(false);
  }
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  root->addWidget(buttons, 0, Qt::AlignRight);
}

BtNodeEditorResult BtNodeEditorDialog::result() const {
  BtNodeEditorResult out;
  out.instance_name = instance_edit_ != nullptr ? instance_edit_->text().trimmed() : QString();
  out.skip_if = skip_if_edit_ != nullptr ? skip_if_edit_->text().trimmed() : QString();
  out.success_if = success_if_edit_ != nullptr ? success_if_edit_->text().trimmed() : QString();
  out.failure_if = failure_if_edit_ != nullptr ? failure_if_edit_->text().trimmed() : QString();
  out.while_script = while_edit_ != nullptr ? while_edit_->text().trimmed() : QString();
  out.on_success = on_success_edit_ != nullptr ? on_success_edit_->text().trimmed() : QString();
  out.on_failure = on_failure_edit_ != nullptr ? on_failure_edit_->text().trimmed() : QString();
  out.on_halted = on_halted_edit_ != nullptr ? on_halted_edit_->text().trimmed() : QString();
  out.post_script = post_edit_ != nullptr ? post_edit_->text().trimmed() : QString();
  out.description =
      description_edit_ != nullptr ? description_edit_->toPlainText().trimmed() : QString();
  return out;
}

}  // namespace behavior_tree
}  // namespace autoviz
