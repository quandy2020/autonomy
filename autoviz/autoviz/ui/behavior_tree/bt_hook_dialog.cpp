/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_hook_dialog.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

namespace autoviz {
namespace behavior_tree {

BtHookDialog::BtHookDialog(const QString& node_name, const BtHook& current, QWidget* parent)
    : QDialog(parent), node_name_(node_name), node_uid_(current.node_uid) {
  setWindowTitle(tr("Hook Config"));
  setModal(true);
  setupUi(current);
  onModeChanged();
}

void BtHookDialog::setupUi(const BtHook& current) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(14, 14, 14, 14);
  root->setSpacing(10);

  auto* title = new QLabel(tr("Node: <b>%1</b>").arg(node_name_), this);
  root->addWidget(title);

  auto* mode_box = new QGroupBox(tr("Hook mode"), this);
  auto* mode_layout = new QVBoxLayout(mode_box);
  none_radio_ = new QRadioButton(tr("None"), mode_box);
  breakpoint_radio_ = new QRadioButton(tr("Breakpoint (Interactive)"), mode_box);
  replace_radio_ = new QRadioButton(tr("Replace Node"), mode_box);
  mode_group_ = new QButtonGroup(this);
  mode_group_->addButton(none_radio_, static_cast<int>(BtHookMode::kNone));
  mode_group_->addButton(breakpoint_radio_, static_cast<int>(BtHookMode::kBreakpoint));
  mode_group_->addButton(replace_radio_, static_cast<int>(BtHookMode::kReplace));
  mode_layout->addWidget(none_radio_);
  mode_layout->addWidget(breakpoint_radio_);
  mode_layout->addWidget(replace_radio_);
  root->addWidget(mode_box);

  auto* status_box = new QGroupBox(tr("Desired status"), this);
  status_frame_label_ = new QLabel(
      tr("Used when unlocking a breakpoint or replacing the node result."), status_box);
  status_frame_label_->setWordWrap(true);
  status_frame_label_->setStyleSheet(QStringLiteral("color: #64748b; font-size: 11px;"));
  auto* status_layout = new QVBoxLayout(status_box);
  status_layout->addWidget(status_frame_label_);
  success_radio_ = new QRadioButton(tr("Success"), status_box);
  failure_radio_ = new QRadioButton(tr("Failure"), status_box);
  running_radio_ = new QRadioButton(tr("Running"), status_box);
  status_group_ = new QButtonGroup(this);
  status_group_->addButton(success_radio_);
  status_group_->addButton(failure_radio_);
  status_group_->addButton(running_radio_);
  status_layout->addWidget(success_radio_);
  status_layout->addWidget(failure_radio_);
  status_layout->addWidget(running_radio_);
  root->addWidget(status_box);

  auto* position_box = new QGroupBox(tr("Position"), this);
  auto* position_layout = new QHBoxLayout(position_box);
  pre_radio_ = new QRadioButton(tr("PRE (before tick)"), position_box);
  post_radio_ = new QRadioButton(tr("POST (after tick)"), position_box);
  position_group_ = new QButtonGroup(this);
  position_group_->addButton(pre_radio_);
  position_group_->addButton(post_radio_);
  position_layout->addWidget(pre_radio_);
  position_layout->addWidget(post_radio_);
  root->addWidget(position_box);

  once_check_ = new QCheckBox(tr("Once (remove after hit)"), this);
  root->addWidget(once_check_);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  connect(button_box_, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);
  root->addWidget(button_box_);

  connect(mode_group_, &QButtonGroup::idClicked, this, [this](int) { onModeChanged(); });

  // Restore current values (Groot2 defaults: Breakpoint + Success + POST).
  switch (current.mode) {
    case BtHookMode::kReplace:
      replace_radio_->setChecked(true);
      break;
    case BtHookMode::kBreakpoint:
      breakpoint_radio_->setChecked(true);
      break;
    case BtHookMode::kNone:
    default:
      if (current.node_name.isEmpty() && current.node_uid < 0) {
        breakpoint_radio_->setChecked(true);
      } else {
        none_radio_->setChecked(true);
      }
      break;
  }

  switch (current.desired_status) {
    case BtNodeStatus::kFailure:
      failure_radio_->setChecked(true);
      break;
    case BtNodeStatus::kRunning:
      running_radio_->setChecked(true);
      break;
    case BtNodeStatus::kSuccess:
    default:
      success_radio_->setChecked(true);
      break;
  }

  if (current.position == BtHookPosition::kPre) {
    pre_radio_->setChecked(true);
  } else {
    post_radio_->setChecked(true);
  }
  once_check_->setChecked(current.once);
}

void BtHookDialog::onModeChanged() {
  const bool active = !none_radio_->isChecked();
  success_radio_->setEnabled(active);
  failure_radio_->setEnabled(active);
  running_radio_->setEnabled(active);
  pre_radio_->setEnabled(active);
  post_radio_->setEnabled(active);
  once_check_->setEnabled(active);
}

BtHook BtHookDialog::result() const {
  BtHook hook;
  hook.node_name = node_name_;
  hook.node_uid = node_uid_;
  hook.enabled = true;

  if (none_radio_->isChecked()) {
    hook.mode = BtHookMode::kNone;
    return hook;
  }
  if (replace_radio_->isChecked()) {
    hook.mode = BtHookMode::kReplace;
  } else {
    hook.mode = BtHookMode::kBreakpoint;
  }

  if (failure_radio_->isChecked()) {
    hook.desired_status = BtNodeStatus::kFailure;
  } else if (running_radio_->isChecked()) {
    hook.desired_status = BtNodeStatus::kRunning;
  } else {
    hook.desired_status = BtNodeStatus::kSuccess;
  }

  hook.position =
      pre_radio_->isChecked() ? BtHookPosition::kPre : BtHookPosition::kPost;
  hook.once = once_check_->isChecked();
  return hook;
}

BtBreakpointReachedDialog::BtBreakpointReachedDialog(const QString& node_name,
                                                     BtNodeStatus current_status,
                                                     BtNodeStatus default_result,
                                                     QWidget* parent)
    : QDialog(parent), chosen_status_(default_result) {
  setWindowTitle(tr("Breakpoint Reached"));
  setModal(true);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(14, 14, 14, 14);
  root->setSpacing(10);

  auto* info = new QLabel(
      tr("Interactive breakpoint hit on <b>%1</b><br/>Current status: <b>%2</b>")
          .arg(node_name, StatusToString(current_status)),
      this);
  info->setTextFormat(Qt::RichText);
  root->addWidget(info);

  auto* hint = new QLabel(tr("Choose the status used to continue execution:"), this);
  hint->setStyleSheet(QStringLiteral("color: #64748b;"));
  root->addWidget(hint);

  auto* remove_check = new QCheckBox(tr("Remove this hook after continue"), this);
  root->addWidget(remove_check);

  auto* buttons = new QHBoxLayout();
  auto* success_btn = new QPushButton(tr("Continue SUCCESS"), this);
  auto* failure_btn = new QPushButton(tr("Continue FAILURE"), this);
  auto* running_btn = new QPushButton(tr("Continue RUNNING"), this);
  auto* cancel_btn = new QPushButton(tr("Pause"), this);
  success_btn->setStyleSheet(QStringLiteral("QPushButton { color: #166534; font-weight: 600; }"));
  failure_btn->setStyleSheet(QStringLiteral("QPushButton { color: #991b1b; font-weight: 600; }"));
  running_btn->setStyleSheet(QStringLiteral("QPushButton { color: #9a3412; font-weight: 600; }"));
  buttons->addWidget(success_btn);
  buttons->addWidget(failure_btn);
  buttons->addWidget(running_btn);
  buttons->addWidget(cancel_btn);
  root->addLayout(buttons);

  const auto finish = [this, remove_check](BtNodeStatus status) {
    chosen_status_ = status;
    remove_hook_ = remove_check->isChecked();
    accept();
  };
  connect(success_btn, &QPushButton::clicked, this, [finish]() { finish(BtNodeStatus::kSuccess); });
  connect(failure_btn, &QPushButton::clicked, this, [finish]() { finish(BtNodeStatus::kFailure); });
  connect(running_btn, &QPushButton::clicked, this, [finish]() { finish(BtNodeStatus::kRunning); });
  connect(cancel_btn, &QPushButton::clicked, this, &QDialog::reject);

  switch (default_result) {
    case BtNodeStatus::kFailure:
      failure_btn->setDefault(true);
      break;
    case BtNodeStatus::kRunning:
      running_btn->setDefault(true);
      break;
    default:
      success_btn->setDefault(true);
      break;
  }
}

}  // namespace behavior_tree
}  // namespace autoviz
