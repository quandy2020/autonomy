/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_hook.hpp"

#include <QDialog>

class QButtonGroup;
class QCheckBox;
class QDialogButtonBox;
class QLabel;
class QRadioButton;

namespace autoviz {
namespace behavior_tree {

/**
 * Groot2-style Hook Config dialog:
 * None / Breakpoint (Interactive) / Replace Node + Once.
 */
class BtHookDialog : public QDialog {
  Q_OBJECT

 public:
  BtHookDialog(const QString& node_name, const BtHook& current, QWidget* parent = nullptr);

  BtHook result() const;
  bool acceptedRemove() const { return result().mode == BtHookMode::kNone; }

 private slots:
  void onModeChanged();

 private:
  void setupUi(const BtHook& current);

  QString node_name_;
  int node_uid_ = -1;

  QRadioButton* none_radio_ = nullptr;
  QRadioButton* breakpoint_radio_ = nullptr;
  QRadioButton* replace_radio_ = nullptr;
  QButtonGroup* mode_group_ = nullptr;

  QRadioButton* success_radio_ = nullptr;
  QRadioButton* failure_radio_ = nullptr;
  QRadioButton* running_radio_ = nullptr;
  QButtonGroup* status_group_ = nullptr;

  QRadioButton* pre_radio_ = nullptr;
  QRadioButton* post_radio_ = nullptr;
  QButtonGroup* position_group_ = nullptr;

  QCheckBox* once_check_ = nullptr;
  QDialogButtonBox* button_box_ = nullptr;
  QLabel* status_frame_label_ = nullptr;
};

/** Shown when an interactive breakpoint is hit during Monitor/Replay. */
class BtBreakpointReachedDialog : public QDialog {
  Q_OBJECT

 public:
  BtBreakpointReachedDialog(const QString& node_name, BtNodeStatus current_status,
                            BtNodeStatus default_result, QWidget* parent = nullptr);

  BtNodeStatus chosenStatus() const { return chosen_status_; }
  bool removeHook() const { return remove_hook_; }

 private:
  BtNodeStatus chosen_status_ = BtNodeStatus::kSuccess;
  bool remove_hook_ = false;
};

}  // namespace behavior_tree
}  // namespace autoviz
