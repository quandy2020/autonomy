/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QWidget>

class QComboBox;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/view_tree_delegate.hpp"

namespace autoviz {
namespace common {
class VisualizationManager;
}
namespace rendering {
class ViewController;
}

enum class ViewTreeItemKind {
  kCurrentView = 0,
  kNearClip,
  kInvertZ,
  kTargetFrame,
  kDistance,
  kFocalShapeSize,
  kFocalShapeFixedSize,
  kYaw,
  kPitch,
  kFocalPointGroup,
  kFocalPointX,
  kFocalPointY,
  kFocalPointZ,
  kSavedView,
};

constexpr int kViewTreeColName = 0;
constexpr int kViewTreeColValue = 1;
constexpr int kViewTreeRoleKind = Qt::UserRole;
constexpr int kViewTreeRoleSavedIndex = Qt::UserRole + 1;

class ViewsPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ViewsPanel(rendering::ViewController* view_controller,
                      common::VisualizationManager* manager = nullptr,
                      QWidget* parent = nullptr);

  std::vector<common::SavedViewConfig> savedViews() const;
  void setSavedViews(const std::vector<common::SavedViewConfig>& views);
  void setManager(common::VisualizationManager* manager);
  void setViewController(rendering::ViewController* view_controller);
  void refreshFromController();
  /** Refresh TF frame list for the Target Frame combo without rebuilding the tree. */
  void refreshFrameList();
  void zeroView();

 signals:
  void viewsChanged();
  void viewChanged();

 private slots:
  void onTypeChanged(int index);
  void onZeroClicked();
  void onSaveClicked();
  void onRemoveClicked();
  void onRenameClicked();
  void onTreeItemChanged(QTreeWidgetItem* item, int column);
  void onTreeSelectionChanged();
  void onTreeItemActivated(QTreeWidgetItem* item, int column);

 private:
  void setupUi();
  void populateTypeSelector();
  void populateTree();
  void populateCurrentViewProperties(QTreeWidgetItem* parent);
  void updateCurrentViewValues();
  void updatePropertyVisibility();
  void updateActionButtons();
  void updateFrameDelegate();
  QString formattedTypeName(const QString& type) const;
  QTreeWidgetItem* findItemByKind(ViewTreeItemKind kind) const;

  common::VisualizationManager* manager_ = nullptr;
  rendering::ViewController* view_controller_ = nullptr;
  ViewTreeDelegate* value_delegate_ = nullptr;
  QComboBox* type_selector_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  QPushButton* remove_button_ = nullptr;
  QPushButton* rename_button_ = nullptr;
  std::vector<common::SavedViewConfig> saved_views_;
  bool updating_ = false;
};

}  // namespace autoviz
