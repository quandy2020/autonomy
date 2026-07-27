/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QListWidget;
class QPushButton;

namespace autoviz {
namespace common {
class TransformationManager;
}

class TransformationPanel : public QWidget {
  Q_OBJECT

 public:
  explicit TransformationPanel(common::TransformationManager* manager,
                               QWidget* parent = nullptr);

 public slots:
  void refresh();

 signals:
  void transformerChanged();
  void configChanged();

 private slots:
  void onItemChanged();
  void onSaveClicked();

 private:
  void setupUi();
  void populateList();
  void updateSaveButton();

  common::TransformationManager* manager_ = nullptr;
  QListWidget* list_ = nullptr;
  QPushButton* save_button_ = nullptr;
  std::string pending_transformer_id_;
  bool updating_ = false;
};

}  // namespace autoviz
