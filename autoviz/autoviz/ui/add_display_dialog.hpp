/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include <QDialog>
#include <QString>

class QCheckBox;
class QDialogButtonBox;
class QGroupBox;
class QLineEdit;
class QTabWidget;
class QTextBrowser;
class QTreeWidget;

namespace autoviz {
namespace common {
class VisualizationManager;
}

struct AddDisplaySelection {
  QString type;
  QString display_name;
  QString channel;
  QString description_html;
};

class AddDisplayDialog : public QDialog {
  Q_OBJECT

 public:
  AddDisplayDialog(
      std::shared_ptr<common::VisualizationManager> manager,
      const QStringList& disallowed_display_names,
      QWidget* parent = nullptr);

  QSize sizeHint() const override;

  QString selectedType() const { return selection_.type; }
  QString selectedName() const { return selection_.display_name; }
  QString selectedChannel() const { return selection_.channel; }

 public slots:
  void accept() override;

 private slots:
  void onTabChanged(int index);
  void onNameChanged();

 private:
  void updateUi();
  bool isValid();
  void setError(const QString& error_text);
  void refreshTopicTree();

  std::shared_ptr<common::VisualizationManager> manager_;
  QStringList disallowed_display_names_;
  AddDisplaySelection selection_;
  AddDisplaySelection display_tab_selection_;
  AddDisplaySelection topic_tab_selection_;

  QTabWidget* tab_widget_ = nullptr;
  int display_tab_ = 0;
  int topic_tab_ = 0;
  QTreeWidget* topic_tree_ = nullptr;
  QLineEdit* topic_filter_box_ = nullptr;
  QCheckBox* show_unvisualizable_topics_ = nullptr;
  QTextBrowser* description_ = nullptr;
  QLineEdit* name_editor_ = nullptr;
  QDialogButtonBox* button_box_ = nullptr;
  QString last_selection_key_;
  bool user_edited_name_ = false;
};

}  // namespace autoviz
