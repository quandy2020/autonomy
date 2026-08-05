/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QLabel;
class QVBoxLayout;

namespace autoviz {

/** Shared sidebar inspector for the active main-panel selection. */
class PropertyInspectorPanel : public QWidget {
  Q_OBJECT

 public:
  explicit PropertyInspectorPanel(QWidget* parent = nullptr);

  void setContentWidget(QWidget* widget, const QString& title);
  void clearContent();
  QWidget* contentWidget() const { return content_widget_; }

 private:
  void showPlaceholder();
  void clearContentLayout();

  QLabel* title_label_ = nullptr;
  QWidget* content_host_ = nullptr;
  QVBoxLayout* content_layout_ = nullptr;
  QWidget* placeholder_ = nullptr;
  QWidget* content_widget_ = nullptr;
};

}  // namespace autoviz
