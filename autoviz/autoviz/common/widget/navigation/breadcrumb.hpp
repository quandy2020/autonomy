/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QVariant>
#include <QVector>
#include <QWidget>

class QHBoxLayout;
class QKeyEvent;
class QToolButton;

namespace autoviz {
namespace widget {

struct BreadcrumbItem {
  QString text;
  QVariant data;
  bool enabled = true;
};

class Breadcrumb : public QWidget {
  Q_OBJECT

 public:
  explicit Breadcrumb(QWidget* parent = nullptr);

  const QVector<BreadcrumbItem>& items() const;
  void SetItems(QVector<BreadcrumbItem> items);
  void AppendItem(BreadcrumbItem item);
  void Clear();

  int current_index() const;
  void SetCurrentIndex(int index);

 signals:
  void ItemActivated(int index, const QVariant& data);
  void CurrentIndexChanged(int index);

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private:
  void Rebuild();
  void MoveCurrent(int step);
  void ActivateCurrent();
  bool IsSelectable(int index) const;

  QVector<BreadcrumbItem> items_;
  QVector<QToolButton*> buttons_;
  QHBoxLayout* layout_ = nullptr;
  int current_index_ = -1;
};

}  // namespace widget
}  // namespace autoviz
