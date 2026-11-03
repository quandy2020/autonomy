/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>
#include <QIcon>
#include <QString>
#include <QVariant>
#include <QVector>

class QButtonGroup;
class QKeyEvent;
class QToolButton;
class QVBoxLayout;

namespace autoviz {
namespace widget {

struct NavigationItem {
  QString text;
  QIcon icon;
  QVariant data;
  bool enabled = true;
};

class NavigationRail : public QFrame {
  Q_OBJECT

 public:
  explicit NavigationRail(QWidget* parent = nullptr);

  const QVector<NavigationItem>& items() const;
  void SetItems(QVector<NavigationItem> items);
  void AddItem(NavigationItem item);
  void Clear();

  int current_index() const;
  void SetCurrentIndex(int index);

  bool labels_visible() const;
  void SetLabelsVisible(bool visible);

 signals:
  void CurrentChanged(int index, const QVariant& data);
  void ItemActivated(int index, const QVariant& data);

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private:
  void Rebuild();
  void MoveCurrent(int step);
  void ActivateCurrent();
  bool IsSelectable(int index) const;

  QVector<NavigationItem> items_;
  QVector<QToolButton*> buttons_;
  QVBoxLayout* layout_ = nullptr;
  QButtonGroup* button_group_ = nullptr;
  int current_index_ = -1;
  bool labels_visible_ = true;
};

}  // namespace widget
}  // namespace autoviz
