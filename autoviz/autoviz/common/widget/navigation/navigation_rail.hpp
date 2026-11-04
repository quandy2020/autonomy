/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>
#include <QIcon>
#include <QString>
#include <QVariant>
#include <QVector>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QButtonGroup;
class QFocusEvent;
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

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 signals:
  void CurrentChanged(int index, const QVariant& data);
  void ItemActivated(int index, const QVariant& data);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  void ApplyTheme();
  void Rebuild();
  void MoveCurrent(int step);
  void ActivateCurrent();
  bool IsSelectable(int index) const;
  void UpdateAccessibility();

  QVector<NavigationItem> items_;
  QVector<QToolButton*> buttons_;
  QVBoxLayout* layout_ = nullptr;
  QButtonGroup* button_group_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  int current_index_ = -1;
  bool labels_visible_ = true;
};

}  // namespace widget
}  // namespace autoviz
