/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QVariant>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QHBoxLayout;
class QEvent;
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

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 signals:
  void ItemActivated(int index, const QVariant& data);
  void CurrentIndexChanged(int index);

 protected:
  void changeEvent(QEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  void ApplyTheme();
  void Rebuild();
  void MoveCurrent(int step);
  void ActivateCurrent();
  bool IsSelectable(int index) const;
  void UpdateAccessibility();

  QVector<BreadcrumbItem> items_;
  QVector<QToolButton*> buttons_;
  QHBoxLayout* layout_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  int current_index_ = -1;
};

}  // namespace widget
}  // namespace autoviz
