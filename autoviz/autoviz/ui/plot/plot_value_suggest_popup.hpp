/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>

#include <QObject>
#include <QPointer>
#include <QStringList>

class QLineEdit;
class QMenu;

namespace autoviz {
namespace plot {

/** Non-modal channel/field suggestion menu anchored to a plot Value line edit. */
class PlotValueSuggestList : public QObject {
 public:
  explicit PlotValueSuggestList(QLineEdit* anchor, QObject* parent = nullptr);

  void showSuggestions(const QStringList& suggestions);
  void hideSuggestions();
  bool isVisible() const;

  std::function<void(const QString&)> on_selected;

 private:
  void emitSelection(const QString& value);

  QPointer<QLineEdit> anchor_;
  QMenu* menu_ = nullptr;
  bool selecting_ = false;
};

}  // namespace plot
}  // namespace autoviz
