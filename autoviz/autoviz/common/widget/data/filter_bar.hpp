/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QHBoxLayout;
class QHideEvent;
class QTimer;

namespace autoviz::widget {

class Button;
class SearchBox;

// A debounced search field plus an extensible row of filter controls. Added
// widgets are owned by the bar until removed with TakeFilterWidget().
class FilterBar final : public QFrame {
  Q_OBJECT
  Q_PROPERTY(QString query READ query WRITE SetQuery NOTIFY QueryChanged)

 public:
  explicit FilterBar(QWidget* parent = nullptr);

  QString query() const;
  void SetQuery(const QString& query);

  int debounce_interval_ms() const noexcept { return debounce_interval_ms_; }
  void SetDebounceIntervalMs(int interval_ms);

  void AddFilterWidget(QWidget* widget, int stretch = 0);
  QWidget* TakeFilterWidget(QWidget* widget);
  int filter_count() const noexcept;

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 public slots:
  void Clear();
  void Submit();

 signals:
  void QueryChanged(const QString& query);
  void SearchRequested(const QString& query);
  void Cleared();

 protected:
  void hideEvent(QHideEvent* event) override;

 private:
  void ScheduleQueryChanged();
  void UpdateClearButton();

  SearchBox* search_box_ = nullptr;
  Button* clear_button_ = nullptr;
  QHBoxLayout* filters_layout_ = nullptr;
  QTimer* debounce_timer_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  int debounce_interval_ms_ = 150;
};

}  // namespace autoviz::widget
