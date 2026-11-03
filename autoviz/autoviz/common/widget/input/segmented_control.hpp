/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QList>
#include <QVariant>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QButtonGroup;
class QHBoxLayout;

namespace autoviz {
namespace widget {

/**
 * @brief Exclusive, compact choice control with a model-like index API.
 */
class SegmentedControl final : public QWidget {
  Q_OBJECT
  Q_PROPERTY(int currentIndex READ current_index WRITE SetCurrentIndex NOTIFY
                 CurrentIndexChanged)

 public:
  explicit SegmentedControl(QWidget* parent = nullptr);
  ~SegmentedControl() override;

  int AddSegment(const QString& text, const QVariant& data = {});
  int InsertSegment(int index, const QString& text, const QVariant& data = {});
  void RemoveSegment(int index);
  void Clear();

  int count() const;
  int current_index() const;
  QVariant current_data() const;
  QString segment_text(int index) const;
  QVariant segment_data(int index) const;
  bool is_segment_enabled(int index) const;

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 public slots:
  void SetCurrentIndex(int index);
  void SetSegmentEnabled(int index, bool enabled);
  void SetSegmentText(int index, const QString& text);

 signals:
  void CurrentIndexChanged(int index);
  void CurrentDataChanged(const QVariant& data);
  void SegmentClicked(int index);

 private:
  class SegmentButton;

  void HandleButtonClicked(SegmentButton* button);
  void UpdateButtonPositions();

  QButtonGroup* button_group_ = nullptr;
  QHBoxLayout* layout_ = nullptr;
  QList<SegmentButton*> buttons_;
  QList<QVariant> data_;
  int current_index_ = -1;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace widget
}  // namespace autoviz
