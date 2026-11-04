/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QKeySequence>
#include <QList>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QLabel;
class QKeySequenceEdit;

namespace autoviz::widget {

// Accessible key-binding editor with deterministic conflict validation.
class ShortcutRecorder final : public QWidget {
  Q_OBJECT

 public:
  explicit ShortcutRecorder(QWidget *parent = nullptr);

  QKeySequence key_sequence() const;
  void SetKeySequence(const QKeySequence &sequence);
  const QList<QKeySequence> &reserved_sequences() const noexcept {
    return reserved_sequences_;
  }
  void SetReservedSequences(QList<QKeySequence> sequences);
  bool is_valid() const noexcept { return valid_; }
  QKeySequenceEdit *editor() const noexcept { return editor_; }
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

 signals:
  void KeySequenceChanged(const QKeySequence &sequence);
  void ValidityChanged(bool valid);

 private:
  void UpdateValidation();
  void ApplyTheme();

  QKeySequenceEdit *editor_ = nullptr;
  QLabel *validation_label_ = nullptr;
  QList<QKeySequence> reserved_sequences_;
  style::Tone tone_ = style::Tone::Shell;
  bool valid_ = true;
};

}  // namespace autoviz::widget
