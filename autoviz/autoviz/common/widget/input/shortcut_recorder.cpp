/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/shortcut_recorder.hpp"

#include <QHBoxLayout>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QPalette>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

ShortcutRecorder::ShortcutRecorder(QWidget *parent)
    : QWidget(parent),
      editor_(new QKeySequenceEdit(this)),
      validation_label_(new QLabel(this)) {
  setObjectName(QStringLiteral("AutovizShortcutRecorder"));
  setProperty("autovizRole", QStringLiteral("shortcutRecorder"));
  setAccessibleName(tr("Keyboard shortcut"));

  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(GetThemeTokens(this, tone_).spacing_sm);
  editor_->setAccessibleName(tr("Keyboard shortcut"));
  layout->addWidget(editor_, 1);
  validation_label_->setAccessibleName(tr("Shortcut validation"));
  validation_label_->hide();
  layout->addWidget(validation_label_);

  connect(editor_, &QKeySequenceEdit::keySequenceChanged, this,
          [this](const QKeySequence &sequence) {
            UpdateValidation();
            emit KeySequenceChanged(sequence);
          });
  ApplyTheme();
}

QKeySequence ShortcutRecorder::key_sequence() const {
  return editor_->keySequence();
}

void ShortcutRecorder::SetKeySequence(const QKeySequence &sequence) {
  editor_->setKeySequence(sequence);
  UpdateValidation();
}

void ShortcutRecorder::SetReservedSequences(QList<QKeySequence> sequences) {
  reserved_sequences_ = std::move(sequences);
  UpdateValidation();
}

void ShortcutRecorder::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

void ShortcutRecorder::UpdateValidation() {
  const QKeySequence sequence = editor_->keySequence();
  bool conflict = false;
  if (!sequence.isEmpty()) {
    for (const QKeySequence &reserved : reserved_sequences_) {
      if (sequence.matches(reserved) == QKeySequence::ExactMatch) {
        conflict = true;
        break;
      }
    }
  }

  const bool was_valid = valid_;
  valid_ = !conflict;
  validation_label_->setText(valid_ ? QString() : tr("Shortcut already used"));
  validation_label_->setVisible(!valid_);
  editor_->setAccessibleDescription(valid_ ? tr("Shortcut is available")
                                           : tr("Shortcut is already used"));
  setProperty("autovizValidationError", !valid_);
  if (was_valid != valid_) {
    emit ValidityChanged(valid_);
  }
}

void ShortcutRecorder::ApplyTheme() {
  ApplyWidgetTheme(this, tone_, true);
  QPalette palette = validation_label_->palette();
  palette.setColor(QPalette::WindowText, GetThemeTokens(this, tone_).danger);
  validation_label_->setPalette(palette);
}

}  // namespace autoviz::widget
