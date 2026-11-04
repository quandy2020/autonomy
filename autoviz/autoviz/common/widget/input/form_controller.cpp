/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/form_controller.hpp"

#include <QAbstractButton>
#include <QComboBox>
#include <QDateTimeEdit>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QLineEdit>
#include <QMetaProperty>
#include <QSpinBox>
#include <QWidget>

namespace autoviz::widget {

FormController::FormController(QObject *parent) : QObject(parent) {}

void FormController::RegisterField(QWidget *field, bool required) {
  if (field == nullptr || fields_.contains(field)) {
    return;
  }

  FieldState state;
  state.initial_value = ReadValue(field);
  state.required = required;
  state.base_accessible_description = field->accessibleDescription();
  fields_.insert(field, state);

  auto changed = [this, field]() { NotifyFieldChanged(field); };
  QMetaObject::Connection value_connection;
  if (auto *edit = qobject_cast<QLineEdit *>(field); edit != nullptr) {
    value_connection = connect(edit, &QLineEdit::textChanged, this,
                               [changed](const QString &) { changed(); });
  } else if (auto *combo = qobject_cast<QComboBox *>(field); combo != nullptr) {
    value_connection = connect(combo, &QComboBox::currentIndexChanged, this,
                               [changed](int) { changed(); });
  } else if (auto *button = qobject_cast<QAbstractButton *>(field);
             button != nullptr) {
    value_connection = connect(button, &QAbstractButton::toggled, this,
                               [changed](bool) { changed(); });
  } else if (auto *spin = qobject_cast<QSpinBox *>(field); spin != nullptr) {
    value_connection = connect(spin, &QSpinBox::valueChanged, this,
                               [changed](int) { changed(); });
  } else if (auto *spin = qobject_cast<QDoubleSpinBox *>(field);
             spin != nullptr) {
    value_connection = connect(spin, &QDoubleSpinBox::valueChanged, this,
                               [changed](double) { changed(); });
  } else if (auto *date_time = qobject_cast<QDateTimeEdit *>(field);
             date_time != nullptr) {
    value_connection = connect(date_time, &QDateTimeEdit::dateTimeChanged, this,
                               [changed](const QDateTime &) { changed(); });
  }

  FieldState &stored = fields_[field];
  stored.value_connection = value_connection;
  stored.destroyed_connection =
      connect(field, &QObject::destroyed, this, [this, field]() {
        fields_.remove(field);
        RecomputeState();
      });
  field->installEventFilter(this);
  UpdateField(field);
  RecomputeState();
}

void FormController::UnregisterField(QWidget *field) {
  auto iterator = fields_.find(field);
  if (iterator == fields_.end()) {
    return;
  }
  disconnect(iterator->value_connection);
  disconnect(iterator->destroyed_connection);
  field->removeEventFilter(this);
  field->setProperty("autovizValidationError", false);
  field->setAccessibleDescription(iterator->base_accessible_description);
  fields_.erase(iterator);
  RecomputeState();
}

void FormController::SetRequired(QWidget *field, bool required) {
  auto iterator = fields_.find(field);
  if (iterator == fields_.end() || iterator->required == required) {
    return;
  }
  iterator->required = required;
  UpdateField(field);
  RecomputeState();
}

void FormController::SetFieldError(QWidget *field, const QString &error) {
  auto iterator = fields_.find(field);
  if (iterator == fields_.end() || iterator->manual_error == error) {
    return;
  }
  iterator->manual_error = error;
  UpdateField(field);
  RecomputeState();
}

void FormController::ClearFieldError(QWidget *field) {
  SetFieldError(field, {});
}

QString FormController::field_error(QWidget *field) const {
  const auto iterator = fields_.constFind(field);
  return iterator == fields_.constEnd() ? QString() : iterator->error;
}

void FormController::MarkClean() {
  for (auto iterator = fields_.begin(); iterator != fields_.end(); ++iterator) {
    iterator->initial_value = ReadValue(iterator.key());
  }
  RecomputeState();
}

void FormController::NotifyFieldChanged(QWidget *field) {
  if (!fields_.contains(field)) {
    return;
  }
  UpdateField(field);
  RecomputeState();
}

void FormController::SetSubmitButton(QAbstractButton *button) {
  submit_button_ = button;
  if (submit_button_ != nullptr) {
    submit_button_->setEnabled(valid_);
  }
}

bool FormController::eventFilter(QObject *watched, QEvent *event) {
  auto *field = qobject_cast<QWidget *>(watched);
  if (field != nullptr && fields_.contains(field) &&
      event->type() == QEvent::FocusOut) {
    NotifyFieldChanged(field);
  }
  return QObject::eventFilter(watched, event);
}

QVariant FormController::ReadValue(QWidget *field) const {
  if (const auto *edit = qobject_cast<QLineEdit *>(field); edit != nullptr) {
    return edit->text();
  }
  if (const auto *combo = qobject_cast<QComboBox *>(field); combo != nullptr) {
    return combo->currentIndex();
  }
  if (const auto *button = qobject_cast<QAbstractButton *>(field);
      button != nullptr) {
    return button->isChecked();
  }
  if (const auto *spin = qobject_cast<QSpinBox *>(field); spin != nullptr) {
    return spin->value();
  }
  if (const auto *spin = qobject_cast<QDoubleSpinBox *>(field);
      spin != nullptr) {
    return spin->value();
  }
  if (const auto *date_time = qobject_cast<QDateTimeEdit *>(field);
      date_time != nullptr) {
    return date_time->dateTime();
  }
  const QMetaProperty property = field->metaObject()->userProperty();
  return property.isValid() ? property.read(field) : QVariant();
}

QString FormController::ValidateField(QWidget *field,
                                      const FieldState &state) const {
  if (!state.manual_error.isEmpty()) {
    return state.manual_error;
  }
  if (const auto *edit = qobject_cast<QLineEdit *>(field); edit != nullptr) {
    if (state.required && edit->text().trimmed().isEmpty()) {
      return tr("This field is required");
    }
    if (!edit->hasAcceptableInput()) {
      return tr("The value is invalid");
    }
    return {};
  }
  if (const auto *combo = qobject_cast<QComboBox *>(field);
      combo != nullptr && state.required && combo->currentIndex() < 0) {
    return tr("This field is required");
  }
  if (const auto *button = qobject_cast<QAbstractButton *>(field);
      button != nullptr && state.required && button->isCheckable() &&
      !button->isChecked()) {
    return tr("This field is required");
  }
  const QVariant value = ReadValue(field);
  if (state.required &&
      (!value.isValid() || value.isNull() ||
       (value.canConvert<QString>() && value.toString().trimmed().isEmpty()))) {
    return tr("This field is required");
  }
  return {};
}

void FormController::UpdateField(QWidget *field) {
  auto iterator = fields_.find(field);
  if (iterator == fields_.end()) {
    return;
  }
  const QString previous_error = iterator->error;
  iterator->error = ValidateField(field, *iterator);
  field->setProperty("autovizValidationError", !iterator->error.isEmpty());
  field->setAccessibleDescription(iterator->error.isEmpty()
                                      ? iterator->base_accessible_description
                                      : iterator->error);
  if (previous_error != iterator->error) {
    emit FieldErrorChanged(field, iterator->error);
  }
}

void FormController::RecomputeState() {
  bool valid = true;
  bool dirty = false;
  for (auto iterator = fields_.constBegin(); iterator != fields_.constEnd();
       ++iterator) {
    valid = valid && iterator->error.isEmpty();
    dirty = dirty || ReadValue(iterator.key()) != iterator->initial_value;
  }
  if (valid_ != valid) {
    valid_ = valid;
    emit ValidityChanged(valid_);
  }
  if (dirty_ != dirty) {
    dirty_ = dirty;
    emit DirtyChanged(dirty_);
  }
  if (submit_button_ != nullptr) {
    submit_button_->setEnabled(valid_);
  }
}

}  // namespace autoviz::widget
