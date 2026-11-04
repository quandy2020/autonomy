/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>
#include <QHash>
#include <QMetaObject>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QVariant>

class QEvent;
class QWidget;

namespace autoviz::widget {

// Coordinates validity and dirty state for native Qt form controls. It does
// not own registered fields and never replaces their validators.
class FormController final : public QObject {
  Q_OBJECT

 public:
  explicit FormController(QObject *parent = nullptr);

  void RegisterField(QWidget *field, bool required = false);
  void UnregisterField(QWidget *field);
  void SetRequired(QWidget *field, bool required);
  void SetFieldError(QWidget *field, const QString &error);
  void ClearFieldError(QWidget *field);
  QString field_error(QWidget *field) const;
  bool is_valid() const noexcept { return valid_; }
  bool is_dirty() const noexcept { return dirty_; }
  void MarkClean();
  void NotifyFieldChanged(QWidget *field);
  void SetSubmitButton(QAbstractButton *button);

 signals:
  void ValidityChanged(bool valid);
  void DirtyChanged(bool dirty);
  void FieldErrorChanged(QWidget *field, const QString &error);

 protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

 private:
  struct FieldState {
    QVariant initial_value;
    QString manual_error;
    QString error;
    QString base_accessible_description;
    QMetaObject::Connection value_connection;
    QMetaObject::Connection destroyed_connection;
    bool required = false;
  };

  QVariant ReadValue(QWidget *field) const;
  QString ValidateField(QWidget *field, const FieldState &state) const;
  void UpdateField(QWidget *field);
  void RecomputeState();

  QHash<QWidget *, FieldState> fields_;
  QPointer<QAbstractButton> submit_button_;
  bool valid_ = true;
  bool dirty_ = false;
};

}  // namespace autoviz::widget
