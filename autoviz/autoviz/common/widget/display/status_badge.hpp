/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QEvent;
class QPaintEvent;
class QResizeEvent;

namespace autoviz {
namespace widget {

/**
 * @brief A small semantic status pill for connection and runtime state.
 */
class StatusBadge final : public QWidget {
  Q_OBJECT

 public:
  enum class Status {
    kNeutral,
    kInformation,
    kSuccess,
    kWarning,
    kError,
  };
  Q_ENUM(Status)
  Q_PROPERTY(QString text READ text WRITE SetText NOTIFY TextChanged)
  Q_PROPERTY(Status status READ status WRITE SetStatus NOTIFY StatusChanged)

  explicit StatusBadge(QWidget* parent = nullptr);
  explicit StatusBadge(const QString& text, Status status = Status::kNeutral,
                       QWidget* parent = nullptr);

  QString text() const;
  Status status() const;
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 public slots:
  void SetText(const QString& text);
  void SetStatus(Status status);

 signals:
  void TextChanged(const QString& text);
  void StatusChanged(Status status);

 protected:
  void changeEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  void RefreshElisionState();
  void UpdateAccessibilityDescription();

  QString automatic_tool_tip_;
  QString text_;
  Status status_ = Status::kNeutral;
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace widget
}  // namespace autoviz
