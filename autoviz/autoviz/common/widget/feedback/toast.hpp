/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QFrame>
#include <QString>

#include "autoviz/common/widget/feedback/info_bar.hpp"

class QEvent;
class QHideEvent;
class QLabel;
class QPushButton;
class QTimer;

namespace autoviz {
namespace widget {

// A non-modal notification anchored to the bottom center of its parent.
class Toast : public QFrame {
  Q_OBJECT

 public:
  explicit Toast(QWidget* parent = nullptr);

  QString message() const;
  void SetMessage(const QString& message);

  FeedbackLevel level() const;
  void SetLevel(FeedbackLevel level);

  int duration_ms() const;
  void SetDurationMs(int duration_ms);

  void Show();
  void Dismiss();

  static Toast* ShowMessage(QWidget* parent, const QString& message,
                            FeedbackLevel level = FeedbackLevel::kInformation,
                            int duration_ms = 4000);

 signals:
  void Dismissed();

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;
  void changeEvent(QEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private:
  void Reposition();
  void UpdateIcon();

  FeedbackLevel level_ = FeedbackLevel::kInformation;
  int duration_ms_ = 4000;
  QLabel* icon_label_ = nullptr;
  QLabel* message_label_ = nullptr;
  QPushButton* close_button_ = nullptr;
  QTimer* dismiss_timer_ = nullptr;
  bool dismissed_ = false;
};

}  // namespace widget
}  // namespace autoviz
