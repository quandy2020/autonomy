/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QFrame>
#include <QString>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QLabel;
class QPaintEvent;
class QPushButton;

namespace autoviz {
namespace widget {

// Semantic level shared by transient and persistent feedback widgets.
enum class FeedbackLevel {
  kInformation,
  kSuccess,
  kWarning,
  kError,
};

// A compact, accessible notification banner. The widget uses the current
// QPalette and therefore follows application theme changes without global QSS.
class InfoBar : public QFrame {
  Q_OBJECT

 public:
  explicit InfoBar(QWidget* parent = nullptr);
  InfoBar(const QString& title, const QString& message,
          FeedbackLevel level = FeedbackLevel::kInformation,
          QWidget* parent = nullptr);

  QString title() const;
  void SetTitle(const QString& title);

  QString message() const;
  void SetMessage(const QString& message);

  FeedbackLevel level() const;
  void SetLevel(FeedbackLevel level);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  bool is_closable() const;
  void SetClosable(bool closable);

  QSize sizeHint() const override;

 signals:
  void CloseRequested();

 protected:
  void changeEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  void Initialize();
  void UpdatePresentation();
  void UpdateSemanticColors();

  FeedbackLevel level_ = FeedbackLevel::kInformation;
  QLabel* icon_label_ = nullptr;
  QLabel* title_label_ = nullptr;
  QLabel* message_label_ = nullptr;
  QPushButton* close_button_ = nullptr;
  QColor background_color_;
  QColor border_color_;
  style::Tone tone_ = style::Tone::Shell;
  bool closable_ = true;
};

}  // namespace widget
}  // namespace autoviz
