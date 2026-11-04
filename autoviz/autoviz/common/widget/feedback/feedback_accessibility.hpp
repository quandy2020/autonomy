/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QCoreApplication>
#include <QString>

#include "autoviz/common/widget/feedback/info_bar.hpp"

namespace autoviz::widget::internal {

inline QString FeedbackLevelText(FeedbackLevel level) {
  switch (level) {
    case FeedbackLevel::kInformation:
      return QCoreApplication::translate("AutovizFeedback", "Information");
    case FeedbackLevel::kSuccess:
      return QCoreApplication::translate("AutovizFeedback", "Success");
    case FeedbackLevel::kWarning:
      return QCoreApplication::translate("AutovizFeedback", "Warning");
    case FeedbackLevel::kError:
      return QCoreApplication::translate("AutovizFeedback", "Error");
  }
  return QCoreApplication::translate("AutovizFeedback", "Information");
}

inline QString FeedbackLevelAccessibleName(FeedbackLevel level) {
  return QCoreApplication::translate("AutovizFeedback", "%1 notification")
      .arg(FeedbackLevelText(level));
}

inline QString FeedbackAccessibleDescription(FeedbackLevel level,
                                             const QString& content) {
  const QString level_name = FeedbackLevelAccessibleName(level);
  if (content.isEmpty()) {
    return level_name;
  }
  return QCoreApplication::translate("AutovizFeedback", "%1. %2")
      .arg(level_name, content);
}

}  // namespace autoviz::widget::internal
