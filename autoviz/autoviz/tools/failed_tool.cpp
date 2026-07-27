/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/failed_tool.hpp"

namespace autoviz {
namespace tools {

FailedTool::FailedTool(std::string id, std::string reason)
    : id_(std::move(id)), reason_(std::move(reason)) {}

QString FailedTool::label() const {
  return QStringLiteral("Failed: %1").arg(QString::fromStdString(id_));
}

QString FailedTool::statusText() const {
  return QString::fromStdString(reason_.empty() ? "Failed to load tool"
                                                : reason_);
}

}  // namespace tools
}  // namespace autoviz
