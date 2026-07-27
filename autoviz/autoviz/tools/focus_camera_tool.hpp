/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class FocusCameraTool : public common::Tool {
 public:
  std::string id() const override { return "FocusCamera"; }
  QString label() const override { return QStringLiteral("Focus Camera"); }

  bool mousePressEvent(QMouseEvent* event) override;
  QString statusText() const override;
};

}  // namespace tools
}  // namespace autoviz
