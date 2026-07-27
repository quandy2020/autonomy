/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class MoveCameraTool : public common::Tool {
 public:
  std::string id() const override { return "MoveCamera"; }
  QString label() const override { return QStringLiteral("Move Camera"); }
  QString statusText() const override;
};

}  // namespace tools
}  // namespace autoviz
