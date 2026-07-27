/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/** rviz_common::FailedTool — placeholder when a tool plugin cannot load. */
class FailedTool : public common::Tool {
 public:
  FailedTool(std::string id, std::string reason);

  std::string id() const override { return id_; }
  QString label() const override;
  QString statusText() const override;

 private:
  std::string id_;
  std::string reason_;
};

}  // namespace tools
}  // namespace autoviz
