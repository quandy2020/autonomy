/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/** rviz_common::FailedDisplay — placeholder when a display plugin cannot load. */
class FailedDisplay : public Display {
 public:
  FailedDisplay(std::string type, std::string reason);

  std::string typeId() const override { return type_; }

 protected:
  void onEnable() override;
  void onDraw(rendering::SceneOverlay& /*scene*/) override {}

 private:
  std::string type_;
  std::string reason_;
};

}  // namespace display
}  // namespace autoviz
