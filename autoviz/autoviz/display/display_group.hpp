/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/** rviz_common::DisplayGroup equivalent — container for nested displays. */
class DisplayGroup : public Display {
 public:
  std::string typeId() const override { return "Group"; }

  void addChild(std::unique_ptr<Display> child);
  void insertChild(std::size_t index, std::unique_ptr<Display> child);
  std::unique_ptr<Display> takeChild(std::size_t index);
  const std::vector<std::unique_ptr<Display>>& children() const {
    return children_;
  }
  Display* child(std::size_t index);
  const Display* child(std::size_t index) const;

  void reset() override;
  void load(const common::Config& config) override;
  void save(common::Config config) const override;
  void saveToConfig(common::DisplayConfig* config) const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::vector<std::unique_ptr<Display>> children_;
};

}  // namespace display
}  // namespace autoviz
