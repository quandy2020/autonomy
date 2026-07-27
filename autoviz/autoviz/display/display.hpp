/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <QImage>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/config.hpp"
#include "autoviz/common/session_config.hpp"
#include "autoviz/common/display_status.hpp"
#include "autoviz/integration/autolink_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

class Display {
 public:
  virtual ~Display() = default;

  virtual std::string typeId() const { return "Display"; }
  virtual std::string name() const { return display_name_.empty() ? typeId() : display_name_; }
  virtual std::string channel() const { return ""; }
  virtual void setChannel(const std::string& /*channel*/) {}
  virtual bool enabled() const { return enabled_; }

  DisplayStatus status() const {
    return {status_level_, status_message_};
  }

  virtual std::vector<common::DisplayPropertySpec> propertySpecs() const {
    return {};
  }
  void setProperties(const common::DisplayPropertyMap& properties);
  const common::DisplayPropertyMap& properties() const { return properties_; }
  std::string propertyValue(const std::string& key,
                            const std::string& default_value) const;
  void setPropertyValue(const std::string& key, const std::string& value);

  /** rviz_common-style Config persistence. */
  virtual void load(const common::Config& config);
  virtual void save(common::Config config) const;
  void loadFromConfig(const common::DisplayConfig& config);
  virtual void saveToConfig(common::DisplayConfig* config) const;

  void setDisplayName(const std::string& name) { display_name_ = name; }
  void setContext(common::DisplayContext* context) { context_ = context; }

  void setEnabled(bool enabled);
  void update();
  void draw(rendering::SceneOverlay& scene);

  void setVisibilityBits(uint32_t bits);
  void unsetVisibilityBits(uint32_t bits);
  uint32_t visibilityBits() const { return visibility_bits_; }

 protected:
  virtual void onEnable() {}
  virtual void onDisable() {}
  virtual void onUpdate() {}
  virtual void onPropertyChanged(const std::string& /*key*/) {}
  virtual void onDraw(rendering::SceneOverlay& scene) = 0;

  void resetStatus();
  void setStatusOk();
  void setStatusWarn(const std::string& message);
  void setStatusError(const std::string& message);

  common::DisplayContext* context_ = nullptr;

 private:
  std::string display_name_;
  common::DisplayPropertyMap properties_;
  bool enabled_ = true;
  bool subscribed_ = false;
  uint32_t visibility_bits_ = 0xFFFFFFFFu;
  DisplayStatusLevel status_level_ = DisplayStatusLevel::kOk;
  std::string status_message_;
};

}  // namespace display
}  // namespace autoviz
