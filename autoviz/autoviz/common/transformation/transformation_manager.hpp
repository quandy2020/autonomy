/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file transformation_manager.hpp
 * @brief Selects and persists the active @ref FrameTransformer (RViz-style).
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoviz/common/config.hpp"
#include "autoviz/common/plugin_info.hpp"
#include "autoviz/common/transformation/frame_transformer.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {

class FrameManager;
class SessionConfig;

namespace transformation {

/**
 * @class TransformationManager
 * @brief Manages available FrameTransformers and the currently selected one.
 */
class TransformationManager {
 public:
  TransformationManager() = default;

  explicit TransformationManager(transform::Buffer* buffer);

  void initialize(transform::Buffer* buffer);

  void setFrameManager(FrameManager* frame_manager);

  void registerBuiltinTransformers();

  void registerTransformer(const std::string& class_id,
                           FrameTransformerCreator creator);

  std::vector<PluginInfo> availableTransformers() const;

  PluginInfo currentTransformerInfo() const;

  std::string currentTransformerId() const;

  void setTransformer(const std::string& class_id);

  void setTransformer(const PluginInfo& info);

  void loadFromSession(const SessionConfig& config);

  void saveToSession(SessionConfig* config) const;

  void load(const Config& config);

  void save(Config config) const;

  static std::string mapRvizTransformerClass(const std::string& rviz_class);

  void loadPluginsFromEnv();

  void loadPluginsFromPath(const std::string& path);

  void setConfigChangedCallback(std::function<void()> callback);

  FrameTransformer* currentTransformer() const { return current_.get(); }

  std::function<void()> addTransformerChangedCallback(
      std::function<void(FrameTransformer*)> callback);

 private:
  void applyCurrentTransformer();

  void notifyTransformerChanged();

  transform::Buffer* buffer_ = nullptr;
  FrameManager* frame_manager_ = nullptr;
  std::unordered_map<std::string, FrameTransformerCreator> creators_;
  std::unique_ptr<FrameTransformer> current_;
  std::string current_id_;
  std::vector<void*> plugin_handles_;
  std::function<void()> config_changed_callback_;
  std::vector<std::function<void(FrameTransformer*)>>
      transformer_changed_callbacks_;
};

}  // namespace transformation
}  // namespace common
}  // namespace autoviz
