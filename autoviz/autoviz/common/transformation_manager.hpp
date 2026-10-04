/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file transformation_manager.hpp
 * @brief Selects and persists the active @ref FrameTransformer (RViz-style).
 *
 * Owns the registry of transformer creators, the current instance, and
 * optional dynamically loaded plugins. Syncs the choice with @ref FrameManager
 * and @ref SessionConfig.
 *
 * @see FrameTransformer
 * @see FrameManager
 * @see TransformerPlugin
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoviz/common/config.hpp"
#include "autoviz/common/frame_transformer.hpp"
#include "autoviz/common/plugin_info.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {

class FrameManager;
class SessionConfig;

/**
 * @class TransformationManager
 * @brief Manages available FrameTransformers and the currently selected one.
 */
class TransformationManager {
 public:
  /**
   * @brief Default-constructs with no buffer (call @ref initialize later).
   */
  TransformationManager() = default;

  /**
   * @brief Constructs and binds a TF buffer immediately.
   * @param buffer Non-owning buffer passed to transformer creators.
   */
  explicit TransformationManager(transform::Buffer* buffer);

  /**
   * @brief Binds (or rebinds) the TF buffer used by transformers.
   * @param buffer Non-owning buffer pointer.
   */
  void initialize(transform::Buffer* buffer);

  /**
   * @brief Attaches the @ref FrameManager that mirrors identity mode / buffer.
   * @param frame_manager Non-owning pointer; may be @c nullptr.
   */
  void setFrameManager(FrameManager* frame_manager);

  /**
   * @brief Registers built-in transformers (Autolink TF, Identity, …).
   */
  void registerBuiltinTransformers();

  /**
   * @brief Registers a transformer creator under @p class_id.
   *
   * @param class_id Plugin class id (e.g. @c "autoviz/AutolinkTf").
   * @param creator Factory producing a @ref FrameTransformer.
   */
  void registerTransformer(const std::string& class_id,
                           FrameTransformerCreator creator);

  /**
   * @brief Lists metadata for all registered transformers.
   * @return Plugin info vector for UI.
   */
  std::vector<PluginInfo> availableTransformers() const;

  /**
   * @brief Metadata for the currently active transformer.
   * @return @ref PluginInfo of @c current_ (empty fields if none).
   */
  PluginInfo currentTransformerInfo() const;

  /**
   * @brief Class id of the currently active transformer.
   * @return Id string (may be empty before selection).
   */
  std::string currentTransformerId() const;

  /**
   * @brief Activates the transformer registered under @p class_id.
   * @param class_id Plugin class id.
   */
  void setTransformer(const std::string& class_id);

  /**
   * @brief Activates the transformer described by @p info.
   * @param info Plugin metadata whose @c class_id is looked up.
   */
  void setTransformer(const PluginInfo& info);

  /**
   * @brief Restores the transformer id from session config.
   * @param config Source session.
   */
  void loadFromSession(const SessionConfig& config);

  /**
   * @brief Writes the current transformer id into session config.
   * @param[in,out] config Destination session (non-null).
   */
  void saveToSession(SessionConfig* config) const;

  /**
   * @brief Loads transformer settings from a hierarchical @ref Config node.
   * @param config Source config subtree.
   */
  void load(const Config& config);

  /**
   * @brief Saves transformer settings into a hierarchical @ref Config node.
   * @param config Destination config (by value copy-on-write style).
   */
  void save(Config config) const;

  /**
   * @brief Maps an RViz transformer class string to an Autoviz class id.
   *
   * @param rviz_class RViz plugin class name.
   * @return Autoviz @c class_id, or empty if unmapped.
   */
  static std::string mapRvizTransformerClass(const std::string& rviz_class);

  /**
   * @brief Loads transformer plugins from @c AUTOVIZ_PLUGIN_PATH.
   */
  void loadPluginsFromEnv();

  /**
   * @brief Loads transformer plugins from a directory.
   * @param path Directory to scan.
   */
  void loadPluginsFromPath(const std::string& path);

  /**
   * @brief Registers a callback fired when the active transformer changes.
   * @param callback No-arg functor (e.g. request session dirty / redraw).
   */
  void setConfigChangedCallback(std::function<void()> callback);

 private:
  /**
   * @brief Instantiates @c current_ from @c current_id_ and syncs FrameManager.
   */
  void applyCurrentTransformer();

  transform::Buffer* buffer_ = nullptr;
  FrameManager* frame_manager_ = nullptr;
  std::unordered_map<std::string, FrameTransformerCreator> creators_;
  std::unique_ptr<FrameTransformer> current_;
  std::string current_id_;
  std::vector<void*> plugin_handles_;
  std::function<void()> config_changed_callback_;
};

}  // namespace common
}  // namespace autoviz
