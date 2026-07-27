/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/transformation_manager.hpp"

#include <QDir>
#include <QFileInfo>
#include <QString>

#include "autoviz/common/frame_manager.hpp"
#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"
#include "autoviz/common/session_config.hpp"

namespace autoviz {
namespace common {
namespace {

constexpr const char* kDefaultTransformerId = "autoviz/AutolinkTf";

}  // namespace

TransformationManager::TransformationManager(transform::Buffer* buffer) {
  initialize(buffer);
}

void TransformationManager::initialize(transform::Buffer* buffer) {
  if (buffer == nullptr || buffer_ == buffer) {
    return;
  }
  buffer_ = buffer;
  if (creators_.empty()) {
    registerBuiltinTransformers();
    loadPluginsFromEnv();
  }
  if (current_id_.empty()) {
    setTransformer(kDefaultTransformerId);
  } else {
    applyCurrentTransformer();
  }
}

void TransformationManager::setFrameManager(FrameManager* frame_manager) {
  frame_manager_ = frame_manager;
  applyCurrentTransformer();
}

void TransformationManager::registerBuiltinTransformers() {
  registerTransformer(
      "autoviz/AutolinkTf",
      [](transform::Buffer* buffer) {
        return std::make_unique<AutolinkTfFrameTransformer>(buffer);
      });
  registerTransformer(
      "autoviz/Identity",
      [](transform::Buffer*) { return std::make_unique<IdentityFrameTransformer>(); });
}

void TransformationManager::registerTransformer(
    const std::string& class_id, FrameTransformerCreator creator) {
  creators_[class_id] = std::move(creator);
}

std::vector<PluginInfo> TransformationManager::availableTransformers() const {
  std::vector<PluginInfo> infos;
  infos.reserve(creators_.size());
  for (const auto& [class_id, creator] : creators_) {
    (void)class_id;
    if (auto transformer = creator(buffer_)) {
      infos.push_back(transformer->info());
    }
  }
  return infos;
}

PluginInfo TransformationManager::currentTransformerInfo() const {
  if (current_ != nullptr) {
    return current_->info();
  }
  PluginInfo info;
  info.class_id = current_id_;
  return info;
}

std::string TransformationManager::currentTransformerId() const {
  return current_id_;
}

void TransformationManager::setTransformer(const std::string& class_id) {
  const auto it = creators_.find(class_id);
  if (it == creators_.end()) {
    return;
  }
  current_ = it->second(buffer_);
  current_id_ = class_id;
  applyCurrentTransformer();
  if (config_changed_callback_) {
    config_changed_callback_();
  }
}

void TransformationManager::setTransformer(const PluginInfo& info) {
  setTransformer(info.class_id);
}

void TransformationManager::applyCurrentTransformer() {
  if (frame_manager_ == nullptr || current_ == nullptr) {
    return;
  }
  frame_manager_->setIdentityMode(current_->identityMode());
  frame_manager_->setBuffer(current_->buffer());
}

void TransformationManager::loadFromSession(const SessionConfig& config) {
  if (!config.transformer_id.empty() &&
      creators_.find(config.transformer_id) != creators_.end()) {
    setTransformer(config.transformer_id);
  }
}

void TransformationManager::load(const Config& config) {
  Config current = config.mapGetChild("Current");
  QString class_id;
  if (current.mapGetString("Class", &class_id)) {
    setTransformer(mapRvizTransformerClass(class_id.toStdString()));
    return;
  }
  QString transformer;
  if (config.mapGetString("Transformer", &transformer)) {
    setTransformer(transformer.toStdString());
  }
}

void TransformationManager::save(Config config) const {
  Config current = config.mapMakeChild("Current");
  current.mapSetValue("Class", QString::fromStdString(current_id_));
}

std::string TransformationManager::mapRvizTransformerClass(
    const std::string& rviz_class) {
  if (rviz_class.find("Identity") != std::string::npos) {
    return "autoviz/Identity";
  }
  return "autoviz/AutolinkTf";
}

void TransformationManager::saveToSession(SessionConfig* config) const {
  if (config == nullptr) {
    return;
  }
  config->transformer_id = current_id_;
}

void TransformationManager::loadPluginsFromEnv() {
  for (const QString& part : pluginSearchPaths()) {
    loadPluginsFromPath(part.toStdString());
  }
}

void TransformationManager::loadPluginsFromPath(const std::string& path) {
  QDir dir(QString::fromStdString(path));
  if (!dir.exists()) {
    return;
  }
  for (const QFileInfo& info : dir.entryInfoList(
           PluginLoader::libraryFilenameFilters(), QDir::Files | QDir::Readable)) {
    void* handle = PluginLoader::open(info.absoluteFilePath().toStdString());
    if (handle == nullptr) {
      continue;
    }
    using RegisterFn = void (*)(TransformationManager*);
    auto* register_fn = reinterpret_cast<RegisterFn>(
        PluginLoader::symbol(handle, "autoviz_register_transformers"));
    if (register_fn == nullptr) {
      PluginLoader::close(handle);
      continue;
    }
    register_fn(this);
    plugin_handles_.push_back(handle);
  }
}

void TransformationManager::setConfigChangedCallback(
    std::function<void()> callback) {
  config_changed_callback_ = std::move(callback);
}

}  // namespace common
}  // namespace autoviz
