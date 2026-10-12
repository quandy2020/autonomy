/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common::PluginlibFactory
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file plugin_factory.hpp
 * @brief Built-in (+ optional dynamic) plugin factory without pluginlib.
 *
 * RViz @c PluginlibFactory wraps @c pluginlib::ClassLoader. Autoviz uses
 * @ref PluginLoader + @c AUTOVIZ_PLUGIN_PATH at the registry layer; this class
 * owns the in-process built-in map and a secondary creator map that registries
 * can populate after scanning shared libraries.
 *
 * @see ClassIdRecordingFactory
 * @see PluginLoader
 * @see DisplayRegistry
 */

#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <QHash>
#include <QString>

#include "autoviz/common/factory/class_id_recording_factory.hpp"
#include "autoviz/common/plugin_info.hpp"
#include "autoviz/ui/app/icon_loader.hpp"

namespace autoviz {
namespace common {
namespace factory {

/**
 * @class PluginFactory
 * @brief RViz PluginlibFactory analogue backed by built-in creators.
 *
 * @tparam Type Product base type.
 *
 * ## Registration
 *
 * - @ref addBuiltInClass — id @c "package/name" (RViz style)
 * - @ref addClass — arbitrary @p class_id (Autoviz type ids like @c "Grid")
 */
template <class Type>
class PluginFactory : public ClassIdRecordingFactory<Type> {
 private:
  struct ClassRecord {
    QString class_id;
    QString package;
    QString name;
    QString description;
    std::function<Type*()> factory_function;
  };

 public:
  PluginFactory() = default;
  ~PluginFactory() override = default;

  /**
   * @brief Returns built-in and dynamically added plugin metadata.
   * @return Plugin list (order not guaranteed).
   */
  std::vector<PluginInfo> getDeclaredPlugins() override {
    std::vector<PluginInfo> plugins;
    plugins.reserve(static_cast<size_t>(classes_.size()));
    for (auto it = classes_.constBegin(); it != classes_.constEnd(); ++it) {
      plugins.push_back(getPluginInfo(it.key()));
    }
    return plugins;
  }

  /**
   * @brief Looks up metadata for @p class_id.
   *
   * @param class_id Plugin class id.
   * @return Filled @ref PluginInfo, or id-only stub if unknown.
   */
  PluginInfo getPluginInfo(const QString& class_id) const override {
    PluginInfo info;
    info.class_id = class_id.toStdString();
    const auto it = classes_.constFind(class_id);
    if (it == classes_.constEnd()) {
      return info;
    }
    info.name = it->name.toStdString();
    info.package = it->package.toStdString();
    info.description = it->description.toStdString();
    info.icon = getIcon(info);
    return info;
  }

  /**
   * @brief Registers a built-in class as @c package/name (RViz style).
   *
   * @param package Package / plugin package string (e.g. @c "autoviz").
   * @param name Short class name (e.g. @c "Grid").
   * @param description Human-readable help text.
   * @param factory_function Nullary creator returning a new @tparam Type.
   */
  virtual void addBuiltInClass(const QString& package, const QString& name,
                               const QString& description,
                               std::function<Type*()> factory_function) {
    ClassRecord record;
    record.class_id = package + QStringLiteral("/") + name;
    record.package = package;
    record.name = name;
    record.description = description;
    record.factory_function = std::move(factory_function);
    classes_.insert(record.class_id, record);
  }

  /**
   * @brief Registers a class under an explicit @p class_id.
   *
   * Preferred for Autoviz type ids that omit the @c package/ prefix
   * (e.g. @c "PointCloud2").
   *
   * @param class_id Stable registry id.
   * @param package Package string for metadata / icons.
   * @param name Short display name.
   * @param description Help text.
   * @param factory_function Nullary creator.
   */
  virtual void addClass(const QString& class_id, const QString& package,
                        const QString& name, const QString& description,
                        std::function<Type*()> factory_function) {
    ClassRecord record;
    record.class_id = class_id;
    record.package = package;
    record.name = name;
    record.description = description;
    record.factory_function = std::move(factory_function);
    classes_.insert(record.class_id, record);
  }

  /**
   * @brief Returns whether @p class_id has a registered creator.
   *
   * @param class_id Plugin class id.
   * @return @c true when @ref make can succeed for @p class_id.
   */
  bool hasClass(const QString& class_id) const {
    return classes_.contains(class_id);
  }

 protected:
  /**
   * @brief Instantiates via the registered creator for @p class_id.
   *
   * @param class_id Class id.
   * @param error_return Optional error string out-parameter.
   * @return New instance, or @c nullptr.
   */
  Type* makeRaw(const QString& class_id,
                QString* error_return = nullptr) override {
    const auto it = classes_.constFind(class_id);
    if (it == classes_.constEnd()) {
      if (error_return != nullptr) {
        *error_return =
            QStringLiteral("PluginFactory: unknown class '%1'.").arg(class_id);
      }
      return nullptr;
    }
    Type* instance = it->factory_function();
    if (instance == nullptr && error_return != nullptr) {
      *error_return =
          QStringLiteral(
              "Factory function for class '%1' returned nullptr.")
              .arg(class_id);
    }
    return instance;
  }

  /**
   * @brief Resolves a class icon via @ref IconLoader.
   *
   * @param info Plugin metadata (uses @c name, then @c class_id basename).
   * @return Icon (may be null).
   */
  virtual QIcon getIcon(const PluginInfo& info) const {
    QString name = QString::fromStdString(info.name);
    if (name.isEmpty()) {
      const QString id = QString::fromStdString(info.class_id);
      const int slash = id.lastIndexOf(QLatin1Char('/'));
      name = slash >= 0 ? id.mid(slash + 1) : id;
    }
    if (name.isEmpty()) {
      return {};
    }
    return IconLoader::displayIcon(name);
  }

 private:
  QHash<QString, ClassRecord> classes_;
};

}  // namespace factory
}  // namespace common
}  // namespace autoviz
