/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common::ClassIdRecordingFactory
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file class_id_recording_factory.hpp
 * @brief Factory that stamps created objects with their class id string.
 *
 * RViz calls @c setClassId() / @c setDescription() on every instance. Autoviz
 * types often use @c typeId() instead; those hooks are invoked only when the
 * product type provides matching methods (C++17 detection).
 *
 * @see Factory
 * @see PluginFactory
 */

#pragma once

#include <type_traits>
#include <utility>

#include <QString>

#include "autoviz/common/factory/factory.hpp"

namespace autoviz {
namespace common {
namespace factory {
namespace detail {

template <typename T, typename = void>
struct HasSetClassId : std::false_type {};

template <typename T>
struct HasSetClassId<
    T, std::void_t<decltype(std::declval<T&>().setClassId(QString()))>>
    : std::true_type {};

template <typename T, typename = void>
struct HasSetDescription : std::false_type {};

template <typename T>
struct HasSetDescription<
    T, std::void_t<decltype(std::declval<T&>().setDescription(QString()))>>
    : std::true_type {};

template <typename T>
void ApplyClassId(T* obj, const QString& class_id) {
  if constexpr (HasSetClassId<T>::value) {
    obj->setClassId(class_id);
  }
  (void)obj;
  (void)class_id;
}

template <typename T>
void ApplyDescription(T* obj, const QString& description) {
  if constexpr (HasSetDescription<T>::value) {
    obj->setDescription(description);
  }
  (void)obj;
  (void)description;
}

}  // namespace detail

/**
 * @class ClassIdRecordingFactory
 * @brief Typed factory that records @p class_id onto created instances.
 *
 * @tparam Type Product base type (e.g. @c display::Display).
 */
template <class Type>
class ClassIdRecordingFactory : public Factory {
 public:
  /**
   * @brief Instantiates a subclass of @tparam Type via @ref makeRaw.
   *
   * On success, optionally calls @c setClassId / @c setDescription when
   * available on @tparam Type.
   *
   * @param class_id Unique class id among siblings (e.g. @c "Grid").
   * @param error_return Optional; set to an error description on failure.
   * @return New instance, or @c nullptr on failure.
   */
  virtual Type* make(const QString& class_id,
                     QString* error_return = nullptr) {
    Type* obj = makeRaw(class_id, error_return);
    if (obj != nullptr) {
      detail::ApplyClassId(obj, class_id);
      detail::ApplyDescription(
          obj, QString::fromStdString(getPluginInfo(class_id).description));
    }
    return obj;
  }

 protected:
  /**
   * @brief Subclass hook that performs the actual construction.
   *
   * @param class_id Class id to instantiate.
   * @param error_return Optional error out-parameter.
   * @return Raw owning pointer, or @c nullptr.
   */
  virtual Type* makeRaw(const QString& class_id,
                        QString* error_return = nullptr) = 0;
};

}  // namespace factory
}  // namespace common
}  // namespace autoviz
