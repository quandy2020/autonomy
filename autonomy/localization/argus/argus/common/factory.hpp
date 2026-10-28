/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file factory.hpp
 * @brief Identifier-keyed factory for interface implementations.
 */

#ifndef ARGUS_COMMON_FACTORY_HPP_
#define ARGUS_COMMON_FACTORY_HPP_

#include <map>
#include <memory>
#include <utility>

namespace argus {
namespace common {

/**
 * @brief Registers creators by identifier and constructs products on demand.
 *
 * Every product implements @p AbstractProduct. Callers register an id before
 * @ref CreateObject. The concrete type is chosen at the registration site.
 *
 * @tparam IdentifierType  Key type, typically `std::string`.
 * @tparam AbstractProduct Interface implemented by registered products.
 * @tparam ProductCreator  Callable returning a raw @p AbstractProduct pointer.
 *                         The factory wraps that pointer in `std::unique_ptr`.
 * @tparam MapContainer    Map from identifier to creator. Defaults to
 *                         `std::map`.
 */
template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct* (*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory {
 public:
  /**
   * @brief Register @p creator under @p id.
   * Registration must happen before @ref CreateObject.
   * @param[in] id      Identifier of the class being registered.
   * @param[in] creator Function returning a new instance of that class.
   * @return `true` if @p id was not already registered.
   */
  bool Register(const IdentifierType& id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }

  /**
   * @brief Whether @p id has a registered creator.
   * @param[in] id Identifier to look up.
   * @return `true` if a creator is registered for @p id.
   */
  bool Contains(const IdentifierType& id) const {
    return producers_.find(id) != producers_.end();
  }

  /**
   * @brief Remove the creator registered under @p id.
   * @param[in] id Identifier to unregister.
   * @return `true` if a creator was removed.
   */
  bool Unregister(const IdentifierType& id) {
    return producers_.erase(id) == 1;
  }

  /** @brief Remove every registered creator. */
  void Clear() { producers_.clear(); }

  /**
   * @brief Whether no creators are registered.
   * @return `true` if the registry is empty.
   */
  bool Empty() const { return producers_.empty(); }

  /**
   * @brief Construct a product for @p id, or return null if unknown.
   * @param[in] id   Identifier previously passed to @ref Register.
   * @param[in] args Arguments forwarded to the registered creator.
   * @return Owning pointer, or `nullptr` if @p id is not registered.
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType& id,
                                                      Args&&... args) const {
    auto id_iter = producers_.find(id);
    if (id_iter == producers_.end()) {
      return nullptr;
    }
    return std::unique_ptr<AbstractProduct>(
        (id_iter->second)(std::forward<Args>(args)...));
  }

  /**
   * @brief Construct a product for @p id.
   * An unknown id yields `nullptr`, same as @ref CreateObjectOrNull.
   * @param[in] id   Identifier previously passed to @ref Register.
   * @param[in] args Arguments forwarded to the registered creator.
   * @return Owning pointer, or `nullptr` if @p id is not registered.
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType& id,
                                                Args&&... args) const {
    return CreateObjectOrNull(id, std::forward<Args>(args)...);
  }

 private:
  MapContainer producers_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_FACTORY_HPP_
