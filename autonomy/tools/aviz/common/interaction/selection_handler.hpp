/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_
#define AVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/tools/aviz/common/interaction/forwards.hpp"
#include "autonomy/tools/aviz/common/properties/property.hpp"

// Forward declaration
class RenderableObject;

namespace aviz {
namespace common {

class DisplayContext;
class ViewportMouseEvent;

namespace interaction {

/// Use this function to create a SelectionHandler
/**
 * This function creates a shared_ptr for any SelectionHandler, registering a handle with the
 * HandlerManager.
 */
template <typename T, typename... Args>
std::shared_ptr<T> createSelectionHandler(Args... arguments) {
  auto selection_handler = std::shared_ptr<T>(new T(arguments...));
  selection_handler->registerHandle();
  return selection_handler;
}

class SelectionHandler : public std::enable_shared_from_this<SelectionHandler> {
 public:
  virtual ~SelectionHandler();

  void addTrackedObject(RenderableObject* object);
  void removeTrackedObject(RenderableObject* object);

  virtual void updateTrackedBoxes();

  /// Override to create properties of the given picked object(s).
  /**
   * Top-level properties created here should be added to properties_ so they
   * will be automatically deleted by deleteProperties().
   *
   * This base implementation does nothing.
   */
  virtual void createProperties(const Picked& obj, aviz::common::properties::Property* parent_property);

  /// Destroy all properties for the given picked object(s).
  /**
   * This base implementation destroys all the properties in properties_.
   *
   * If createProperties() adds all the top-level properties to properties_,
   * there is no need to override this in a subclass.
   */
  virtual void destroyProperties(const Picked& obj, aviz::common::properties::Property* parent_property);

  /** @brief Override to update property values.
   *
   * updateProperties() is called on a timer to give selection
   * handlers a chance to update displayed property values.
   * Subclasses with properties that can change should implement this
   * to update the property values based on new information from the
   * selected object(s).
   *
   * This base implementation does nothing.
   */
  virtual void updateProperties();

  /// Override to indicate if an additional render pass is required.
  virtual bool needsAdditionalRenderPass(uint32_t pass);

  /// Override to hook before a render pass.
  virtual void preRenderPass(uint32_t pass);

  /// Override to hook after a render pass.
  virtual void postRenderPass(uint32_t pass);

  /// Override to get called on selection.
  virtual void onSelect(const Picked& obj);

  /// Override to get called on deselection.
  virtual void onDeselect(const Picked& obj);

  /// Get CollObjectHandle.
  CollObjectHandle getHandle() const;

 protected:
  explicit SelectionHandler(DisplayContext* context);

  void registerHandle();

  QList<aviz::common::properties::Property*> properties_;

  DisplayContext* context_;

  using S_Renderable = std::set<RenderableObject*>;
  S_Renderable tracked_objects_;

 private:
  // pick_handle_ must never be changed, otherwise the destructor will
  // call removeObject() with the wrong handle.  Use getHandle() to
  // access the value.
  CollObjectHandle pick_handle_;

  friend class SelectionManager;
  template <typename T, typename... Args>
  friend typename std::shared_ptr<T> aviz::common::interaction::createSelectionHandler(Args... arguments);
};

using SelectionHandlerPtr = std::shared_ptr<SelectionHandler>;
using SelectionHandlerWeakPtr = std::weak_ptr<SelectionHandler>;
using V_SelectionHandler = std::vector<SelectionHandlerPtr>;
using S_SelectionHandler = std::set<SelectionHandlerPtr>;

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_
