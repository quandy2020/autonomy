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

#ifndef AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_
#define AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_

#include <iterator>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "autonomy/tools/aviz/common/interaction/handler_manager_listener.hpp"
#include "autonomy/tools/aviz/common/interaction/selection_handler.hpp"

namespace aviz {
namespace common {
namespace interaction {

using M_ObjectHandleToSelectionHandler = std::unordered_map<CollObjectHandle, SelectionHandlerWeakPtr>;

/**
 * Light-weight container wrapper for M_ObjectHandleToSelectionHandler that allows iterating
 * directly over handlers.
 */
class HandlerRange {
 public:
  class iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = SelectionHandlerWeakPtr;
    using difference_type = std::ptrdiff_t;
    using pointer = SelectionHandlerWeakPtr*;
    using reference = SelectionHandlerWeakPtr&;

    reference operator*() const { return iterator_->second; }

    bool operator!=(iterator other) { return iterator_ != other.iterator_; }

    iterator& operator++() {
      ++iterator_;
      return *this;
    }

    iterator operator++(int) { return iterator(iterator_++); }

   private:
    explicit iterator(M_ObjectHandleToSelectionHandler::iterator it) : iterator_(it) {}

    M_ObjectHandleToSelectionHandler::iterator iterator_;

    friend class HandlerRange;
  };

  explicit HandlerRange(M_ObjectHandleToSelectionHandler& handlers) : handlers_(handlers) {}

  iterator begin() { return iterator(handlers_.begin()); }

  iterator end() { return iterator(handlers_.end()); }

 private:
  M_ObjectHandleToSelectionHandler& handlers_;
};

/**
 * \brief The HandlerManagerIface manages selection handlers
 *
 * It is mainly used by the SelectionManager
 *
 * The HandlerManager must be locked (using one of the lock methods) if exclusive access to the
 * handlers is necessary.
 */
class HandlerManagerIface {
 public:
  virtual ~HandlerManagerIface() = default;

  /**
   * Registers a new handle-handler pair. Locks internally to guarantee exclusive access.
   * \param handle
   * \param handler
   */
  virtual void addHandler(CollObjectHandle handle, SelectionHandlerWeakPtr handler) = 0;

  /**
   * Removes a handle-handler pair identified by its handle. Locks internally to guarantee
   * exclusive access
   * \param handle
   */
  virtual void removeHandler(CollObjectHandle handle) = 0;

  /// obtains the handler for a handle
  virtual SelectionHandlerPtr getHandler(CollObjectHandle handle) = 0;

  /**
   * Locks the handlers container
   * \return the lock for the handlers (already locked)
   */
  virtual std::unique_lock<std::recursive_mutex> lock() = 0;

  /**
   * Returns a lock for the handlers container that is not yet locked
   * \param defer_lock std::defer_lock
   * \return the lock for the handlers (not yet locked)
   */
  virtual std::unique_lock<std::recursive_mutex> lock(std::defer_lock_t defer_lock) = 0;

  /**
   * Registers a listener that is notified for every handle that is removed
   * \param listener
   */
  virtual void addListener(HandlerManagerListener* listener) = 0;

  /**
   * Removes a listeners.
   * \param listener
   */
  virtual void removeListener(HandlerManagerListener* listener) = 0;

  /**
   * Creates a new unique handle.
   * \return new handle
   */
  virtual CollObjectHandle createHandle() = 0;

  /// Tells all handlers that interactive mode is active/inactive.
  virtual void enableInteraction(bool enable) = 0;

  /// Retrieves the current interaction mode (active/inactive).
  virtual bool getInteractionEnabled() const = 0;

  /**
   * Gives access to all managed handlers in form of a container that can be accessed via a
   * forward iterator.
   * Its usage needs to be protected by explicitly calling lock
   * \return container of all handlers
   */
  virtual HandlerRange handlers() = 0;
};

using HandlerManagerIfacePtr = std::shared_ptr<HandlerManagerIface>;

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_
