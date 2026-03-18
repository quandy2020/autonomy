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

#pragma once


#include <cstdint>
#include <memory>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "frame_manager_iface.hpp"

class QKeyEvent;

namespace Ogre
{
class SceneManager;
}  // namespace Ogre

namespace rclcpp
{
class Clock;
}  // namespace rclcpp

// namespace tf
// {
// class TransformListener;
// }

namespace autoview {
namespace common {

namespace interaction {
class SelectionManagerIface;
class HandlerManagerIface;
class ViewPickerIface;

}  // namespace interaction

namespace transformation
{

class TransformationManager;

}  // namespace transformation

class BitAllocator;
class DisplayFactory;
class DisplayGroup;
class FrameManager;
class RenderPanel;
class ToolManager;
class ViewportMouseEvent;
class ViewManager;
class WindowManagerInterface;

/// Pure-virtual base class for objects which give Display subclasses context in which to work.
/**
 * This interface class mainly exists to enable more isolated unit
 * tests by enabling small mock objects to take the place of the large
 * VisualizationManager implementation class.
 * It also serves to define a narrower, and more maintainable API for use in
 * the Display plugins.
 */
class DisplayContext : public QObject
{
  Q_OBJECT

public:
  // TODO(wjwwood): I disabled many of the convenience functions so that each
  //                use of them will cause a compiler error, at which point
  //                we can evaluate if that function needs to be reenabled or
  //                the calling code needs to be changed on a case by case basis.
  //                Once we've done that, the TODO is to remove the still
  //                disabled functions from this interface to keep it small
  //                and to keep it with as few dependencies as possible.

  // TODO(wjwwood): refactor this to return something from rviz_rendering without Ogre
  /// Returns the Ogre::SceneManager used for the main RenderPanel.
  virtual Ogre::SceneManager * getSceneManager() const = 0;

  /// Return the window manager, if any.
  virtual
  WindowManagerInterface *
  getWindowManager() const = 0;

  /// Return a pointer to the SelectionManager.
  virtual
  std::shared_ptr<autoview::common::interaction::SelectionManagerIface>
  getSelectionManager() const = 0;

  /// Return a pointer to the HandlerManager.
  virtual
  std::shared_ptr<autoview::common::interaction::HandlerManagerIface>
  getHandlerManager() const = 0;

  /// Return a pointer to the ViewPicker.
  virtual
  std::shared_ptr<autoview::common::interaction::ViewPickerIface>
  getViewPicker() const = 0;

  /// Return the FrameManager instance.
  virtual
  FrameManagerIface *
  getFrameManager() const = 0;

  /// Return the fixed frame name.
  virtual
  QString
  getFixedFrame() const = 0;

  // TODO(wjwwood): replace uint64_t with size_t unless there is a reason for it
  /// Return the current value of the frame count.
  /**
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something.
   */
  virtual
  uint64_t
  getFrameCount() const = 0;

  /// Return a factory for creating Display subclasses based on a class id string.
  virtual
  DisplayFactory *
  getDisplayFactory() const = 0;

  /// Return a weak pointer to the ros node (abstraction) used by rviz
  virtual
  ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const = 0;

#if 0
  /// Return a CallbackQueue using a different thread than the main GUI one.
  virtual ros::CallbackQueueInterface * getThreadedQueue() = 0;
#endif

  /// Handle a single key event for a given RenderPanel.
  virtual
  void
  handleChar(QKeyEvent * event, RenderPanel * panel) = 0;

  /// Handle a mouse event.
  virtual
  void
  handleMouseEvent(const ViewportMouseEvent & event) = 0;

  /// Return the ToolManager.
  virtual
  ToolManager *
  getToolManager() const = 0;

  /// Return the ViewManager.
  virtual
  ViewManager *
  getViewManager() const = 0;

  virtual
  transformation::TransformationManager *
  getTransformationManager() = 0;

  /// Return the root DisplayGroup.
  virtual
  DisplayGroup *
  getRootDisplayGroup() const = 0;

  /// Get the default visibility bit.
  virtual
  uint32_t
  getDefaultVisibilityBit() const = 0;

  /// Get the visibility bits.
  virtual
  BitAllocator *
  visibilityBits() = 0;

  /// Set the message displayed in the status bar.
  virtual
  void
  setStatus(const QString & message) = 0;

  virtual
  QString
  getHelpPath() const = 0;

  virtual
  std::shared_ptr<rclcpp::Clock>
  getClock() = 0;

  virtual
  void
  lockRender() = 0;

  virtual
  void
  unlockRender() = 0;

public Q_SLOTS:
  /// Queue a render.
  /**
   * Multiple calls before a render happens will only cause a single render.
   *
   * \note This function can be called from any thread.
   */
  virtual
  void
  queueRender() = 0;
};

}  // namespace common
}  // namespace autoview

