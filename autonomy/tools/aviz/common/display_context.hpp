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

#ifndef AVIZ_COMMON__DISPLAY_CONTEXT_HPP_
#define AVIZ_COMMON__DISPLAY_CONTEXT_HPP_

#include <cstdint>
#include <memory>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

class QKeyEvent;
class SceneViewer;
class RenderPanel;
class ViewportMouseEvent;

namespace Ogre {
class SceneManager;
}  // namespace Ogre

namespace aviz {
namespace common {

namespace interaction {

class SelectionManagerIface;
class HandlerManagerIface;
class ViewPickerIface;

}  // namespace interaction

namespace transformation {

class TransformationManager;

}  // namespace transformation

class BitAllocator;
class DisplayFactory;
class DisplayGroup;
class FrameManagerIface;
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
    /// Returns the SceneViewer used for the main RenderPanel.
    virtual SceneViewer* getSceneViewer() const = 0;

    /// Returns the Ogre SceneManager used for rendering.
    virtual Ogre::SceneManager* getSceneManager() const = 0;

    /// Return the window manager, if any.
    virtual WindowManagerInterface* getWindowManager() const = 0;

    /// Return a pointer to the SelectionManager.
    virtual std::shared_ptr<aviz::common::interaction::SelectionManagerIface> getSelectionManager() const = 0;

    /// Return a pointer to the HandlerManager.
    virtual std::shared_ptr<aviz::common::interaction::HandlerManagerIface> getHandlerManager() const = 0;

    /// Return a pointer to the ViewPicker.
    virtual std::shared_ptr<aviz::common::interaction::ViewPickerIface> getViewPicker() const = 0;

    /// Return the FrameManager instance.
    virtual FrameManagerIface* getFrameManager() const = 0;

    /// Return the fixed frame name.
    virtual QString getFixedFrame() const = 0;

    /// Return the current value of the frame count.
    /**
     * The frame count is just a number which increments each time a
     * frame is rendered.  This lets clients check if a new frame has
     * been rendered since the last time they did something.
     */
    virtual uint64_t getFrameCount() const = 0;

    /// Return a factory for creating Display subclasses based on a class id string.
    virtual DisplayFactory* getDisplayFactory() const = 0;

    /// Handle a single key event for a given RenderPanel.
    virtual void handleChar(QKeyEvent* event, RenderPanel* panel) = 0;

    /// Handle a mouse event.
    virtual void handleMouseEvent(const ViewportMouseEvent& event) = 0;

    /// Return the ToolManager.
    virtual ToolManager* getToolManager() const = 0;

    /// Return the ViewManager.
    virtual ViewManager* getViewManager() const = 0;

    virtual transformation::TransformationManager* getTransformationManager() = 0;

    /// Return the root DisplayGroup.
    virtual DisplayGroup* getRootDisplayGroup() const = 0;

    /// Get the default visibility bit.
    virtual uint32_t getDefaultVisibilityBit() const = 0;

    /// Get the visibility bits.
    virtual BitAllocator* visibilityBits() = 0;

    /// Set the message displayed in the status bar.
    virtual void setStatus(const QString& message) = 0;

    virtual QString getHelpPath() const = 0;

    /// Queue a render.
    /**
     * Multiple calls before a render happens will only cause a single render.
     *
     * \note This function can be called from any thread.
     */
public Q_SLOTS:
    virtual void queueRender() = 0;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__DISPLAY_CONTEXT_HPP_
