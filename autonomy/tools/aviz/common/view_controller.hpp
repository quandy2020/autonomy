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

#ifndef AVIZ_COMMON__VIEW_CONTROLLER_HPP_
#define AVIZ_COMMON__VIEW_CONTROLLER_HPP_

#include <QCursor>  // NOLINT: cpplint is unable to handle the include order here
#include <QMap>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here
#include <QVector3D>  // NOLINT: cpplint is unable to handle the include order here
#include <string>

#include "autonomy/tools/aviz/common/properties/property.hpp"

class QKeyEvent;

namespace aviz {
namespace common {

class DisplayContext;
class SceneViewer;
class ViewportMouseEvent;

namespace properties {
class EnumProperty;
class FloatProperty;
class BoolProperty;
}  // namespace properties

struct FocalPointStatus {
    FocalPointStatus() {
        exists_ = false;
        value_ = QVector3D(0, 0, 0);
    }

    FocalPointStatus(bool has_focal_point, QVector3D focal_point) {
        exists_ = has_focal_point;
        value_ = focal_point;
    }

    bool exists_;
    QVector3D value_;
};

/// Base class for view controllers
/**
 * ViewControllers control the camera/viewpoint in the 3D scene.
 * Examples include orbit view, FPS view, top-down view, etc.
 */
class ViewController : public properties::Property
{
    Q_OBJECT

public:
    ViewController();
    ~ViewController() override;

    /// Do all setup that can't be done in the constructor.
    /**
     * Calls onInitialize() just before returning.
     */
    void initialize(DisplayContext* context);

    /// Return a formatted class_id.
    static QString formatClassId(const QString& class_id);

    /// Overridden to give a different background color and bold font if this
    /// view is active.
    QVariant getViewData(int column, int role) const override;  // from Property

    /// Overridden to make this draggable if it is not active.
    Qt::ItemFlags getViewFlags(int column) const override;  // from Property

    /// Called by SceneViewer when this view controller is about to be used.
    /**
     * There is no deactivate() because ViewControllers leaving "current" are
     * destroyed. Put any cleanup in the destructor.
     */
    virtual void activate();

    /// Called at 30Hz by ViewManager::update() while this view is active.
    /**
     * Override with code that needs to run repeatedly.
     */
    virtual void update(float dt, float ros_dt);

    /// Called when mouse events are fired.
    virtual void handleMouseEvent(ViewportMouseEvent& evt);

    /// Called when keyboard events are passed to them.
    /**
     * The default implementation here handles the "F" (focus on object) and "Z"
     * (zero - reset) keys.
     */
    virtual void handleKeyEvent(QKeyEvent* event, SceneViewer* viewer);

    /// Convenience function which calls lookAt(QVector3D).
    void lookAt(float x, float y, float z);

    /// Aim the camera at the given point in space.
    /**
     * The point in space is relative to the fixed frame.
     */
    virtual void lookAt(const QVector3D& point) = 0;

    /// Reset the view controller to some sane initial state.
    /**
     * For example, by looking at (0, 0, 0) from a few meters away.
     */
    virtual void reset() = 0;

    /// Setup this view controller using values from the source view controller.
    /**
     * The idea is to have as similar of a view as possible when switching view
     * controllers.
     *
     * This base class implementation does nothing.
     */
    virtual void mimic(ViewController* source_view);

    /// Called by ViewManager when this ViewController is being made current.
    /**
     * This gives ViewController subclasses an opportunity to implement a smooth
     * transition from a previous viewpoint to the new viewpoint.
     *
     * This base class implementation does nothing.
     *
     * \param previous_view is the previous "current" view, and will not be
     * nullptr.
     */
    virtual void transitionFrom(ViewController* previous_view);

    /// Subclasses should call this whenever a config change is made.
    /**
     * A config change is considered any change which would change the results
     * of toString()
     */
    void emitConfigChanged();

    /// Return the class identifier which was used to create this instance.
    /**
     * The default version returns whatever was set with setClassId().
     */
    virtual QString getClassId() const;

    /// Set the class identifier used to create this instance.
    /**
     * Typically this will be set by the factory object which created it.
     */
    virtual void setClassId(const QString& class_id);

    /// Load settings from a Config object.
    void load(const Config& config) override;

    /// Save settings to a Config object.
    void save(Config config) const override;

    /// Return true if this view controller is active.
    bool isActive() const;

    /// Return a mouse cursor representing the current state.
    virtual QCursor getCursor();

    virtual FocalPointStatus getFocalPointStatus();

Q_SIGNALS:
    void configChanged();

protected:
    /// Do subclass-specific initialization.
    /**
     * Called by ViewController::initialize after context_ is set.
     * Default implementation does nothing.
     */
    virtual void onInitialize();

    /// Called by activate().
    /**
     * Override to implement view-specific activation.
     *
     * This base class implementation does nothing.
     */
    virtual void onActivate();

    // choose a cursor from the standard set
    enum CursorType {
        Default,
        Rotate2D,
        Rotate3D,
        MoveXY,
        MoveZ,
        Zoom,
        Crosshair
    };

    /// Set the cursor type.
    void setCursor(CursorType cursor_type);

    // Set the cursor using a custom QCursor.
    void setCursor(QCursor cursor);

    DisplayContext* context_;

    bool is_active_;

    // this cursor will be displayed when the mouse is within the
    // window controlled by this view controller
    // use SetCursor to modify.
    QCursor cursor_;

    /// Set the status on the main render frame.
    void setStatus(const QString& message);

private:
    QString class_id_;

    // Default cursors for the most common actions
    QMap<CursorType, QCursor> standard_cursors_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__VIEW_CONTROLLER_HPP_
