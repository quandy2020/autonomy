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

#ifndef AVIZ_COMMON__DISPLAY_HPP_
#define AVIZ_COMMON__DISPLAY_HPP_

#include <string>

#include <QIcon>     // NOLINT: cpplint is unable to handle the include order here
#include <QMap>      // NOLINT: cpplint is unable to handle the include order here
#include <QObject>   // NOLINT: cpplint is unable to handle the include order here
#include <QSet>      // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autonomy/tools/aviz/common/config.hpp"

class QDockWidget;
class QWidget;
class SceneViewer;

namespace aviz {
namespace common {

class DisplayContext;
class PanelDockWidget;

/// Base class for all display types.
/**
 * Each Display represents a visualization element (robot model, point cloud, etc.)
 * that can be enabled/disabled and updated over time.
 *
 * Similar to rviz_common::aviz::common::Display, but adapted for aviz system.
 */
class Display : public QObject
{
    Q_OBJECT

public:
    Display();
    virtual ~Display();

    /// Main initialization, called after constructor, before load() or setEnabled().
    virtual void initialize(DisplayContext* context);

    /// Return data appropriate for the given column (0 or 1) and role for this Display.
    QVariant getViewData(int column, int role) const;

    /// Return item flags appropriate for the given column (0 or 1) for this Display.
    Qt::ItemFlags getViewFlags(int column) const;

    /// Return the class identifier which was used to create this instance.
    /**
     * This version just returns whatever was set with setClassId().
     */
    virtual QString getClassId() const;

    /// Set the class identifier used to create this instance.
    /**
     * Typically this will be set by the factory object which created it.
     */
    virtual void setClassId(const QString& class_id);

    /// Load the settings for this display from the given Config node, which must be a map.
    /**
     * load() is called after initialize().
     */
    virtual void load(const Config& config);

    /// Write this display to the given Config node.
    virtual void save(Config config) const;

    /// Set the topic to listen to for this display.
    /**
     * By default, do nothing.
     * Subclasses should override this method if they subscribe to a single
     * topic.
     *
     * \param topic The published topic to be visualized.
     * \param datatype The datatype of the topic.
     */
    virtual void setTopic(const QString& topic, const QString& datatype);

    /// Return true if this Display is enabled, false if not.
    bool isEnabled() const;

    /// Set the fixed frame in this display.
    void setFixedFrame(const QString& fixed_frame);

    /// Called periodically by the visualization manager.
    /**
     * \param wall_dt Wall-clock time, in seconds, since the last time the update list was run through.
     * \param ros_dt ROS time, in seconds, since the last time the update list was run through.
     */
    virtual void update(float wall_dt, float ros_dt);

    /// Called to tell the display to clear its state.
    virtual void reset();

    /// Show status level and text.
    /**
     * This is thread-safe.
     *
     * \param level Status level (0=Ok, 1=Warn, 2=Error).
     * \param name The name of the status entry.
     * \param text Description of the status.
     */
    virtual void setStatus(int level, const QString& name, const QString& text);

    /// Show status level and text, using a std::string.
    /**
     * Convenience function which converts std::string to QString and calls
     * setStatus().
     * This is thread-safe.
     */
    void setStatusStd(int level, const std::string& name, const std::string& text);

    /// Convenience: Show and log missing transform
    /**
     * Convenience function which shows an error status for a missing transform.
     * @param frame frame with missing transform to fixed_frame
     * @param additional_info additional info included in the error_message
     */
    void setMissingTransformToFixedFrame(const std::string& frame, const std::string& additional_info = "");

    /// Convenience: Set Transform ok
    void setTransformOk();

    /// Delete the status entry with the given name.
    /**
     * This is thread-safe.
     */
    virtual void deleteStatus(const QString& name);

    /// Delete the status entry with the given std::string name.
    /**
     * This is thread-safe.
     */
    void deleteStatusStd(const std::string& name);

    /// Set the visibility bits.
    /**
     * Default is all bits ON.
     */
    void setVisibilityBits(uint32_t bits);

    /// Unset the visibility bits.
    void unsetVisibilityBits(uint32_t bits);

    /// Get the visibility bits.
    uint32_t getVisibilityBits();

    /// Return the SceneViewer used for rendering.
    SceneViewer* getSceneViewer() const;

    /// Associate the given @a widget with this Display.
    /**
     * Each Display can have one QWidget which is shown when the Display
     * is enabled and hidden when the Display is disabled.
     *
     * Since there is only one slot for such a widget, this dis-associates any
     * previously associated widget.
     *
     * Call this with nullptr to disassociate the current associated widget.
     */
    void setAssociatedWidget(QWidget* widget);

    /// Return the current associated widget, or nullptr if there is none.
    /**
     * \see setAssociatedWidget()
     */
    QWidget* getAssociatedWidget() const;

    /// Return the panel containing the associated widget, or nullptr if there is none.
    /**
     * \see setAssociatedWidget()
     */
    PanelDockWidget* getAssociatedWidgetPanel();

    /// Set the display name.
    void setName(const QString& name);

    /// Get the display name.
    QString getName() const;

    /// Set the Display's icon.
    void setIcon(const QIcon& icon);

    /// Get the Display's icon.
    QIcon getIcon() const;

    /// Return the Ogre::SceneNode holding all 3D scene elements shown by this Display.
    Ogre::SceneNode* getSceneNode() const;

Q_SIGNALS:
    void timeSignal(aviz::common::Display* display, uint64_t time_ns);

public Q_SLOTS:
    /// Enable or disable this Display.
    /**
     * SetEnabled is called after initialize() and at the end of load(),
     * if the Display settings are being loaded from a file.
     */
    void setEnabled(bool enabled);

    /// Convenience function which calls context_->queueRender().
    void queueRender();

protected:
    /// Override this function to do subclass-specific initialization.
    /**
     * This is called after context_ and scene_viewer_ are set, and before
     * load() or setEnabled().
     *
     * setName() may or may not have been called before this.
     */
    virtual void onInitialize();

    /// Derived classes override this to do the actual work of enabling themselves.
    virtual void onEnable();

    /// Derived classes override this to do the actual work of disabling themselves.
    virtual void onDisable();

    /// Delete all status children.
    /**
     * This is thread-safe.
     *
     * This removes all status children and updates the top-level status.
     */
    virtual void clearStatuses();

    /// Called by setFixedFrame().
    /**
     * Override to respond to changes to fixed_frame_.
     */
    virtual void fixedFrameChanged();

    /// Returns true if the display has been initialized.
    bool initialized() const;

    /// This DisplayContext pointer is the main connection a Display has into the rest of aviz.
    /**
     * This is how the FrameManager is accessed, the SelectionManager, etc.
     * When a Display subclass wants to signal that a new render should be done
     * right away, call context_->queueRender().
     *
     * This is set after the constructor and before onInitialize() is called.
     */
    DisplayContext* context_;

    /// A convenience variable equal to context_->getSceneViewer().
    /**
     * This is set after the constructor and before onInitialize() is called.
     */
    SceneViewer* scene_viewer_;

    /// A convenience variable equal to context_->getSceneManager().
    /**
     * This is set after the constructor and before onInitialize() is called.
     */
    Ogre::SceneManager* scene_manager_;

    /// The Ogre::SceneNode to hold all 3D scene elements shown by this Display.
    Ogre::SceneNode* scene_node_;

    /// A convenience variable equal to context_->getFixedFrame().
    /**
     * This is set after the constructor and before onInitialize() is
     * called.
     * Every time it is updated (via setFixedFrame()), fixedFrameChanged() is called.
     */
    QString fixed_frame_;

private Q_SLOTS:
    void setStatusInternal(int level, const QString& name, const QString& text);

    void deleteStatusInternal(const QString& name);

    void clearStatusesInternal();

    void associatedPanelVisibilityChange(bool visible);

    void disable();

    void onEnableChanged();

private:
    QString class_id_;
    QString name_;
    bool initialized_;
    bool enabled_;
    uint32_t visibility_bits_;
    QIcon icon_;
    QWidget* associated_widget_;
    PanelDockWidget* associated_widget_panel_;

    // Simple status storage (can be extended later with StatusList)
    struct StatusEntry {
        int level;
        QString name;
        QString text;
    };
    QMap<QString, StatusEntry> status_map_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__DISPLAY_HPP_
