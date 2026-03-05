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

#ifndef AVIZ_COMMON__TOOL_HPP_
#define AVIZ_COMMON__TOOL_HPP_

#include <QCursor>  // NOLINT: cpplint is unable to handle the include order here
#include <QIcon>    // NOLINT: cpplint is unable to handle the include order here
#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "autonomy/tools/aviz/common/config.hpp"

class QKeyEvent;
class SceneViewer;

namespace aviz {
namespace common {

namespace properties {
class Property;
}

class DisplayContext;
class ViewportMouseEvent;

/// Base class for interactive tools
/**
 * Tools are interactive elements that respond to mouse and keyboard events.
 * Examples include selection tools, measurement tools, etc.
 */
class Tool : public QObject {
  Q_OBJECT

 public:
  /**
   * Pluginlib only instantiates classes via default constructors.
   * Subclasses of Tool should set the shortcut_key_ field in their
   * constructors.
   *
   * Properties to appear in the Tool Properties panel are typically
   * created in the constructor, as children of the property from
   * getPropertyContainer(), which is set up in this Tool
   * constructor.
   */
  Tool();

  ~Tool() override;

  /// Initialize the tool.
  /**
   * Sets the DisplayContext and calls onInitialize().
   */
  void initialize(DisplayContext* context);

  /// Return the container for properties of this Tool.
  virtual aviz::common::properties::Property* getPropertyContainer() const;

  /// Get the shortcut key for the tool.
  char getShortcutKey() const;

  /// Return true if the tool needs to access all keys or false if not.
  bool accessAllKeys() const;

  /// Override to get called when the tool is activated.
  virtual void activate() = 0;

  /// Override to get called when the tool is deactivated.
  virtual void deactivate() = 0;

  /// Called periodically, typically at 30Hz.
  virtual void update(float wall_dt, float ros_dt);

  enum { Render = 1, Finished = 2 };

  /// Process a mouse event.
  /**
   * This is the central function of all the tools, as it defines how the
   * mouse is used.
   */
  virtual int processMouseEvent(ViewportMouseEvent& event);

  /// Process a key event.
  /**
   * Override if your tool should handle any other keypresses than the tool
   * shortcuts, which are handled separately.
   */
  virtual int processKeyEvent(QKeyEvent* event, SceneViewer* viewer);

  /// Get the name of the tool.
  QString getName() const;

  /// Set the name of the tool.
  void setName(const QString& name);

  /// Get the description.
  QString getDescription() const;

  /// Set the description of the tool.
  void setDescription(const QString& description);

  /// Return the class identifier which was used to create this instance.
  virtual QString getClassId() const;

  /// Set the class identifier used to create this instance.
  virtual void setClassId(const QString& class_id);

  /// Load properties from the given Config.
  virtual void load(const Config& config);

  /// Save this entire tool into the given Config node.
  virtual void save(Config config) const;

  /// Set the toolbar icon for this tool (will also set its cursor).
  void setIcon(const QIcon& icon);

  /// Get the icon of this tool.
  const QIcon& getIcon();

  /// Set the cursor for this tool.
  void setCursor(const QCursor& cursor);

  /// Get current cursor of this tool.
  const QCursor& getCursor();

  /// Set the status message.
  void setStatus(const QString& message);

 Q_SIGNALS:
  /// Emitted when closed.
  void close();
  /// Emitted when name property has been changed.
  void nameChanged(const QString& name);

 protected:
  /// Override onInitialize to do any setup needed after the DisplayContext has been set.
  /**
   * This is called by Tool::initialize().
   */
  virtual void onInitialize() {}

  DisplayContext* context_;

  char shortcut_key_;
  bool access_all_keys_;

  QIcon icon_;

  QCursor cursor_;

 private:
  QString class_id_;
  aviz::common::properties::Property* property_container_;
  QString name_;
  QString description_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__TOOL_HPP_
