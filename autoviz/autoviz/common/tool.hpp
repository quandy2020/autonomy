/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <QCursor>
#include <QImage>
#include <QString>
#include <QVector3D>

#include "autolink/node/node.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/common/selection_manager.hpp"

class QMouseEvent;
class QWheelEvent;

namespace autoviz {
namespace display {
class InteractiveMarkerRegistry;
}
namespace rendering {
class SceneOverlay;
class ViewController;
}

namespace common {

struct ToolContext {
  rendering::ViewController* view_controller = nullptr;
  rendering::SceneOverlay* scene_overlay = nullptr;
  int viewport_width = 1;
  int viewport_height = 1;
  /** True when hardware GPU was detected; enables depth-buffer picking. */
  bool gpu_picking_enabled = false;
  std::function<bool(int pixel_x, int pixel_y, QVector3D* world)>
      gpu_depth_pick;
  std::function<common::PickHandle(int pixel_x, int pixel_y)> gpu_pick_id_read;
  common::PickRegistry* pick_registry = nullptr;
  common::HandlerManager* handler_manager = nullptr;
  std::shared_ptr<::autolink::Node> autolink_node;
  std::string fixed_frame = "map";
  DisplayContext* display_context = nullptr;
  SelectionManager* selection_manager = nullptr;
  autoviz::display::InteractiveMarkerRegistry* interactive_markers = nullptr;
  std::function<void()> request_redraw;
  /** Refresh display_context->ogre_scene_host before Ogre tool drawing. */
  std::function<void()> sync_ogre_host;
  std::function<void(const QString&)> set_status;
  std::function<void(const std::vector<SelectionEntry>&)> selections_changed;
  /** RViz Tool::Finished — switch back to Move Camera after one-shot tools. */
  std::function<void()> revert_to_default_tool;
};

/** RViz-style interactive tool (Move Camera, Focus, Measure, …). */
class Tool {
 public:
  virtual ~Tool() = default;

  virtual std::string id() const = 0;
  virtual QString label() const = 0;

  virtual void activate(ToolContext* context) { context_ = context; }
  /** Refresh context pointers without resetting tool interaction state. */
  virtual void updateContext(ToolContext* context) { context_ = context; }
  virtual void deactivate() { context_ = nullptr; }

  const QCursor& cursor() const { return cursor_; }
  void setCursor(const QCursor& cursor) { cursor_ = cursor; }

  /** @return true if the event was consumed (skip default camera drag). */
  virtual bool mousePressEvent(QMouseEvent* event);
  virtual bool mouseMoveEvent(QMouseEvent* event);
  virtual bool mouseReleaseEvent(QMouseEvent* event);
  virtual bool wheelEvent(QWheelEvent* event);

  virtual void onDraw(rendering::SceneOverlay& /*scene*/) {}
  virtual QString statusText() const { return {}; }
  /** RViz Tool::getShortcutKey — '\0' if none. */
  virtual char shortcutKey() const { return '\0'; }

  virtual std::vector<DisplayPropertySpec> propertySpecs() const { return {}; }
  void setProperties(const DisplayPropertyMap& properties);
  const DisplayPropertyMap& properties() const { return properties_; }
  std::string propertyValue(const std::string& key,
                            const std::string& default_value) const;
  void setPropertyValue(const std::string& key, const std::string& value);

 protected:
  ToolContext* context() const { return context_; }

 private:
  ToolContext* context_ = nullptr;
  DisplayPropertyMap properties_;
  QCursor cursor_ = Qt::ArrowCursor;
};

}  // namespace common
}  // namespace autoviz
