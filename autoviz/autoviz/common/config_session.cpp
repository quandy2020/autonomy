/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/config_session.hpp"

#include <glog/logging.h>

namespace autoviz {
namespace common {
namespace {

void ReadDisplayFromConfig(const Config& node, DisplayConfig* entry) {
  if (entry == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Type", &value)) {
    entry->type = value.toStdString();
  }
  if (node.mapGetString("Name", &value)) {
    entry->name = value.toStdString();
  } else {
    entry->name = entry->type;
  }
  if (node.mapGetString("Channel", &value)) {
    entry->channel = value.toStdString();
  }
  bool enabled = true;
  if (node.mapGetBool("Enabled", &enabled)) {
    entry->enabled = enabled;
  }
  entry->properties.clear();
  Config props = node.mapGetChild("Properties");
  if (props.isValid()) {
    for (Config::MapIterator it = props.mapIterator(); it.isValid();
         it.advance()) {
      Config child = it.currentChild();
      if (child.getType() == Config::Value) {
        entry->properties[it.currentKey().toStdString()] =
            child.getValue().toString().toStdString();
      }
    }
  }
  entry->children.clear();
  Config children = node.mapGetChild("Children");
  if (children.isValid()) {
    for (int i = 0; i < children.listLength(); ++i) {
      DisplayConfig child;
      ReadDisplayFromConfig(children.listChildAt(i), &child);
      entry->children.push_back(std::move(child));
    }
  }
}

void WriteDisplayToConfig(const DisplayConfig& display, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Type", QString::fromStdString(display.type));
  node->mapSetValue("Name", QString::fromStdString(display.name()));
  node->mapSetValue("Channel", QString::fromStdString(display.channel));
  node->mapSetValue("Enabled", display.enabled);
  if (!display.properties.empty()) {
    Config props = node->mapMakeChild("Properties");
    for (const auto& prop : display.properties) {
      props.mapSetValue(QString::fromStdString(prop.first),
                        QString::fromStdString(prop.second));
    }
  }
  if (!display.children.empty()) {
    Config children = node->mapMakeChild("Children");
    for (const auto& child : display.children) {
      Config child_node = children.listAppendNew();
      WriteDisplayToConfig(child, &child_node);
    }
  }
}

void ReadViewFromConfig(const Config& node, SavedViewConfig* view) {
  if (view == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Name", &value)) {
    view->name = value.toStdString();
  }
  if (node.mapGetString("Type", &value)) {
    view->type = value.toStdString();
  }
  node.mapGetFloat("NearClipDistance", &view->near_clip_distance);
  node.mapGetBool("InvertZAxis", &view->invert_z_axis);
  node.mapGetFloat("Yaw", &view->yaw);
  node.mapGetFloat("Pitch", &view->pitch);
  node.mapGetFloat("Distance", &view->distance);
  node.mapGetFloat("FocalShapeSize", &view->focal_shape_size);
  node.mapGetBool("FocalShapeFixedSize", &view->focal_shape_fixed_size);
  node.mapGetFloat("FpsYaw", &view->fps_yaw);
  node.mapGetFloat("FpsPitch", &view->fps_pitch);
  if (node.mapGetString("TargetFrame", &value)) {
    view->target_frame = value.toStdString();
  }
  Config target = node.mapGetChild("Target");
  target.mapGetFloat("X", &view->target_x);
  target.mapGetFloat("Y", &view->target_y);
  target.mapGetFloat("Z", &view->target_z);
  Config fps = node.mapGetChild("FpsPosition");
  fps.mapGetFloat("X", &view->fps_position_x);
  fps.mapGetFloat("Y", &view->fps_position_y);
  fps.mapGetFloat("Z", &view->fps_position_z);
}

void WriteViewToConfig(const SavedViewConfig& view, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Name", QString::fromStdString(view.name()));
  node->mapSetValue("Type", QString::fromStdString(view.type));
  node->mapSetValue("NearClipDistance", view.near_clip_distance);
  node->mapSetValue("InvertZAxis", view.invert_z_axis);
  node->mapSetValue("Yaw", view.yaw);
  node->mapSetValue("Pitch", view.pitch);
  node->mapSetValue("Distance", view.distance);
  node->mapSetValue("FocalShapeSize", view.focal_shape_size);
  node->mapSetValue("FocalShapeFixedSize", view.focal_shape_fixed_size);
  node->mapSetValue("FpsYaw", view.fps_yaw);
  node->mapSetValue("FpsPitch", view.fps_pitch);
  if (!view.target_frame.empty()) {
    node->mapSetValue("TargetFrame",
                      QString::fromStdString(view.target_frame));
  }
  Config target = node->mapMakeChild("Target");
  target.mapSetValue("X", view.target_x);
  target.mapSetValue("Y", view.target_y);
  target.mapSetValue("Z", view.target_z);
  Config fps = node->mapMakeChild("FpsPosition");
  fps.mapSetValue("X", view.fps_position_x);
  fps.mapSetValue("Y", view.fps_position_y);
  fps.mapSetValue("Z", view.fps_position_z);
}

std::string MapRvizTransformerClass(const std::string& rviz_class) {
  if (rviz_class.find("Identity") != std::string::npos) {
    return "autoviz/Identity";
  }
  if (rviz_class.find("TF") != std::string::npos ||
      rviz_class.find("Tf") != std::string::npos) {
    return "autoviz/AutolinkTf";
  }
  return "autoviz/AutolinkTf";
}

bool SessionConfigFromNativeConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr) {
    return false;
  }
  Config global = root.mapGetChild("Global");
  if (global.isValid()) {
    QString value;
    if (global.mapGetString("FixedFrame", &value)) {
      config->fixed_frame = value.toStdString();
    }
    global.mapGetBool("ShowGrid", &config->show_grid);
    global.mapGetInt("FrameRate", &config->frame_rate);
    if (global.mapGetString("BackgroundColor", &value)) {
      config->background_color = value.toStdString();
    }
    if (global.mapGetString("ViewController", &value)) {
      config->view_controller = value.toStdString();
    }
    if (global.mapGetString("RenderBackend", &value)) {
      config->render_backend = value.toStdString();
    }
    if (global.mapGetString("ActiveTool", &value)) {
      config->active_tool = value.toStdString();
    }
    if (global.mapGetString("Transformer", &value)) {
      config->transformer_id = value.toStdString();
    }
  }

  Config window = root.mapGetChild("Window");
  if (window.isValid()) {
    QString value;
    if (window.mapGetString("State", &value)) {
      config->window_state_b64 = value.toStdString();
    }
    if (window.mapGetString("Geometry", &value)) {
      config->window_geometry_b64 = value.toStdString();
    }
    window.mapGetBool("HideLeftDock", &config->hide_left_dock);
    window.mapGetBool("HideRightDock", &config->hide_right_dock);
    window.mapGetInt("X", &config->window_x);
    window.mapGetInt("Y", &config->window_y);
    window.mapGetInt("Width", &config->window_width);
    window.mapGetInt("Height", &config->window_height);
    config->visible_panels.clear();
    Config visible = window.mapGetChild("VisiblePanels");
    for (int i = 0; i < visible.listLength(); ++i) {
      config->visible_panels.push_back(
          visible.listChildAt(i).getValue().toString().toStdString());
    }
    config->panel_layouts.clear();
    Config panels = window.mapGetChild("Panels");
    for (int i = 0; i < panels.listLength(); ++i) {
      PanelLayoutConfig panel;
      Config node = panels.listChildAt(i);
      QString name;
      if (node.mapGetString("Name", &name)) {
        panel.object_name = name.toStdString();
      }
      node.mapGetBool("Collapsed", &panel.collapsed);
      config->panel_layouts.push_back(std::move(panel));
    }
  }

  config->displays.clear();
  Config displays = root.mapGetChild("Displays");
  for (int i = 0; i < displays.listLength(); ++i) {
    DisplayConfig entry;
    ReadDisplayFromConfig(displays.listChildAt(i), &entry);
    config->displays.push_back(std::move(entry));
  }

  Config current = root.mapGetChild("CurrentView");
  if (current.isValid()) {
    config->has_current_view = true;
    config->current_view.name = "Current";
    ReadViewFromConfig(current, &config->current_view);
    config->view_controller = config->current_view.type;
  }

  config->views.clear();
  Config views = root.mapGetChild("Views");
  for (int i = 0; i < views.listLength(); ++i) {
    SavedViewConfig view;
    ReadViewFromConfig(views.listChildAt(i), &view);
    config->views.push_back(std::move(view));
  }

  config->toolbar_tools.clear();
  Config toolbar = root.mapGetChild("Toolbar");
  for (int i = 0; i < toolbar.listLength(); ++i) {
    config->toolbar_tools.push_back(
        toolbar.listChildAt(i).getValue().toString().toStdString());
  }

  config->tools.clear();
  Config tools = root.mapGetChild("Tools");
  for (int i = 0; i < tools.listLength(); ++i) {
    ToolConfig tool;
    Config node = tools.listChildAt(i);
    QString id;
    if (node.mapGetString("Id", &id)) {
      tool.id = id.toStdString();
    }
    Config props = node.mapGetChild("Properties");
    for (Config::MapIterator it = props.mapIterator(); it.isValid();
         it.advance()) {
      tool.properties[it.currentKey().toStdString()] =
          it.currentChild().getValue().toString().toStdString();
    }
    config->tools.push_back(std::move(tool));
  }
  return true;
}

bool SessionConfigFromRvizConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr) {
    return false;
  }
  Config manager = root.mapGetChild("Visualization Manager");
  if (!manager.isValid()) {
    return false;
  }

  Config global = manager.mapGetChild("Global Options");
  if (global.isValid()) {
    QString value;
    if (global.mapGetString("Fixed Frame", &value)) {
      config->fixed_frame = value.toStdString();
      if (!config->fixed_frame.empty() && config->fixed_frame.front() == '/') {
        config->fixed_frame.erase(config->fixed_frame.begin());
      }
    }
    if (global.mapGetString("Background Color", &value)) {
      config->background_color = value.toStdString();
    }
    global.mapGetInt("Frame Rate", &config->frame_rate);
  }

  Config transformation = manager.mapGetChild("Transformation");
  if (transformation.isValid()) {
    Config current = transformation.mapGetChild("Current");
    QString class_id;
    if (current.mapGetString("Class", &class_id)) {
      config->transformer_id = MapRvizTransformerClass(class_id.toStdString());
    }
  }

  Config window = root.mapGetChild("Window Geometry");
  if (window.isValid()) {
    window.mapGetBool("Hide Left Dock", &config->hide_left_dock);
    window.mapGetBool("Hide Right Dock", &config->hide_right_dock);
    QString state;
    if (window.mapGetString("QMainWindow State", &state)) {
      config->window_state_b64 = state.toStdString();
    }
    window.mapGetInt("Width", &config->window_width);
    window.mapGetInt("Height", &config->window_height);
    window.mapGetInt("X", &config->window_x);
    window.mapGetInt("Y", &config->window_y);
  }
  return true;
}

}  // namespace

void SessionConfigToConfig(const SessionConfig& session, Config* root) {
  if (root == nullptr) {
    return;
  }
  root->setType(Config::Map);

  Config global = root->mapMakeChild("Global");
  global.mapSetValue("FixedFrame", QString::fromStdString(session.fixed_frame));
  global.mapSetValue("ShowGrid", session.show_grid);
  global.mapSetValue("FrameRate", session.frame_rate);
  global.mapSetValue("BackgroundColor",
                     QString::fromStdString(session.background_color));
  global.mapSetValue("ViewController",
                     QString::fromStdString(session.view_controller));
  global.mapSetValue("RenderBackend",
                     QString::fromStdString(session.render_backend));
  global.mapSetValue("ActiveTool",
                     QString::fromStdString(session.active_tool));
  global.mapSetValue("Transformer",
                     QString::fromStdString(session.transformer_id));

  Config window = root->mapMakeChild("Window");
  if (!session.window_state_b64.empty()) {
    window.mapSetValue("State", QString::fromStdString(session.window_state_b64));
  }
  if (!session.window_geometry_b64.empty()) {
    window.mapSetValue("Geometry",
                       QString::fromStdString(session.window_geometry_b64));
  }
  window.mapSetValue("HideLeftDock", session.hide_left_dock);
  window.mapSetValue("HideRightDock", session.hide_right_dock);
  if (session.window_x >= 0) {
    window.mapSetValue("X", session.window_x);
  }
  if (session.window_y >= 0) {
    window.mapSetValue("Y", session.window_y);
  }
  if (session.window_width > 0) {
    window.mapSetValue("Width", session.window_width);
  }
  if (session.window_height > 0) {
    window.mapSetValue("Height", session.window_height);
  }
  if (!session.visible_panels.empty()) {
    Config visible = window.mapMakeChild("VisiblePanels");
    for (const auto& panel : session.visible_panels) {
      visible.listAppendNew().setValue(QString::fromStdString(panel));
    }
  }
  if (!session.panel_layouts.empty()) {
    Config panels = window.mapMakeChild("Panels");
    for (const auto& panel : session.panel_layouts) {
      Config node = panels.listAppendNew();
      node.mapSetValue("Name", QString::fromStdString(panel.object_name));
      if (panel.collapsed) {
        node.mapSetValue("Collapsed", true);
      }
    }
  }

  Config displays = root->mapMakeChild("Displays");
  for (const auto& display : session.displays) {
    Config node = displays.listAppendNew();
    WriteDisplayToConfig(display, &node);
  }

  if (session.has_current_view) {
    Config current_view = root->mapMakeChild("CurrentView");
    WriteViewToConfig(session.current_view, &current_view);
  }

  Config views = root->mapMakeChild("Views");
  for (const auto& view : session.views) {
    Config node = views.listAppendNew();
    WriteViewToConfig(view, &node);
  }

  Config toolbar = root->mapMakeChild("Toolbar");
  for (const auto& tool_id : session.toolbar_tools) {
    toolbar.listAppendNew().setValue(QString::fromStdString(tool_id));
  }

  Config tools = root->mapMakeChild("Tools");
  for (const auto& tool : session.tools) {
    Config node = tools.listAppendNew();
    node.mapSetValue("Id", QString::fromStdString(tool.id));
    if (!tool.properties.empty()) {
      Config props = node.mapMakeChild("Properties");
      for (const auto& prop : tool.properties) {
        props.mapSetValue(QString::fromStdString(prop.first),
                          QString::fromStdString(prop.second));
      }
    }
  }
}

bool SessionConfigFromConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr || !root.isValid()) {
    return false;
  }
  if (root.mapGetChild("Visualization Manager").isValid()) {
    *config = SessionConfig{};
    if (!SessionConfigFromRvizConfig(root, config)) {
      return false;
    }
    LOG(INFO) << "Imported RViz config via Config tree";
    return true;
  }
  return SessionConfigFromNativeConfig(root, config);
}

}  // namespace common
}  // namespace autoviz
