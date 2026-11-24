/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file session_config_test.cpp
 * @brief Round-trip regression for .autoviz session serialization.
 *
 * Covers the fields that panels and the mosaic persist: globals, nested
 * display groups, display properties, saved views and layout blobs.
 */

#include <cmath>
#include <cstdio>
#include <string>

#include <QDir>
#include <QTemporaryDir>

#include "autoviz/common/session_config.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

bool Near(float a, float b) { return std::fabs(a - b) < 1e-5f; }

autoviz::common::SessionConfig MakeSession() {
  using autoviz::common::DisplayConfig;
  using autoviz::common::SavedViewConfig;
  using autoviz::common::SessionConfig;
  using autoviz::common::ToolConfig;

  SessionConfig session;
  session.fixed_frame = "odom";
  session.frame_rate = 45;
  session.background_color = "12;34;56";
  session.view_controller = "XYOrbit";
  session.render_backend = "Ogre";
  session.active_tool = "Measure";
  session.toolbar_tools = {"Interact", "Select"};
  session.main_panel_state_b64 = "bW9zYWljOnYxOnRlc3Q=";
  session.window_geometry_b64 = "Z2VvbWV0cnk=";
  session.visible_panels = {"ViewportDock", "PlotDock"};
  session.hide_left_dock = true;

  DisplayConfig cloud;
  cloud.type = "PointCloud2";
  cloud.name = "Cloud";
  cloud.channel = "/points";
  cloud.enabled = false;
  cloud.properties["style"] = "Flat Squares";
  cloud.properties["color"] = "0;200;255";

  DisplayConfig child;
  child.type = "Grid";
  child.name = "Grid";
  child.enabled = true;
  child.properties["plane"] = "XY";

  DisplayConfig group;
  group.type = "Group";
  group.name = "Sensors";
  group.enabled = true;
  group.children.push_back(child);

  session.displays = {cloud, group};

  SavedViewConfig view;
  view.name = "Overview";
  view.type = "Orbit";
  view.yaw = 0.5f;
  view.pitch = 0.25f;
  view.distance = 12.5f;
  view.target_x = 1.5f;
  view.focal_shape_fixed_size = false;
  session.views = {view};

  ToolConfig tool;
  tool.id = "Measure";
  tool.properties["line_width"] = "0.05";
  session.tools = {tool};

  return session;
}

void TestRoundTrip(const QString& path) {
  using autoviz::common::SessionConfig;
  using autoviz::common::SessionConfigIO;

  const SessionConfig original = MakeSession();
  Check(SessionConfigIO::save(path.toStdString(), original),
        "session saves to disk");

  SessionConfig loaded;
  Check(SessionConfigIO::load(path.toStdString(), &loaded),
        "session loads back");

  Check(loaded.fixed_frame == original.fixed_frame, "fixed frame survives");
  Check(loaded.frame_rate == original.frame_rate, "frame rate survives");
  Check(loaded.background_color == original.background_color,
        "background colour survives");
  Check(loaded.view_controller == original.view_controller,
        "view controller survives");
  Check(loaded.render_backend == original.render_backend,
        "render backend survives");
  Check(loaded.active_tool == original.active_tool, "active tool survives");
  Check(loaded.toolbar_tools == original.toolbar_tools,
        "toolbar tools survive");
  Check(loaded.visible_panels == original.visible_panels,
        "visible panels survive");
  Check(loaded.hide_left_dock == original.hide_left_dock,
        "dock hide state survives");
  Check(loaded.main_panel_state_b64 == original.main_panel_state_b64,
        "mosaic state blob survives");
  Check(loaded.window_geometry_b64 == original.window_geometry_b64,
        "window geometry survives");

  Check(loaded.displays.size() == 2, "top-level display count survives");
  if (loaded.displays.size() == 2) {
    const auto& cloud = loaded.displays[0];
    Check(cloud.type == "PointCloud2", "display type survives");
    Check(cloud.channel == "/points", "display channel survives");
    Check(!cloud.enabled, "display enabled flag survives");
    Check(cloud.properties.count("style") == 1 &&
              cloud.properties.at("style") == "Flat Squares",
          "display property survives");

    const auto& group = loaded.displays[1];
    Check(group.type == "Group", "group type survives");
    Check(group.children.size() == 1, "group child count survives");
    if (!group.children.empty()) {
      Check(group.children[0].type == "Grid", "group child type survives");
      Check(group.children[0].properties.count("plane") == 1,
            "group child property survives");
    }
  }

  Check(loaded.views.size() == 1, "saved view count survives");
  if (!loaded.views.empty()) {
    const auto& view = loaded.views.front();
    Check(view.name == "Overview", "view name survives");
    Check(view.type == "Orbit", "view type survives");
    Check(Near(view.yaw, 0.5f) && Near(view.pitch, 0.25f),
          "view orientation survives");
    Check(Near(view.distance, 12.5f), "view distance survives");
    Check(Near(view.target_x, 1.5f), "view target survives");
    Check(!view.focal_shape_fixed_size, "view bool survives");
  }

  Check(loaded.tools.size() == 1, "tool config count survives");
  if (!loaded.tools.empty()) {
    Check(loaded.tools.front().id == "Measure", "tool id survives");
    Check(loaded.tools.front().properties.count("line_width") == 1,
          "tool property survives");
  }
}

}  // namespace

int main() {
  QTemporaryDir dir;
  if (!dir.isValid()) {
    std::fprintf(stderr, "FAIL: temporary directory unavailable\n");
    return 1;
  }
  TestRoundTrip(dir.filePath(QStringLiteral("session.autoviz")));

  if (g_failures != 0) {
    std::fprintf(stderr, "%d session config test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("session config tests passed\n");
  return 0;
}

