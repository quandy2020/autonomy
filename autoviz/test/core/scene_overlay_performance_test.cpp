// Copyright 2026 The Openbot Authors
#include <chrono>
#include <cmath>
#include <cstdio>

#include "autoviz/rendering/scene_overlay.hpp"

int main() {
  using Overlay = autoviz::rendering::SceneOverlay;
  QMatrix4x4 view;
  view.lookAt({0, 0, 20}, {0, 0, 0}, {0, 1, 0});
  for (int segments : {2000, 8000, 30000}) {
    Overlay scene;
    for (int i = 0; i < segments; ++i) {
      const float y = (i % 100) * .05f;
      scene.addViewFacingPolylineStrip({{0, y, .1f}, {1, y, .1f}}, .025f, Qt::yellow);
    }
    const auto start = std::chrono::steady_clock::now();
    const auto triangles = scene.expandedFlatTriangles(view);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::printf("%d segments: %.3f ms, %zu vertices\n", segments, ms, triangles.size());
    std::fflush(stdout);
    if (triangles.size() != static_cast<size_t>(segments) * 6) return 1;
    const float width = (triangles[0].position - triangles[1].position).length();
    if (std::abs(width - .025f) > 1e-6f || triangles[0].color != QVector4D(1, 1, 0, 1)) return 1;
    // Catch the former per-segment reallocation (seconds), allowing slow CI hosts.
    if (segments == 30000 && ms > 1000.) return 1;
  }
  Overlay scene;
  scene.addColoredTriangle({-1, 0, 0}, {0, 1, 0}, {1, 0, 0}, Qt::red, Qt::red, Qt::red);
  scene.addViewFacingPolylineStrip({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, .025f, Qt::green);
  scene.addViewFacingPolylineStrip({{2, 2, 0}, {2, 2, 0}}, .025f, Qt::blue);
  auto triangles = scene.expandedFlatTriangles(view);
  if (triangles.size() != 15 || triangles[0].color != QVector4D(1, 0, 0, 1)) return 1;
  scene.clear();
  if (!scene.expandedFlatTriangles(view).empty()) return 1;
  std::puts("PASS large constraint geometry, metric width, colors, joints and clear");
  return 0;
}
