/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/** RViz-style two-click measure with live preview line and distance.
 *  Measurement state is stored per 3D viewport so Split panels are independent.
 */
class MeasureTool : public common::Tool {
 public:
  std::string id() const override { return "Measure"; }
  QString label() const override { return QStringLiteral("Measure"); }

  void deactivate() override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void clearViewportSession(const std::string& viewport_key) override;
  QString statusText() const override;

 private:
  struct Session {
    bool line_started = false;
    std::optional<QVector3D> start_point;
    std::optional<QVector3D> end_point;
    std::optional<QVector3D> hover_point;
  };

  std::string currentViewportKey() const;
  Session& sessionFor(const std::string& key);
  const Session* findSession(const std::string& key) const;

  bool pickPoint(int x, int y, QVector3D* hit) const;
  void resetMeasurement(const std::string& key);
  void clearOgreOverlay(const std::string& key) const;
  void updateOgreLineVisual(const std::string& key, const QVector3D& start,
                            const QVector3D& end) const;
  void refreshLineVisual(const std::string& key) const;
  void updateStatus() const;
  float currentLength(const Session& session) const;
  std::optional<QVector3D> endPreview(const Session& session) const;
  void drawSession(rendering::SceneOverlay& scene, const std::string& key,
                   const Session& session) const;

  std::unordered_map<std::string, Session> sessions_;
};

}  // namespace tools
}  // namespace autoviz
