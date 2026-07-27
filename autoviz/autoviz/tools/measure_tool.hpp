/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/** RViz-style two-click measure with live preview line and distance. */
class MeasureTool : public common::Tool {
 public:
  std::string id() const override { return "Measure"; }
  QString label() const override { return QStringLiteral("Measure"); }

  void deactivate() override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  QString statusText() const override;

 private:
  bool pickPoint(int x, int y, QVector3D* hit) const;
  void resetMeasurement();
  void clearOgreOverlay() const;
  void updateOgreLineVisual(const QVector3D& start, const QVector3D& end) const;
  void refreshLineVisual() const;
  void updateStatus() const;
  float currentLength() const;
  std::optional<QVector3D> endPreview() const;

  static constexpr const char* kToolOgreId = "Measure";

  /** True after first left-click release; preview follows the cursor. */
  bool line_started_ = false;
  std::optional<QVector3D> start_point_;
  std::optional<QVector3D> end_point_;
  std::optional<QVector3D> hover_point_;
};

}  // namespace tools
}  // namespace autoviz
