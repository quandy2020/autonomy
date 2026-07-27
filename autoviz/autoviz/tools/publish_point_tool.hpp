/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QCursor>
#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class PublishPointTool : public common::Tool {
 public:
  std::string id() const override { return "PublishPoint"; }
  QString label() const override { return QStringLiteral("Publish Point"); }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"topic", "Topic", "/clicked_point"}};
  }

  bool mousePressEvent(QMouseEvent* event) override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  void activate(common::ToolContext* context) override;
  void deactivate() override;
  void onDraw(rendering::SceneOverlay& scene) override;
  QString statusText() const override;

 private:
  bool pickPoint(int x, int y, QVector3D* hit) const;
  void updateStatusFromPoint(const QVector3D& point, const QString& prefix) const;
  std::string publishChannel() const {
    return propertyValue("topic", "/clicked_point");
  }

  std::optional<QVector3D> last_point_;
  std::optional<QVector3D> hover_point_;
  QCursor hit_cursor_;
  QCursor std_cursor_;
};

}  // namespace tools
}  // namespace autoviz
