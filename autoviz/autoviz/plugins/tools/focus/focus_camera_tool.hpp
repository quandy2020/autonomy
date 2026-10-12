/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QCursor>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class FocusCameraTool : public common::Tool {
 public:
  std::string id() const override { return "FocusCamera"; }
  QString label() const override { return QStringLiteral("Focus Camera"); }
  char shortcutKey() const override { return 'c'; }

  void activate(common::ToolContext* context) override;
  void deactivate() override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  QString statusText() const override;

 private:
  bool resolveLookAtPoint(int x, int y, QVector3D* hit) const;
  void updateHover(int x, int y);

  QCursor hit_cursor_;
  QCursor std_cursor_;
};

}  // namespace tools
}  // namespace autoviz
