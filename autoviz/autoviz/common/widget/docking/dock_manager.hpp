/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#pragma once

#include <QByteArray>
#include <QHash>
#include <QPointer>
#include <QString>

class QDockWidget;
class QMainWindow;
class QWidget;

namespace autoviz::widget {

// Semantic dock locations keep callers independent of a docking backend.
enum class DockArea {
  kLeft,
  kRight,
  kTop,
  kBottom,
};

struct DockPanelOptions {
  bool closable = true;
  bool movable = true;
  bool floatable = true;
};

// Stable Autoviz docking facade. The default backend uses QDockWidget. An
// Advanced Docking System backend may implement the same facade in an
// independently linked LGPL shared library.
class DockManager final {
 public:
  explicit DockManager(QMainWindow* host);

  DockManager(const DockManager&) = delete;
  DockManager& operator=(const DockManager&) = delete;

  QDockWidget* AddPanel(const QString& id, const QString& title,
                        QWidget* content, DockArea area,
                        const DockPanelOptions& options = {});
  bool RemovePanel(const QString& id);
  QDockWidget* Panel(const QString& id) const;
  void SetPanelVisible(const QString& id, bool visible);

  QByteArray SaveLayout(int version = 0) const;
  bool RestoreLayout(const QByteArray& state, int version = 0);

 private:
  QPointer<QMainWindow> host_;
  QHash<QString, QPointer<QDockWidget>> panels_;
};

}  // namespace autoviz::widget
