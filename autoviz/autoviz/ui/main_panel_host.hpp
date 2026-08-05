/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMainWindow>

class QResizeEvent;

namespace autoviz {

/** Central column: draggable main panels (3D, Plot, Log, Image, …). */
class MainPanelHost : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainPanelHost(QWidget* parent = nullptr);

  /** Resize visible left/right docks to fill the host width. */
  void syncHorizontalDockLayout();

 protected:
  void resizeEvent(QResizeEvent* event) override;
};

}  // namespace autoviz
