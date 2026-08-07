/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>

#include <QIcon>
#include <QString>
#include <QWidget>

class QToolButton;

namespace autoviz {

struct ViewportFloatingToolbarCallbacks {
  std::function<void()> on_inspect;
  std::function<void()> on_toggle_2d_camera;
  std::function<void()> on_measure;
  std::function<void()> on_recenter_frame;
};

/** Foxglove-style floating tool groups on the right edge of a 3D viewport. */
class ViewportFloatingToolbar : public QWidget {
  Q_OBJECT

 public:
  explicit ViewportFloatingToolbar(QWidget* parent = nullptr);

  void setCallbacks(ViewportFloatingToolbarCallbacks callbacks);

  void setInspectChecked(bool checked);
  void setMeasureChecked(bool checked);
  void set2dCameraChecked(bool checked);

  void setRecenterToolTip(const QString& tip);

 private:
  QToolButton* MakeToolButton(const QIcon& icon, const QString& tip,
                              bool checkable = false);
  QToolButton* MakeTextToolButton(const QString& text, const QString& tip,
                                  bool checkable = false);

  ViewportFloatingToolbarCallbacks callbacks_;
  QToolButton* inspect_button_ = nullptr;
  QToolButton* camera_2d_button_ = nullptr;
  QToolButton* measure_button_ = nullptr;
  QToolButton* recenter_button_ = nullptr;
};

}  // namespace autoviz
