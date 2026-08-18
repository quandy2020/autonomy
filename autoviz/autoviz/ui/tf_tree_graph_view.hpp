/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QGraphicsView>
#include <QString>

#include <vector>

#include "autoviz/transform/buffer.hpp"

class QGraphicsScene;

namespace autoviz {

/** rqt_tf_tree-style canvas: frame boxes joined by labelled parent to child
 *  arrows, laid out top-down the way graphviz `dot` arranges view_frames output.
 */
class TfTreeGraphView : public QGraphicsView {
  Q_OBJECT

 public:
  explicit TfTreeGraphView(QWidget* parent = nullptr);

  /** Rebuild the scene from TF statistics.
   * `current_time_seconds` <= 0 omits the "sec old" annotation, matching
   * `_allFramesAsDot` when no clock is supplied. Zoom and scroll survive the
   * rebuild unless a fit was requested. */
  void setFrames(const std::vector<transform::TfFrameStats>& frames,
                 double current_time_seconds, const QString& filter);
  /** Replace the scene with a single centred message. */
  void showMessage(const QString& message);
  /** Fit on the next render; used when the tree structure changes. */
  void requestFit();
  void zoomToFit();
  void setCurrentFrame(const QString& frame_id);

 signals:
  void graphRendered(int frame_count, int root_count);
  void frameActivated(const QString& frame_id);

 protected:
  void mousePressEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  QGraphicsScene* scene_ = nullptr;
  std::vector<transform::TfFrameStats> last_frames_;
  double last_current_time_seconds_ = 0.0;
  QString last_filter_;
  QString current_frame_id_;
  double zoom_factor_ = 1.0;
  bool fit_pending_ = true;
  /** True while the view still shows a fit; cleared once the user zooms. */
  bool fitted_ = false;
};

}  // namespace autoviz
