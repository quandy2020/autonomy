/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file graph_view.hpp
 * @brief rqt_tf_tree-style canvas: TF frame boxes with labelled parent→child
 *        arrows, laid out top-down like graphviz @c dot / @c view_frames.
 *
 * Embedded as a tab inside @ref TfTreePanel alongside the statistics tree.
 *
 * @see TfTreePanel
 * @see transform::TfFrameStats
 * @see transform::Buffer
 */

#pragma once

#include <QGraphicsView>
#include <QString>

#include <vector>

#include "autoviz/transform/buffer.hpp"

class QGraphicsScene;

namespace autoviz {

/**
 * @class TfTreeGraphView
 * @brief Interactive TF tree diagram with zoom, fit, and frame activation.
 *
 * ## Behaviour
 *
 * - @ref setFrames() rebuilds the scene from TF statistics; zoom/scroll are
 *   preserved unless @ref requestFit() was pending.
 * - @c current_time_seconds <= 0 omits the "sec old" annotation (matches
 *   @c _allFramesAsDot when no clock is supplied).
 * - Clicking a frame box emits @ref frameActivated(); @ref setCurrentFrame()
 *   highlights the matching box from the tree selection.
 *
 * @note Layout is recomputed on each @ref setFrames(); positions are not
 *       user-draggable (unlike @ref channel_graph::ChannelGraphView).
 */
class TfTreeGraphView : public QGraphicsView {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty scene ready for @ref setFrames().
   *
   * @param parent Qt parent (typically a tab inside @ref TfTreePanel).
   */
  explicit TfTreeGraphView(QWidget* parent = nullptr);

  /**
   * @brief Rebuild the scene from TF statistics.
   *
   * @c current_time_seconds <= 0 omits the "sec old" annotation, matching
   * @c _allFramesAsDot when no clock is supplied. Zoom and scroll survive the
   * rebuild unless a fit was requested.
   *
   * @param frames Per-frame TF statistics from @ref transform::Buffer.
   * @param current_time_seconds Clock used for age annotations; <= 0 disables.
   * @param filter Substring filter; empty shows all frames.
   * @see graphRendered()
   */
  void setFrames(const std::vector<transform::TfFrameStats>& frames,
                 double current_time_seconds, const QString& filter);

  /**
   * @brief Replace the scene with a single centred message.
   *
   * Used for empty / error states (e.g. no TF data yet).
   *
   * @param message Text drawn in the middle of the view.
   */
  void showMessage(const QString& message);

  /**
   * @brief Fit on the next render; used when the tree structure changes.
   *
   * Sets @c fit_pending_ so the next @ref setFrames() / show / resize calls
   * @ref zoomToFit().
   */
  void requestFit();

  /**
   * @brief Immediately frames the scene in the viewport.
   *
   * Sets @c fitted_ so subsequent wheel zooms clear the "still fitted" flag.
   */
  void zoomToFit();

  /**
   * @brief Highlights the box for @p frame_id (from tree selection).
   *
   * @param frame_id TF frame name; empty clears the highlight.
   */
  void setCurrentFrame(const QString& frame_id);

 signals:
  /**
   * @brief Emitted after a successful @ref setFrames() rebuild.
   *
   * @param frame_count Number of frame boxes drawn.
   * @param root_count Number of root frames (no parent).
   */
  void graphRendered(int frame_count, int root_count);

  /**
   * @brief Emitted when the user clicks a frame box.
   *
   * @param frame_id Activated TF frame name.
   */
  void frameActivated(const QString& frame_id);

 protected:
  /**
   * @brief Hit-tests frame boxes and emits @ref frameActivated().
   *
   * @param event Mouse press.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Zooms about the cursor; clears @c fitted_.
   *
   * @param event Wheel event.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * @brief Applies a pending fit when the view first becomes visible.
   *
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Re-fits when @c fitted_ is still true after a resize.
   *
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

 private:
  /** Owned graphics scene hosting frame boxes and arrows. */
  QGraphicsScene* scene_ = nullptr;

  /** Last frames passed to @ref setFrames() (for rebuild / highlight). */
  std::vector<transform::TfFrameStats> last_frames_;

  /** Last clock value used for age annotations. */
  double last_current_time_seconds_ = 0.0;

  /** Last substring filter applied. */
  QString last_filter_;

  /** Currently highlighted frame id. */
  QString current_frame_id_;

  /** Cumulative wheel zoom factor relative to fit. */
  double zoom_factor_ = 1.0;

  /** When true, the next opportunity should call @ref zoomToFit(). */
  bool fit_pending_ = true;

  /**
   * @brief True while the view still shows a fit; cleared once the user zooms.
   */
  bool fitted_ = false;
};

}  // namespace autoviz
