/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file record_drop_overlay.hpp
 * @brief Full-window frosted hint shown while dragging Autolink records.
 *
 * Displayed by @ref FrameSession during drag-and-drop of @c .record /
 * @c .bag / @c .mcap files onto @ref VisualizationFrame.
 *
 * @see FrameSession::setupRecordDropOverlay()
 * @see FrameSession::showRecordDropOverlay()
 */

#pragma once

#include <QWidget>

namespace autoviz {

/**
 * @class RecordDropOverlay
 * @brief Full-window drop target hint for Autolink @c .record / @c .bag /
 *        @c .mcap.
 *
 * Transparent to layout (typically stacked over the frame client area). Paints
 * a frosted glass card with a drop message in @ref paintEvent(). Does not
 * accept drops itself — the parent frame handles MIME; this widget is visual
 * feedback only.
 *
 * @note Parent should call @c setAttribute(Qt::WA_TransparentForMouseEvents)
 *       when appropriate so the overlay does not steal drag events.
 *
 * @see FrameSession::handleRecordMime()
 * @see glass::PaintShellGlass()
 */
class RecordDropOverlay : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the overlay (initially intended to be hidden by the
   *        session collaborator).
   *
   * @param parent Typically @ref VisualizationFrame (covers client area).
   */
  explicit RecordDropOverlay(QWidget* parent = nullptr);

 protected:
  /**
   * @brief Paints the frosted drop-target chrome and instructional text.
   * @param event Paint event (rectangle unused; paints @c rect()).
   */
  void paintEvent(QPaintEvent* event) override;
};

}  // namespace autoviz
