/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file viewport_toolbar.hpp
 * @brief Foxglove-style floating tool groups on the right edge of a 3D viewport.
 *
 * Buttons: Inspect (Select), 2D camera toggle, Measure, Recenter, HUD toggle.
 * Uses a click-through mask so empty overlay area does not steal GL drags.
 *
 * @see FrameViewport::installViewportFloatingToolbar()
 * @see ViewportFloatingToolbarCallbacks
 * @see ViewportHudOverlay
 */

#pragma once

#include <functional>

#include <QIcon>
#include <QString>
#include <QWidget>

class QResizeEvent;
class QShowEvent;
class QToolButton;

namespace autoviz {

/**
 * @struct ViewportFloatingToolbarCallbacks
 * @brief Closures invoked when floating toolbar buttons are activated.
 *
 * Installed by @ref FrameViewport when wiring a @ref ViewportPanelEntry.
 */
struct ViewportFloatingToolbarCallbacks {
  std::function<void()> on_inspect;           /**< Toggle Select / inspect tool. */
  std::function<void()> on_toggle_2d_camera;  /**< Flip 2D ortho ↔ 3D camera. */
  std::function<void()> on_measure;           /**< Toggle Measure tool. */
  std::function<void()> on_recenter_frame;    /**< Recenter camera on fixed/target frame. */
  std::function<void(bool)> on_toggle_hud;    /**< Show/hide @ref ViewportHudOverlay. */
};

/**
 * @class ViewportFloatingToolbar
 * @brief Foxglove-style floating tool groups on the right edge of a 3D viewport.
 *
 * ## Click-through
 *
 * The widget spans the viewport edge but @ref updateClickThroughMask() sets a
 * mask to the button group bounds so empty overlay area does not steal OpenGL
 * mouse drags.
 *
 * ## Styling
 *
 * Uses @ref glass::OverlayTokens for frosted navy buttons; checkable tools
 * reflect @ref ViewportPanelEntry::local_tool_id via the @c set*Checked APIs.
 *
 * @see FrameViewport::syncViewportFloatingToolbarForEntry()
 */
class ViewportFloatingToolbar : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds tool buttons and applies initial overlay styling.
   * @param parent Typically the viewport host (stacked above the GL widget).
   */
  explicit ViewportFloatingToolbar(QWidget* parent = nullptr);

  /**
   * @brief Installs (or replaces) button callbacks.
   * @param callbacks Closures from @ref FrameViewport wiring.
   */
  void setCallbacks(ViewportFloatingToolbarCallbacks callbacks);

  /**
   * @brief Sets the Inspect (Select) button checked state.
   * @param checked @c true when the local tool is Select.
   */
  void setInspectChecked(bool checked);

  /**
   * @brief Sets the Measure button checked state.
   * @param checked @c true when the local tool is Measure.
   */
  void setMeasureChecked(bool checked);

  /**
   * @brief Sets the 2D camera toggle checked state.
   * @param checked @c true while top-down ortho is active.
   */
  void set2dCameraChecked(bool checked);

  /**
   * @brief Sets the HUD toggle checked state.
   * @param checked @c true when @ref ViewportHudOverlay is visible.
   */
  void setHudChecked(bool checked);

  /**
   * @brief Updates the Recenter button tooltip (e.g. current target frame).
   * @param tip Localized tooltip text.
   */
  void setRecenterToolTip(const QString& tip);

 protected:
  /**
   * @brief Refreshes the click-through mask when first shown.
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

  /**
   * @brief Repositions button groups and updates the click-through mask.
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

 private:
  /**
   * @brief Factory for an icon tool button with shared overlay chrome.
   *
   * @param icon Button icon.
   * @param tip Tooltip text.
   * @param checkable Whether the button is checkable.
   * @return New @c QToolButton parented to this toolbar.
   */
  QToolButton* MakeToolButton(const QIcon& icon, const QString& tip,
                              bool checkable = false);

  /**
   * @brief Factory for a text-labelled tool button.
   *
   * @param text Visible label.
   * @param tip Tooltip text.
   * @param checkable Whether the button is checkable.
   * @return New @c QToolButton parented to this toolbar.
   */
  QToolButton* MakeTextToolButton(const QString& text, const QString& tip,
                                  bool checkable = false);

  /**
   * @brief Mask to button groups so empty overlay area does not steal GL drags.
   */
  void updateClickThroughMask();

  /** Installed button callbacks (may be empty until wired). */
  ViewportFloatingToolbarCallbacks callbacks_;

  /** Inspect / Select tool. */
  QToolButton* inspect_button_ = nullptr;

  /** 2D camera toggle. */
  QToolButton* camera_2d_button_ = nullptr;

  /** Measure tool. */
  QToolButton* measure_button_ = nullptr;

  /** Recenter on frame. */
  QToolButton* recenter_button_ = nullptr;

  /** HUD visibility toggle. */
  QToolButton* hud_button_ = nullptr;
};

}  // namespace autoviz
