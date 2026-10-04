/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file measure_tool.hpp
 * @brief RViz-style Measure tool — two-click distance with live preview.
 *
 * Measurement state is keyed by viewport (@c ToolContext::viewport_key) so
 * Split 3D panels keep independent sessions. Draws a preview line while
 * hovering after the first click; second click commits the segment length.
 *
 * @see common::Tool
 * @see FocusCameraTool
 */

#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class MeasureTool
 * @brief Two-click ground/scene measure with per-viewport session state.
 *
 * ## Interaction
 *
 * 1. First click: set @c start_point, begin line preview.
 * 2. Move: update @c hover_point and status length.
 * 3. Second click: set @c end_point, show committed length.
 * 4. Next click (or deactivate / clear): reset and start over.
 *
 * @note @ref clearViewportSession() drops state when a Split panel is closed.
 */
class MeasureTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id for the registry / session config.
   * @return @c "Measure".
   */
  std::string id() const override { return "Measure"; }

  /**
   * @brief Human-readable toolbar label.
   * @return Localized @c "Measure".
   */
  QString label() const override { return QStringLiteral("Measure"); }

  /**
   * @brief Clears active measurement visuals and detaches context.
   */
  void deactivate() override;

  /**
   * @brief Updates hover preview while a line is in progress.
   *
   * @param event Mouse move in viewport coordinates.
   * @return @c true when the hover pick was handled for the active session.
   */
  bool mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Places start or end point on release (two-click measure).
   *
   * @param event Mouse release in viewport coordinates.
   * @return @c true when a pick advanced or completed the measurement.
   */
  bool mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Draws the measure line / endpoints for the current viewport session.
   * @param scene Active viewport scene overlay.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Drops interaction state for one Split 3D panel.
   *
   * @param viewport_key Dock / panel key matching @c ToolContext::viewport_key.
   */
  void clearViewportSession(const std::string& viewport_key) override;

  /**
   * @brief Status text with live or committed distance.
   * @return Length summary, or instructional text when idle.
   */
  QString statusText() const override;

 private:
  /**
   * @struct Session
   * @brief Per-viewport measure state (start, end, hover).
   */
  struct Session {
    /** @c true after the first click until reset. */
    bool line_started = false;
    /** First click world position. */
    std::optional<QVector3D> start_point;
    /** Second click world position (committed). */
    std::optional<QVector3D> end_point;
    /** Live hover while waiting for the second click. */
    std::optional<QVector3D> hover_point;
  };

  /**
   * @brief Resolves the active viewport key from @c ToolContext.
   * @return Viewport key string (may be empty if context unset).
   */
  std::string currentViewportKey() const;

  /**
   * @brief Returns a mutable session for @p key, creating one if needed.
   *
   * @param key Viewport key.
   * @return Reference into @c sessions_.
   */
  Session& sessionFor(const std::string& key);

  /**
   * @brief Looks up an existing session without creating one.
   *
   * @param key Viewport key.
   * @return Pointer into @c sessions_, or @c nullptr if absent.
   */
  const Session* findSession(const std::string& key) const;

  /**
   * @brief Ray-picks a world point at viewport pixel (@p x, @p y).
   *
   * @param x Viewport X in pixels.
   * @param y Viewport Y in pixels.
   * @param[out] hit World hit position on success.
   * @return @c true if a hit was found.
   */
  bool pickPoint(int x, int y, QVector3D* hit) const;

  /**
   * @brief Clears session fields and Ogre overlay for @p key.
   * @param key Viewport key to reset.
   */
  void resetMeasurement(const std::string& key);

  /**
   * @brief Removes any Ogre-hosted measure line for @p key.
   * @param key Viewport key.
   */
  void clearOgreOverlay(const std::string& key) const;

  /**
   * @brief Creates / updates the Ogre line visual between @p start and @p end.
   *
   * @param key Viewport key for the visual name.
   * @param start Line start in world coordinates.
   * @param end Line end in world coordinates.
   */
  void updateOgreLineVisual(const std::string& key, const QVector3D& start,
                            const QVector3D& end) const;

  /**
   * @brief Refreshes Ogre / overlay line from the session's start and preview end.
   * @param key Viewport key.
   */
  void refreshLineVisual(const std::string& key) const;

  /** Pushes length / instructional status via @c ToolContext::set_status. */
  void updateStatus() const;

  /**
   * @brief Computes Euclidean length for a session's start→end/hover segment.
   *
   * @param session Session whose points to measure.
   * @return Length in meters (world units), or @c 0 if incomplete.
   */
  float currentLength(const Session& session) const;

  /**
   * @brief Chooses committed end or hover as the preview endpoint.
   *
   * @param session Session to read.
   * @return Endpoint for drawing / length, if available.
   */
  std::optional<QVector3D> endPreview(const Session& session) const;

  /**
   * @brief Draws one session's markers and line into @p scene.
   *
   * @param scene Overlay canvas.
   * @param key Viewport key (for Ogre naming).
   * @param session Session state to visualize.
   */
  void drawSession(rendering::SceneOverlay& scene, const std::string& key,
                   const Session& session) const;

  /** Measure sessions keyed by @c ToolContext::viewport_key. */
  std::unordered_map<std::string, Session> sessions_;
};

}  // namespace tools
}  // namespace autoviz
