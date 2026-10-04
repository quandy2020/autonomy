/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file add_dialog.hpp
 * @brief Modal dialog to pick a panel type from the available catalog.
 *
 * Used by Panels → Add Panel… and related chrome entry points. Returns the
 * selected dock @c objectName via @ref selectedPanelObjectName().
 *
 * @see PanelCatalog()
 * @see FramePanels::onAddPanel()
 * @see FrameChrome::onAddPanel()
 */

#pragma once

#include <QDialog>
#include <QPoint>
#include <QStringList>

class QDialogButtonBox;
class QListWidget;
class QMouseEvent;
class QPaintEvent;

namespace autoviz {

/**
 * @class AddPanelDialog
 * @brief Frameless-style picker listing available panel types.
 *
 * ## Interaction
 *
 * - Double-click or OK accepts the current list selection
 * - Custom @ref paintEvent() draws frosted chrome; title drag via mouse
 *   press/move/release when the dialog is frameless
 *
 * @see selectedPanelObjectName()
 * @see PanelCatalogEntry
 */
class AddPanelDialog : public QDialog {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the dialog and populates @p available_panels.
   *
   * @param available_panels Dock object names (or catalog ids) the user may
   *        add; typically filtered to unimplemented / already-singleton types
   *        by the caller.
   * @param parent Parent widget (usually @ref VisualizationFrame).
   */
  explicit AddPanelDialog(const QStringList& available_panels,
                          QWidget* parent = nullptr);

  /**
   * @brief Preferred dialog size for the panel list.
   * @return Size hint.
   */
  QSize sizeHint() const override;

  /**
   * @brief Object name / type id of the selected panel.
   * @return Selected id, or empty if nothing was chosen.
   */
  QString selectedPanelObjectName() const;

 protected:
  /**
   * @brief Paints frosted dialog chrome behind the list and buttons.
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Starts title-bar window drag when pressing in the chrome region.
   * @param event Mouse press event.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Continues window drag while @c dragging_ is set.
   * @param event Mouse move event.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends window drag.
   * @param event Mouse release event.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  /**
   * @brief Fills @c list_ from @p available_panels (icons + labels).
   * @param available_panels Panel ids to show.
   */
  void populate(const QStringList& available_panels);

  /** Selectable panel list. */
  QListWidget* list_ = nullptr;

  /** OK / Cancel buttons. */
  QDialogButtonBox* button_box_ = nullptr;

  /** @c true while dragging the frameless window. */
  bool dragging_ = false;

  /** Cursor offset from window top-left at drag start. */
  QPoint drag_offset_;
};

}  // namespace autoviz
