/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file change_menu.hpp
 * @brief Searchable panel picker popup (Foxglove Change panel).
 *
 * Standalone @c Qt::Popup — not embedded in @c QMenu — so the full catalog
 * column is visible without menu scrolling.
 *
 * @see CreatePanelContextMenu()
 * @see PanelCatalog()
 * @see FrameLayout::changePanelInDock()
 */

#pragma once

#include <functional>

#include <QSize>
#include <QVector>
#include <QWidget>

class QLineEdit;
class QShowEvent;
class QToolButton;
class QVBoxLayout;

namespace autoviz {

/**
 * @class ChangePanelMenuWidget
 * @brief Searchable panel picker shown as a frameless @c Qt::Popup.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────┐
 * │ [Search…            ]   │
 * ├─────────────────────────┤
 * │  🖼 Image               │
 * │  📈 Plot                │
 * │  … (all rows, one col)  │
 * └─────────────────────────┘
 * @endcode
 *
 * @see PanelCatalog()
 * @see ShowChangePanelPopup()
 */
class ChangePanelMenuWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds search field + catalog rows from @ref PanelCatalog().
   * @param parent Ownership parent (not the popup window parent).
   */
  explicit ChangePanelMenuWidget(QWidget* parent = nullptr);

  /**
   * @brief Shows this popup below @p anchor, sized to every visible row.
   * @param anchor Title-bar / menu widget used for placement.
   */
  void popupBelow(QWidget* anchor);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void showEvent(QShowEvent* event) override;
  bool event(QEvent* event) override;

 signals:
  /**
   * @brief Emitted when the user activates a catalog row.
   * @param object_name Dock object name / type id to switch to.
   */
  void panelSelected(const QString& object_name);

 private slots:
  void onFilterChanged(const QString& text);

 private:
  void populate();
  void refitToContents();

  struct Row {
    QToolButton* button = nullptr;
    QString label;
    QString object_name;
  };

  QLineEdit* search_ = nullptr;
  QWidget* rows_host_ = nullptr;
  QVBoxLayout* rows_layout_ = nullptr;
  QVector<Row> rows_;
  bool selecting_ = false;
};

/**
 * @brief Creates a heap-owned picker and shows it under @p anchor.
 *
 * The picker deletes itself when closed. @p on_selected runs after close.
 */
void ShowChangePanelPopup(
    QWidget* anchor,
    const std::function<void(const QString& object_name)>& on_selected);

}  // namespace autoviz
