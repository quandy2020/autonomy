/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file property_panel.hpp
 * @brief Shared sidebar host that shows one content widget for the current
 *        main-panel selection (inspector chrome).
 *
 * VisualizationFrame swaps Plot / Indicator / other settings widgets into this
 * panel via @ref setContentWidget(); when nothing is selected it shows a
 * placeholder via @ref clearContent().
 *
 * @see SelectionPanel
 * @see ToolPropertiesPanel
 */

#pragma once

#include <QWidget>

class QLabel;
class QVBoxLayout;

namespace autoviz {

/**
 * @class PropertyInspectorPanel
 * @brief Lightweight inspector shell: title label + single content host.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────────┐
 * │ Title                       │
 * ├─────────────────────────────┤
 * │  (content_widget_)          │  or placeholder_
 * └─────────────────────────────┘
 * @endcode
 *
 * The panel does not own domain-specific editors — callers create those
 * widgets and pass them to @ref setContentWidget(). Clearing removes the
 * content from the layout but does not delete it (caller retains ownership
 * unless the widget's Qt parent is this panel).
 *
 * @note Only one content widget is visible at a time; previous content is
 *       detached from @c content_layout_ when a new one is set.
 */
class PropertyInspectorPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty inspector showing the placeholder.
   *
   * @param parent Qt parent (typically an inspector dock).
   */
  explicit PropertyInspectorPanel(QWidget* parent = nullptr);

  /**
   * @brief Shows @p widget under @p title, replacing any previous content.
   *
   * @param widget Content to display; reparented into @c content_host_.
   * @param title Text for the header label.
   * @see clearContent()
   */
  void setContentWidget(QWidget* widget, const QString& title);

  /**
   * @brief Removes the current content and restores the placeholder.
   *
   * @see setContentWidget()
   * @see showPlaceholder()
   */
  void clearContent();

  /**
   * @brief Returns the widget currently hosted, if any.
   *
   * @return Non-owning pointer to @c content_widget_, or @c nullptr when
   *         showing the placeholder.
   */
  QWidget* contentWidget() const { return content_widget_; }

 private:
  /**
   * @brief Ensures the placeholder child is visible and titled accordingly.
   */
  void showPlaceholder();

  /**
   * @brief Detaches all widgets from @c content_layout_ without deleting them.
   */
  void clearContentLayout();

  /** Header label showing the active content title. */
  QLabel* title_label_ = nullptr;

  /** Host widget whose layout holds either content or placeholder. */
  QWidget* content_host_ = nullptr;

  /** Vertical layout inside @c content_host_. */
  QVBoxLayout* content_layout_ = nullptr;

  /** Shown when no selection / content is active. */
  QWidget* placeholder_ = nullptr;

  /** Currently displayed content widget (non-owning aside from Qt parenting). */
  QWidget* content_widget_ = nullptr;
};

}  // namespace autoviz
