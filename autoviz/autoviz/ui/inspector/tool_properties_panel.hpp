/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tool_properties_panel.hpp
 * @brief Inspector form for the currently active viewport tool properties.
 *
 * Reads property names/values from the active tool via
 * @ref common::VisualizationManager and presents them as a QFormLayout of
 * line edits. Edits write back into the tool and emit @ref propertiesChanged().
 *
 * @see PropertyInspectorPanel
 * @see common::VisualizationManager
 */

#pragma once

#include <memory>
#include <vector>

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QWidget>

#include "autoviz/common/visualization_manager.hpp"

namespace autoviz {

/**
 * @class ToolPropertiesPanel
 * @brief Form-based editor for the active tool's key/value properties.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────────┐
 * │ Tool: Interact              │  tool_label_
 * ├─────────────────────────────┤
 * │ prop_a  [____________]      │  property_form_
 * │ prop_b  [____________]      │
 * └─────────────────────────────┘
 * @endcode
 *
 * Call @ref refresh() when the user switches tools or when external code
 * mutates tool properties. While @c updating_ is true, edit signals are
 * ignored to avoid feedback loops during @ref populateProperties().
 *
 * @see propertiesChanged()
 */
class ToolPropertiesPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel and builds the initial property form.
   *
   * @param manager Shared visualization manager providing the active tool.
   * @param parent Qt parent widget.
   */
  explicit ToolPropertiesPanel(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  /**
   * @brief Rebuilds the form from the active tool's current properties.
   *
   * Updates @c tool_label_ and recreates line edits in @c property_form_.
   *
   * @see populateProperties()
   */
  void refresh();

 signals:
  /**
   * @brief Emitted after the user edits a property line edit.
   *
   * Listeners may persist tool config or request a viewport redraw.
   */
  void propertiesChanged();

 private slots:
  /**
   * @brief Line-edit edited: writes the value into the active tool.
   *
   * Suppressed while @c updating_ is set. Emits @ref propertiesChanged() on
   * success.
   */
  void onPropertyEdited();

 private:
  /**
   * @brief Builds the tool label and empty form container.
   */
  void setupUi();

  /**
   * @brief Clears and recreates form rows for each property on the active tool.
   */
  void populateProperties();

  /** Shared manager that owns / exposes the active tool. */
  std::shared_ptr<common::VisualizationManager> manager_;

  /** Shows the active tool's display name. */
  QLabel* tool_label_ = nullptr;

  /** Form layout of property name → QLineEdit rows. */
  QFormLayout* property_form_ = nullptr;

  /** Host widget for @c property_form_. */
  QWidget* property_container_ = nullptr;

  /** Parallel list of editors created by @ref populateProperties(). */
  std::vector<QLineEdit*> property_edits_;

  /**
   * @brief Re-entrancy guard while programmatically filling line edits.
   *
   * Prevents @ref onPropertyEdited() from writing back during @ref refresh().
   */
  bool updating_ = false;
};

}  // namespace autoviz
