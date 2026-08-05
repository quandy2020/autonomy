/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

class QFormLayout;
class QGroupBox;
class QLayout;
class QPushButton;
class QScrollArea;
class QVBoxLayout;
class QWidget;

class QColor;

namespace autoviz {

/** Shared Foxglove-style settings panel styling for Property Inspector content. */
struct PanelSettingsLayout {
  static constexpr int kOuterMargin = 6;
  static constexpr int kOuterSpacing = 6;
  static constexpr int kSectionSpacing = 4;
};

QString SettingsWidgetBackgroundStyle();
QString CompactGroupStyle();
QString SegmentedToggleStyle();
QString PanelStatusBarStyle();
QString PanelFooterStyle();
QString PanelStatusLabelStyle();
QString PropertyInspectorTitleStyle();
QString PropertyInspectorHintStyle();

void ApplyCompactSettingsShell(QWidget* widget);
void ApplyCompactForm(QFormLayout* form);
void ApplyCompactVBox(QVBoxLayout* layout);
void StyleSettingsGroupBox(QGroupBox* group);

QPushButton* MakeFlatActionButton(const QString& text, QWidget* parent);
QPushButton* MakeDestructiveFlatActionButton(const QString& text, QWidget* parent);
void UpdateColorButton(QPushButton* button, const QColor& color);

QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title, QWidget* body,
                                bool expanded);

/** Returns the scroll area for Property Inspector embedding (never detaches to top-level). */
QWidget* SettingsScrollForInspector(QScrollArea* scroll);

/** Restores a settings scroll area into its hidden panel container. */
void RecallSettingsScrollToContainer(QScrollArea* scroll, QWidget* container);

}  // namespace autoviz
