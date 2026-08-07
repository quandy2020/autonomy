/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

class QFormLayout;
class QFrame;
class QGroupBox;
class QHBoxLayout;
class QLabel;
class QLayout;
class QLineEdit;
class QPushButton;
class QScrollArea;
class QVBoxLayout;
class QWidget;

class QColor;

namespace autoviz {

/** QSS object names — styled globally in ApplyAppTheme (CSS-like #id selectors). */
namespace AppThemeIds {
inline constexpr char kPanelContent[] = "AutovizPanelContent";
inline constexpr char kViewportHost[] = "AutovizViewportHost";
inline constexpr char kPanelToolbar[] = "AutovizPanelToolbar";
inline constexpr char kPanelFooter[] = "AutovizPanelFooter";
inline constexpr char kDockTitleBar[] = "AutovizDockTitleBar";
inline constexpr char kPanelTitleTools[] = "AutovizPanelTitleTools";
inline constexpr char kHintLabel[] = "AutovizHintLabel";
inline constexpr char kSectionTitle[] = "AutovizSectionTitle";
inline constexpr char kPanelTree[] = "AutovizPanelTree";
inline constexpr char kSegmentedToggle[] = "AutovizSegmentedToggle";
inline constexpr char kSettingsScroll[] = "AutovizSettingsScroll";
inline constexpr char kPropertyInspectorTitle[] = "AutovizPropertyInspectorTitle";
inline constexpr char kMenuBar[] = "AutovizMenuBar";
inline constexpr char kPanelsMenu[] = "AutovizPanelsMenu";
}  // namespace AppThemeIds

/** Shared Foxglove-style settings panel styling for Property Inspector content. */
struct PanelSettingsLayout {
  static constexpr int kOuterMargin = 6;
  static constexpr int kOuterSpacing = 6;
  static constexpr int kSectionSpacing = 4;
};

/** Shared chrome layout for panel toolbars and footers. */
struct PanelChromeLayout {
  static constexpr int kToolbarMarginH = 8;
  static constexpr int kToolbarMarginV = 5;
  static constexpr int kFooterMarginH = 8;
  static constexpr int kFooterMarginV = 4;
  static constexpr int kFooterHeight = 20;
  static constexpr int kToolbarSpacing = 4;
};

QString SettingsWidgetBackgroundStyle();
QString PanelShellStyle(const QString& object_name);

bool ShouldSkipPanelShellBackground(const QWidget* widget);
QString CompactGroupStyle();
QString SegmentedToggleStyle();
QString PanelStatusBarStyle();
QString PanelFooterStyle();
QString PanelStatusLabelStyle();
QString PanelStatusLabelErrorStyle();
QString PanelHintLabelStyle();
QString PanelFilterLineEditStyle();
QString PanelIconClearButtonStyle();
QString PanelCompactButtonStyle();
QString PanelHelpBrowserStyle();
QString PanelTreeWidgetStyle();
QString PanelSplitterStyle();
QString DockTitleBarStyle();
QString DockTitleLabelStyle();
QString MainWindowStatusBarStyle();
QString PropertyInspectorTitleStyle();
QString PropertyInspectorHintStyle();

void ApplyPanelShell(QWidget* widget);
void ApplyCompactSettingsShell(QWidget* widget);
void ApplyCompactForm(QFormLayout* form);
void ApplyCompactVBox(QVBoxLayout* layout);
void ApplyPanelToolbarLayout(QHBoxLayout* layout);
void StylePanelStatusLabel(QLabel* label, bool is_error = false);
void StyleHintLabel(QLabel* label);
void StyleSectionTitle(QLabel* label);
void StyleSettingsGroupBox(QGroupBox* group);
void StyleFilterLineEdit(QLineEdit* edit);
void StylePanelTree(QWidget* tree);
void ApplyPanelToolbarChrome(QFrame* toolbar);
void ApplyPanelFooterChrome(QFrame* footer);
void ApplyPanelTitleToolsChrome(QWidget* tools);

QFrame* MakePanelToolbar(QWidget* parent, QHBoxLayout** layout_out = nullptr);
QFrame* MakePanelFooter(QWidget* parent, QLabel** status_label_out = nullptr);

QPushButton* MakeFlatActionButton(const QString& text, QWidget* parent);
QPushButton* MakeDestructiveFlatActionButton(const QString& text, QWidget* parent);
void UpdateColorButton(QPushButton* button, const QColor& color);

QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title, QWidget* body,
                                bool expanded);

QWidget* SettingsScrollForInspector(QScrollArea* scroll);
void RecallSettingsScrollToContainer(QScrollArea* scroll, QWidget* container);

}  // namespace autoviz
