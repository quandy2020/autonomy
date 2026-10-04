/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief Shared panel chrome: object-name ids, QSS snippets, and layout helpers.
 *
 * Sidebar panels (Displays, Views, …) and settings surfaces share Foxglove-style
 * toolbar / footer / tree styling. Prefer these helpers over ad-hoc QSS so
 * @ref ApplyAppTheme() @c #objectName rules stay consistent.
 *
 * @see AppThemeIds
 * @see ApplyAppTheme()
 * @see PaintPanelFrostedCard()
 * @see glass::ShellTokens
 */

#pragma once

#include <QString>

class QAbstractItemView;
class QFormLayout;
class QFrame;
class QGroupBox;
class QHBoxLayout;
class QLabel;
class QLayout;
class QLineEdit;
class QPainter;
class QPushButton;
class QRect;
class QScrollArea;
class QVBoxLayout;
class QWidget;

class QColor;

namespace autoviz {

/**
 * @namespace AppThemeIds
 * @brief QSS object names — styled globally in @ref ApplyAppTheme()
 *        (CSS-like @c #id selectors).
 *
 * Call @c setObjectName() with these constants on chrome widgets so the global
 * stylesheet applies without per-widget @c setStyleSheet().
 */
namespace AppThemeIds {
inline constexpr char kPanelContent[] = "AutovizPanelContent"; /**< Panel body host. */
inline constexpr char kViewportHost[] = "AutovizViewportHost"; /**< 3D viewport host. */
inline constexpr char kPanelToolbar[] = "AutovizPanelToolbar"; /**< Panel top toolbar. */
inline constexpr char kPanelFooter[] = "AutovizPanelFooter";   /**< Panel footer strip. */
inline constexpr char kDockTitleBar[] = "AutovizDockTitleBar"; /**< Custom dock title. */
inline constexpr char kPanelTitleTools[] = "AutovizPanelTitleTools"; /**< Title-bar tools host. */
inline constexpr char kHintLabel[] = "AutovizHintLabel";       /**< Muted hint label. */
inline constexpr char kSectionTitle[] = "AutovizSectionTitle"; /**< Settings section title. */
inline constexpr char kPanelTree[] = "AutovizPanelTree";       /**< Property / display tree. */
inline constexpr char kSegmentedToggle[] = "AutovizSegmentedToggle"; /**< Segmented control. */
inline constexpr char kSettingsScroll[] = "AutovizSettingsScroll"; /**< Settings scroll area. */
inline constexpr char kPropertyInspectorTitle[] = "AutovizPropertyInspectorTitle"; /**< Inspector header. */
inline constexpr char kMenuBar[] = "AutovizMenuBar";           /**< Main menu bar. */
inline constexpr char kToolBar[] = "AutovizToolBar";           /**< Main toolbar. */
inline constexpr char kPanelsMenu[] = "AutovizPanelsMenu";     /**< Panels catalog menu. */
inline constexpr char kAppMenu[] = "AutovizAppMenu";           /**< Frosted app popup menu. */
}  // namespace AppThemeIds

/**
 * @struct PanelSettingsLayout
 * @brief Shared Foxglove-style settings panel spacing for Property Inspector
 *        content.
 */
struct PanelSettingsLayout {
  /** Matches PanelDockWidget title-bar left inset (icon / title column). */
  static constexpr int kOuterMargin = 8;    /**< Outer layout margin (px). */
  static constexpr int kOuterSpacing = 4;   /**< Outer layout spacing (px). */
  static constexpr int kSectionSpacing = 3; /**< Between settings sections (px). */
};

/**
 * @struct PanelChromeLayout
 * @brief Shared chrome layout for panel toolbars and footers.
 */
struct PanelChromeLayout {
  static constexpr int kToolbarMarginH = 8;  /**< Toolbar horizontal margin. */
  static constexpr int kToolbarMarginV = 5;  /**< Toolbar vertical margin. */
  static constexpr int kFooterMarginH = 8;   /**< Footer horizontal margin. */
  static constexpr int kFooterMarginV = 4;   /**< Footer vertical margin. */
  static constexpr int kFooterHeight = 20;   /**< Footer fixed height hint. */
  static constexpr int kToolbarSpacing = 4;  /**< Toolbar item spacing. */
};

/**
 * @brief Background QSS for settings / inspector content widgets.
 * @return QSS fragment.
 */
QString SettingsWidgetBackgroundStyle();

/**
 * @brief Shell QSS scoped to @p object_name.
 * @param object_name Widget @c objectName to target.
 * @return QSS fragment.
 */
QString PanelShellStyle(const QString& object_name);

/**
 * @brief @c true when @p widget (or an ancestor) should skip frosted shell
 *        background painting (e.g. viewport hosts).
 * @param widget Widget under test.
 */
bool ShouldSkipPanelShellBackground(const QWidget* widget);

/** @brief Compact @c QGroupBox QSS for settings sections. */
QString CompactGroupStyle();
/** @brief Segmented toggle control QSS. */
QString SegmentedToggleStyle();
/** @brief Panel status-bar strip QSS. */
QString PanelStatusBarStyle();
/** @brief Panel footer strip QSS. */
QString PanelFooterStyle();
/** @brief Default status label QSS. */
QString PanelStatusLabelStyle();
/** @brief Error-state status label QSS. */
QString PanelStatusLabelErrorStyle();
/** @brief Muted hint label QSS. */
QString PanelHintLabelStyle();
/** @brief Filter @c QLineEdit QSS. */
QString PanelFilterLineEditStyle();
/** @brief Icon-only clear button QSS. */
QString PanelIconClearButtonStyle();
/** @brief Compact flat action button QSS. */
QString PanelCompactButtonStyle();
/** @brief Help / documentation browser QSS. */
QString PanelHelpBrowserStyle();
/** @brief Standard panel tree QSS. */
QString PanelTreeWidgetStyle();

/**
 * @brief Displays-matched frosted tree: transparent items + teal continuous
 *        selection.
 * @return QSS fragment.
 */
QString PanelFrostedTreeStyle();

/** @brief Frosted list view QSS (same selection language as the tree). */
QString PanelFrostedListStyle();
/** @brief Frosted help pane QSS. */
QString PanelFrostedHelpStyle();
/** @brief Frosted footer QSS matching Displays / Views. */
QString PanelFrostedFooterStyle();
/** @brief Primary (filled accent) button QSS. */
QString PanelPrimaryButtonStyle();
/** @brief Ghost / outline button QSS. */
QString PanelGhostButtonStyle();
/** @brief Panel splitter handle QSS. */
QString PanelSplitterStyle();
/** @brief Custom dock title bar QSS. */
QString DockTitleBarStyle();
/** @brief Dock title label QSS. */
QString DockTitleLabelStyle();
/** @brief Main window status bar QSS. */
QString MainWindowStatusBarStyle();
/** @brief Property Inspector title QSS. */
QString PropertyInspectorTitleStyle();
/** @brief Property Inspector hint QSS. */
QString PropertyInspectorHintStyle();

/**
 * @brief Applies shell object name + background to @p widget.
 * @param widget Panel content root.
 */
void ApplyPanelShell(QWidget* widget);

/**
 * @brief Applies compact settings shell styling to @p widget.
 * @param widget Settings / inspector content root.
 */
void ApplyCompactSettingsShell(QWidget* widget);

/**
 * @brief Force short Property Inspector fields (overrides tall app-theme
 *        controls).
 * @param field Line/combo/spin widget to compact.
 */
void StyleCompactSettingsField(QWidget* field);

/**
 * @brief Recursively compact all line/combo/spin fields under @p root.
 * @param root Subtree root.
 */
void PolishSettingsFields(QWidget* root);

/**
 * @brief Creates a settings form label with shared typography.
 * @param text Label text.
 * @param parent Parent widget.
 * @return New @c QLabel.
 */
QLabel* MakeSettingsFormLabel(const QString& text, QWidget* parent);

/**
 * @brief Applies compact margins/spacing to a settings @c QFormLayout.
 * @param form Form layout to adjust.
 */
void ApplyCompactForm(QFormLayout* form);

/**
 * @brief Applies compact margins/spacing to a settings @c QVBoxLayout.
 * @param layout Vertical layout to adjust.
 */
void ApplyCompactVBox(QVBoxLayout* layout);

/**
 * @brief Applies @ref PanelChromeLayout toolbar margins/spacing.
 * @param layout Toolbar horizontal layout.
 */
void ApplyPanelToolbarLayout(QHBoxLayout* layout);

/**
 * @brief Styles a status label for normal or error state.
 * @param label Target label.
 * @param is_error @c true to use error colors.
 */
void StylePanelStatusLabel(QLabel* label, bool is_error = false);

/**
 * @brief Applies hint-label object name and QSS.
 * @param label Target label.
 */
void StyleHintLabel(QLabel* label);

/**
 * @brief Applies section-title object name and QSS.
 * @param label Target label.
 */
void StyleSectionTitle(QLabel* label);

/**
 * @brief Styles a settings @c QGroupBox with compact chrome.
 * @param group Target group box.
 */
void StyleSettingsGroupBox(QGroupBox* group);

/**
 * @brief Styles a filter line edit (search boxes).
 * @param edit Target line edit.
 */
void StyleFilterLineEdit(QLineEdit* edit);

/**
 * @brief Applies standard panel tree styling.
 * @param tree Tree or tree-like widget.
 */
void StylePanelTree(QWidget* tree);

/**
 * @brief Frosted list chrome + continuous teal selection pill (name column
 *        paints row).
 *
 * @param tree Item view to style.
 * @param name_column Column index used for full-row selection painting.
 */
void StyleFrostedPanelTree(QAbstractItemView* tree, int name_column = 0);

/**
 * @brief Frosted list view styling (no tree indentation).
 * @param list List view to style.
 */
void StyleFrostedPanelList(QAbstractItemView* list);

/**
 * @brief Applies toolbar object name and chrome QSS to @p toolbar.
 * @param toolbar Toolbar frame.
 */
void ApplyPanelToolbarChrome(QFrame* toolbar);

/**
 * @brief Applies footer object name and chrome QSS to @p footer.
 * @param footer Footer frame.
 */
void ApplyPanelFooterChrome(QFrame* footer);

/**
 * @brief Applies title-tools host chrome to @p tools.
 * @param tools Title-bar tools container.
 */
void ApplyPanelTitleToolsChrome(QWidget* tools);

/**
 * @brief Paint milky frosted card fill used by Displays / Views sidebars.
 *
 * @param painter Active painter.
 * @param rect Card rectangle.
 * @param radius Corner radius (default 14).
 * @see ViewsPanel::paintEvent()
 */
void PaintPanelFrostedCard(QPainter& painter, const QRect& rect,
                           qreal radius = 14.0);

/**
 * @brief Creates a panel toolbar frame with an optional layout out-param.
 *
 * @param parent Parent widget.
 * @param[out] layout_out Receives the toolbar @c QHBoxLayout when non-null.
 * @return New toolbar @c QFrame.
 */
QFrame* MakePanelToolbar(QWidget* parent, QHBoxLayout** layout_out = nullptr);

/**
 * @brief Creates a panel footer frame with an optional status label out-param.
 *
 * @param parent Parent widget.
 * @param[out] status_label_out Receives the status @c QLabel when non-null.
 * @return New footer @c QFrame.
 */
QFrame* MakePanelFooter(QWidget* parent, QLabel** status_label_out = nullptr);

/**
 * @brief Flat (ghost) action button with panel chrome.
 * @param text Button label.
 * @param parent Parent widget.
 * @return New @c QPushButton.
 */
QPushButton* MakeFlatActionButton(const QString& text, QWidget* parent);

/**
 * @brief Primary (accent-filled) action button.
 * @param text Button label.
 * @param parent Parent widget.
 * @return New @c QPushButton.
 */
QPushButton* MakePrimaryActionButton(const QString& text, QWidget* parent);

/**
 * @brief Destructive flat action button (e.g. Remove / Delete).
 * @param text Button label.
 * @param parent Parent widget.
 * @return New @c QPushButton.
 */
QPushButton* MakeDestructiveFlatActionButton(const QString& text,
                                             QWidget* parent);

/**
 * @brief Updates a color-picker button's swatch / stylesheet from @p color.
 * @param button Color button to update.
 * @param color Current color.
 */
void UpdateColorButton(QPushButton* button, const QColor& color);

/**
 * @brief Builds a collapsible settings section (title + body).
 *
 * @param parent Parent widget.
 * @param title Section header text.
 * @param body Content widget (reparented).
 * @param expanded Initial expanded state.
 * @return Section container widget.
 */
QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title,
                                QWidget* body, bool expanded);

/**
 * @brief Wraps @p scroll for Property Inspector use (object name / policies).
 * @param scroll Existing scroll area.
 * @return The same scroll area after styling (convenience).
 */
QWidget* SettingsScrollForInspector(QScrollArea* scroll);

/**
 * @brief Restores @p scroll viewport widget to @p container after temporary
 *        reparenting.
 *
 * @param scroll Scroll area.
 * @param container Content widget to reattach.
 */
void RecallSettingsScrollToContainer(QScrollArea* scroll, QWidget* container);

}  // namespace autoviz
