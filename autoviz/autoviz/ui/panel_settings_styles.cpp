/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_settings_styles.hpp"

#include <QColor>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

namespace autoviz {

QString SettingsWidgetBackgroundStyle() {
  return QStringLiteral("background: #f8f9fb;");
}

QString PanelShellStyle(const QString& object_name) {
  return QStringLiteral("#%1 { background: #f8f9fb; color: #1e293b; }")
      .arg(object_name);
}

bool ShouldSkipPanelShellBackground(const QWidget* widget) {
  if (widget == nullptr) {
    return true;
  }
  return widget->objectName() ==
         QString::fromLatin1(AppThemeIds::kViewportHost);
}

QString CompactGroupStyle() {
  return QStringLiteral(
      "QGroupBox {"
      "  border: 1px solid #cbd5e1;"
      "  border-radius: 10px;"
      "  margin-top: 12px;"
      "  padding: 14px 10px 10px 10px;"
      "  background: #ffffff;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin; left: 10px; padding: 0 6px;"
      "  color: #64748b; font-weight: 700; font-size: 11px;"
      "}");
}

QString SegmentedToggleStyle() {
  return QStringLiteral(
      "QWidget {"
      "  background: #f1f5f9; border: 1px solid #cbd5e1;"
      "  border-radius: 8px; padding: 2px;"
      "}"
      "QPushButton {"
      "  border: none; padding: 4px 10px; background: transparent;"
      "  border-radius: 6px; color: #64748b;"
      "}"
      "QPushButton:checked {"
      "  background: #ffffff; color: #0891b2; font-weight: 700;"
      "}");
}

QString PanelStatusBarStyle() {
  return QStringLiteral(
      "background: #f8f9fb; border-top: 1px solid #cbd5e1;");
}

QString PanelFooterStyle() {
  return QStringLiteral(
      "background: #f8f9fb; border-top: 1px solid #cbd5e1;");
}

QString PanelStatusLabelStyle() {
  return QStringLiteral("color: #64748b; font-size: 11px; padding: 0; margin: 0;");
}

QString PanelStatusLabelErrorStyle() {
  return QStringLiteral("color: #dc2626; font-size: 11px; padding: 0; margin: 0;");
}

QString PanelHintLabelStyle() {
  return QStringLiteral("color: #64748b; font-size: 11px;");
}

QString PanelFilterLineEditStyle() {
  return QStringLiteral(
      "QLineEdit {"
      "  background: #ffffff; color: #1e293b;"
      "  border: 1px solid #cbd5e1; border-radius: 8px;"
      "  padding: 6px 10px; min-height: 26px;"
      "}"
      "QLineEdit:focus { border-color: #0891b2; }");
}

QString PanelIconClearButtonStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  color: #64748b;"
      "  background: transparent;"
      "  border: none;"
      "  font-size: 14px;"
      "  padding: 0px;"
      "  border-radius: 6px;"
      "}"
      "QToolButton:hover { color: #0891b2; background: rgba(8,145,178,0.10); }");
}

QString PanelCompactButtonStyle() {
  return QStringLiteral(
      "QPushButton, QToolButton {"
      "  color: #0891b2; background: rgba(8,145,178,0.10);"
      "  border: 1px solid rgba(8,145,178,0.35); border-radius: 8px;"
      "  padding: 4px 10px; font-size: 12px; font-weight: 600;"
      "}"
      "QPushButton:hover, QToolButton:hover { background: rgba(8,145,178,0.18); }"
      "QPushButton:pressed, QToolButton:pressed { background: rgba(8,145,178,0.26); }");
}

QString PanelHelpBrowserStyle() {
  return QStringLiteral(
      "QTextBrowser {"
      "  background: #ffffff;"
      "  color: #1e293b;"
      "  border: none;"
      "  border-top: 1px solid #cbd5e1;"
      "  padding: 10px;"
      "  font-size: 12px;"
      "}");
}

QString PanelTreeWidgetStyle() {
  return QStringLiteral(
      "QTreeWidget, QTreeView {"
      "  background: #ffffff; color: #1e293b; border: none; outline: none;"
      "}"
      "QTreeWidget::item, QTreeView::item {"
      "  padding: 3px 6px; min-height: 24px;"
      "}"
      "QTreeWidget::item:selected, QTreeView::item:selected {"
      "  background: rgba(8,145,178,0.14); color: #1e293b;"
      "}"
      "QTreeWidget::item:hover:!selected, QTreeView::item:hover:!selected {"
      "  background: rgba(15,23,42,0.04);"
      "}"
      "QHeaderView::section {"
      "  background: #f8f9fb; color: #64748b;"
      "  border: none; border-bottom: 1px solid #cbd5e1;"
      "  padding: 7px 8px; font-size: 11px; font-weight: 700;"
      "}");
}

QString PanelSplitterStyle() {
  return QStringLiteral("QSplitter::handle { background: #cbd5e1; }");
}

QString DockTitleBarStyle() {
  return QStringLiteral(
      "background: #ffffff; border-bottom: 1px solid #cbd5e1;");
}

QString DockTitleLabelStyle() {
  return QStringLiteral(
      "font-weight: 600; color: #1e293b; padding: 0 4px;");
}

QString MainWindowStatusBarStyle() {
  return QStringLiteral(
      "background: #f8f9fb; color: #64748b; border-top: 1px solid #cbd5e1;");
}

QString PropertyInspectorTitleStyle() {
  return QStringLiteral(
      "font-weight: 700; padding: 6px 8px; color: #1e293b;"
      "border-bottom: 1px solid #cbd5e1; background: #f8f9fb;");
}

QString PropertyInspectorHintStyle() {
  return QStringLiteral("color: #64748b; font-size: 11px;");
}

void ApplyPanelShell(QWidget* widget) {
  if (widget == nullptr || ShouldSkipPanelShellBackground(widget)) {
    return;
  }
  if (widget->objectName().isEmpty()) {
    widget->setObjectName(QString::fromLatin1(AppThemeIds::kPanelContent));
  }
  widget->setAutoFillBackground(true);
  widget->setAttribute(Qt::WA_StyledBackground, true);
  widget->setStyleSheet(PanelShellStyle(widget->objectName()));
}

void ApplyCompactSettingsShell(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
  widget->setAutoFillBackground(true);
  widget->setAttribute(Qt::WA_StyledBackground, true);
  widget->setStyleSheet(SettingsWidgetBackgroundStyle());
}

void ApplyCompactForm(QFormLayout* form) {
  if (form == nullptr) {
    return;
  }
  form->setContentsMargins(PanelSettingsLayout::kOuterMargin, 4,
                           PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin);
  form->setSpacing(PanelSettingsLayout::kSectionSpacing);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
}

void ApplyCompactVBox(QVBoxLayout* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->setContentsMargins(PanelSettingsLayout::kOuterMargin,
                             PanelSettingsLayout::kOuterMargin,
                             PanelSettingsLayout::kOuterMargin,
                             PanelSettingsLayout::kOuterMargin);
  layout->setSpacing(PanelSettingsLayout::kOuterSpacing);
  layout->setAlignment(Qt::AlignTop);
}

void ApplyPanelToolbarLayout(QHBoxLayout* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->setContentsMargins(PanelChromeLayout::kToolbarMarginH,
                             PanelChromeLayout::kToolbarMarginV,
                             PanelChromeLayout::kToolbarMarginH,
                             PanelChromeLayout::kToolbarMarginV);
  layout->setSpacing(PanelChromeLayout::kToolbarSpacing);
}

void StylePanelStatusLabel(QLabel* label, bool is_error) {
  if (label == nullptr) {
    return;
  }
  label->setStyleSheet(is_error ? PanelStatusLabelErrorStyle() : PanelStatusLabelStyle());
}

void StyleHintLabel(QLabel* label) {
  if (label == nullptr) {
    return;
  }
  label->setObjectName(QString::fromLatin1(AppThemeIds::kHintLabel));
  label->setStyleSheet(PanelHintLabelStyle());
}

void StyleSectionTitle(QLabel* label) {
  if (label == nullptr) {
    return;
  }
  label->setObjectName(QString::fromLatin1(AppThemeIds::kSectionTitle));
  label->setStyleSheet(QStringLiteral(
      "font-size: 13px; font-weight: 700; color: #1e293b; background: transparent;"));
}

void StyleSettingsGroupBox(QGroupBox* group) {
  if (group == nullptr) {
    return;
  }
  group->setStyleSheet(CompactGroupStyle());
}

void StyleFilterLineEdit(QLineEdit* edit) {
  if (edit == nullptr) {
    return;
  }
  edit->setStyleSheet(PanelFilterLineEditStyle());
}

void StylePanelTree(QWidget* tree) {
  if (tree == nullptr) {
    return;
  }
  tree->setObjectName(QString::fromLatin1(AppThemeIds::kPanelTree));
  tree->setStyleSheet(PanelTreeWidgetStyle());
}

void ApplyPanelToolbarChrome(QFrame* toolbar) {
  if (toolbar == nullptr) {
    return;
  }
  toolbar->setObjectName(QString::fromLatin1(AppThemeIds::kPanelToolbar));
  toolbar->setStyleSheet(QStringLiteral(
      "QFrame#AutovizPanelToolbar {"
      "  background: #ffffff; border-bottom: 1px solid #cbd5e1;"
      "}"));
}

void ApplyPanelFooterChrome(QFrame* footer) {
  if (footer == nullptr) {
    return;
  }
  footer->setObjectName(QString::fromLatin1(AppThemeIds::kPanelFooter));
  footer->setStyleSheet(QStringLiteral(
      "QFrame#AutovizPanelFooter {"
      "  background: #f8f9fb; border-top: 1px solid #cbd5e1;"
      "}"));
}

void ApplyPanelTitleToolsChrome(QWidget* tools) {
  if (tools == nullptr) {
    return;
  }
  tools->setObjectName(QString::fromLatin1(AppThemeIds::kPanelTitleTools));
  tools->setStyleSheet(QString());
}

QFrame* MakePanelToolbar(QWidget* parent, QHBoxLayout** layout_out) {
  auto* toolbar = new QFrame(parent);
  ApplyPanelToolbarChrome(toolbar);
  auto* layout = new QHBoxLayout(toolbar);
  ApplyPanelToolbarLayout(layout);
  if (layout_out != nullptr) {
    *layout_out = layout;
  }
  return toolbar;
}

QFrame* MakePanelFooter(QWidget* parent, QLabel** status_label_out) {
  auto* footer = new QFrame(parent);
  ApplyPanelFooterChrome(footer);
  footer->setFixedHeight(PanelChromeLayout::kFooterHeight);
  auto* layout = new QHBoxLayout(footer);
  layout->setContentsMargins(PanelChromeLayout::kFooterMarginH, 0,
                             PanelChromeLayout::kFooterMarginH, 0);
  layout->setSpacing(0);
  if (status_label_out != nullptr) {
    auto* label = new QLabel(footer);
    label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    StylePanelStatusLabel(label);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(label, 1);
    *status_label_out = label;
  }
  return footer;
}

QPushButton* MakeFlatActionButton(const QString& text, QWidget* parent) {
  auto* button = new QPushButton(text, parent);
  button->setFlat(true);
  button->setCursor(Qt::PointingHandCursor);
  return button;
}

QPushButton* MakeDestructiveFlatActionButton(const QString& text, QWidget* parent) {
  auto* button = new QPushButton(text, parent);
  button->setFlat(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(
      QStringLiteral(
          "QPushButton { text-align: left; padding: 2px 4px; color: #c62828; }"
          "QPushButton:hover { background: rgba(198,40,40,0.08); border-radius: 6px; }"));
  return button;
}

void UpdateColorButton(QPushButton* button, const QColor& color) {
  if (button == nullptr) {
    return;
  }
  if (!color.isValid()) {
    button->setStyleSheet(QString());
    return;
  }
  button->setText(color.name(QColor::HexRgb));
  button->setStyleSheet(
      QStringLiteral("background:%1; color:white; border: 1px solid palette(mid);"
                       " border-radius: 6px; padding: 2px 6px;")
          .arg(color.name(QColor::HexRgb)));
}

QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title, QWidget* body,
                                bool expanded) {
  auto* section = new QGroupBox(title, parent);
  section->setCheckable(true);
  section->setChecked(expanded);
  StyleSettingsGroupBox(section);
  auto* layout = new QVBoxLayout(section);
  layout->setContentsMargins(PanelSettingsLayout::kOuterMargin, 4,
                             PanelSettingsLayout::kOuterMargin,
                             PanelSettingsLayout::kOuterMargin);
  layout->setSpacing(PanelSettingsLayout::kSectionSpacing);
  layout->addWidget(body);
  body->setVisible(expanded);
  QObject::connect(section, &QGroupBox::toggled, body, &QWidget::setVisible);
  return section;
}

QWidget* SettingsScrollForInspector(QScrollArea* scroll) {
  if (scroll != nullptr) {
    scroll->setObjectName(QString::fromLatin1(AppThemeIds::kSettingsScroll));
    scroll->setStyleSheet(QString());
  }
  return scroll;
}

void RecallSettingsScrollToContainer(QScrollArea* scroll, QWidget* container) {
  if (scroll == nullptr || container == nullptr) {
    return;
  }
  if (scroll->parentWidget() == container) {
    return;
  }
  scroll->setParent(container);
  if (QLayout* layout = container->layout()) {
    layout->addWidget(scroll);
  }
  scroll->hide();
}

}  // namespace autoviz
