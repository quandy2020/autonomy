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
  return QStringLiteral("background: palette(window);");
}

QString PanelShellStyle(const QString& object_name) {
  return QStringLiteral("#%1 { background: palette(window); }").arg(object_name);
}

bool ShouldSkipPanelShellBackground(const QWidget* widget) {
  if (widget == nullptr) {
    return true;
  }
  return widget->objectName() ==
         QString::fromLatin1(AppThemeIds::kViewportHost);
}

QString CompactGroupStyle() {
  return QString();
}

QString SegmentedToggleStyle() {
  return QString();
}

QString PanelStatusBarStyle() {
  return QString();
}

QString PanelFooterStyle() {
  return QString();
}

QString PanelStatusLabelStyle() {
  return QStringLiteral("color: palette(mid); font-size: 10px; padding: 0; margin: 0;");
}

QString PanelStatusLabelErrorStyle() {
  return QStringLiteral("color: #c62828; font-size: 10px; padding: 0; margin: 0;");
}

QString PanelHintLabelStyle() {
  return QString();
}

QString PanelFilterLineEditStyle() {
  return QString();
}

QString PanelIconClearButtonStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  color: palette(mid);"
      "  background: transparent;"
      "  border: none;"
      "  font-size: 14px;"
      "  padding: 0px;"
      "  border-radius: 6px;"
      "}"
      "QToolButton:hover { color: palette(text); background: palette(alternate-base); }");
}

QString PanelCompactButtonStyle() {
  return QString();
}

QString PanelHelpBrowserStyle() {
  return QStringLiteral(
      "QTextBrowser {"
      "  background: palette(base);"
      "  color: palette(text);"
      "  border: none;"
      "  border-top: 1px solid palette(mid);"
      "  padding: 8px;"
      "  font-size: 11px;"
      "}");
}

QString PanelTreeWidgetStyle() {
  return QString();
}

QString PanelSplitterStyle() {
  return QString();
}

QString DockTitleBarStyle() {
  return QString();
}

QString DockTitleLabelStyle() {
  return QString();
}

QString MainWindowStatusBarStyle() {
  return QString();
}

QString PropertyInspectorTitleStyle() {
  return QString();
}

QString PropertyInspectorHintStyle() {
  return QString();
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
  label->setStyleSheet(QString());
}

void StyleSectionTitle(QLabel* label) {
  if (label == nullptr) {
    return;
  }
  label->setObjectName(QString::fromLatin1(AppThemeIds::kSectionTitle));
  label->setStyleSheet(QString());
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
  tree->setStyleSheet(QString());
}

void ApplyPanelToolbarChrome(QFrame* toolbar) {
  if (toolbar == nullptr) {
    return;
  }
  toolbar->setObjectName(QString::fromLatin1(AppThemeIds::kPanelToolbar));
  toolbar->setStyleSheet(QString());
}

void ApplyPanelFooterChrome(QFrame* footer) {
  if (footer == nullptr) {
    return;
  }
  footer->setObjectName(QString::fromLatin1(AppThemeIds::kPanelFooter));
  footer->setStyleSheet(QString());
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
  toolbar->setObjectName(QString::fromLatin1(AppThemeIds::kPanelToolbar));
  toolbar->setStyleSheet(QString());
  auto* layout = new QHBoxLayout(toolbar);
  ApplyPanelToolbarLayout(layout);
  if (layout_out != nullptr) {
    *layout_out = layout;
  }
  return toolbar;
}

QFrame* MakePanelFooter(QWidget* parent, QLabel** status_label_out) {
  auto* footer = new QFrame(parent);
  footer->setObjectName(QString::fromLatin1(AppThemeIds::kPanelFooter));
  footer->setFixedHeight(PanelChromeLayout::kFooterHeight);
  footer->setStyleSheet(QString());
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
