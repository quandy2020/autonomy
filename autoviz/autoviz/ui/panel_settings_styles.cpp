/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_settings_styles.hpp"

#include <QColor>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

namespace autoviz {

QString SettingsWidgetBackgroundStyle() {
  return QStringLiteral("background: palette(window);");
}

QString CompactGroupStyle() {
  return QStringLiteral(
      "QGroupBox {"
      "  font-weight: 600;"
      "  margin-top: 6px;"
      "  padding-top: 12px;"
      "  border: 1px solid palette(midlight);"
      "  border-radius: 4px;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  left: 6px;"
      "  padding: 0 4px;"
      "}");
}

QString SegmentedToggleStyle() {
  return QStringLiteral(
      "QWidget { background: palette(midlight); border-radius: 4px; }"
      "QPushButton { border: none; padding: 4px 10px; }"
      "QPushButton:checked { background: palette(base); border-radius: 4px; }");
}

QString PanelStatusBarStyle() {
  return QStringLiteral(
      "QFrame {"
      "  background: palette(base);"
      "  border-bottom: 1px solid palette(midlight);"
      "}");
}

QString PanelFooterStyle() {
  return QStringLiteral(
      "QFrame {"
      "  background: palette(base);"
      "  border-top: 1px solid palette(midlight);"
      "}");
}

QString PanelStatusLabelStyle() {
  return QStringLiteral("color: palette(mid); font-size: 10px; padding: 0; margin: 0;");
}

QString PropertyInspectorTitleStyle() {
  return QStringLiteral(
      "font-weight: 600; padding: 6px 8px; color: palette(text);"
      "border-bottom: 1px solid palette(midlight); background: palette(base);");
}

QString PropertyInspectorHintStyle() {
  return QStringLiteral("color: palette(mid); padding: 12px;");
}

void ApplyCompactSettingsShell(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
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

void StyleSettingsGroupBox(QGroupBox* group) {
  if (group == nullptr) {
    return;
  }
  group->setStyleSheet(CompactGroupStyle());
}

QPushButton* MakeFlatActionButton(const QString& text, QWidget* parent) {
  auto* button = new QPushButton(text, parent);
  button->setFlat(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(
      QStringLiteral("QPushButton { text-align: left; padding: 2px 4px; color: #1976D2; }"
                     "QPushButton:hover { background: rgba(25,118,210,0.08); }"));
  return button;
}

QPushButton* MakeDestructiveFlatActionButton(const QString& text, QWidget* parent) {
  auto* button = MakeFlatActionButton(text, parent);
  button->setStyleSheet(
      QStringLiteral("QPushButton { text-align: left; padding: 2px 4px; color: #c62828; }"
                     "QPushButton:hover { background: rgba(198,40,40,0.08); }"));
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
                     " border-radius: 3px; padding: 2px 6px;")
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
