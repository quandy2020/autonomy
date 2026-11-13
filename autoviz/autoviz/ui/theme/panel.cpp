/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/panel.hpp"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QChildEvent>
#include <QColor>
#include <QComboBox>
#include <QEvent>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHash>
#include <QLabel>
#include <QLineEdit>
#include <QLinearGradient>
#include <QMetaObject>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPen>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStyleOptionViewItem>
#include <QStyledItemDelegate>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/widget/layout/collapsible_panel.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {

void PolishSettingsFields(QWidget* root);

namespace {

class ContinuousNameColumnDelegate : public QStyledItemDelegate {
 public:
  explicit ContinuousNameColumnDelegate(int name_column,
                                        QObject* parent = nullptr)
      : QStyledItemDelegate(parent), name_column_(name_column) {}

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);
    const bool selected = opt.state.testFlag(QStyle::State_Selected);
    const bool hovered = opt.state.testFlag(QStyle::State_MouseOver);

    if (index.column() == name_column_ && (selected || hovered)) {
      QRect row = opt.rect;
      if (const auto* view =
              qobject_cast<const QAbstractItemView*>(opt.widget)) {
        row = QRect(2, opt.rect.y(), view->viewport()->width() - 4,
                    opt.rect.height());
      }
      painter->save();
      painter->setRenderHint(QPainter::Antialiasing, true);
      painter->setPen(Qt::NoPen);
      painter->setBrush(selected ? glass::ShellSelectionFill()
                                 : glass::ShellHoverFill());
      painter->drawRoundedRect(QRectF(row).adjusted(0.5, 0.5, -0.5, -0.5), 6.0,
                               6.0);
      painter->restore();
    }

    // Kill focus/QSS second plate; keep icon+text via style with flags cleared.
    opt.state.setFlag(QStyle::State_Selected, false);
    opt.state.setFlag(QStyle::State_MouseOver, false);
    opt.state.setFlag(QStyle::State_HasFocus, false);
    opt.backgroundBrush = Qt::NoBrush;
    opt.showDecorationSelected = false;
    {
      const QColor ink =
          selected ? glass::ShellSelectionInk() : glass::Shell().text;
      opt.palette.setColor(QPalette::Text, ink);
      opt.palette.setColor(QPalette::WindowText, ink);
      if (selected) {
        opt.font.setWeight(QFont::DemiBold);
      }
    }
    QStyledItemDelegate::paint(painter, opt, index);
  }

 private:
  int name_column_ = 0;
};

class SettingsPolishFilter : public QObject {
 public:
  explicit SettingsPolishFilter(QObject* parent) : QObject(parent) {}

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override {
    if (event != nullptr && event->type() == QEvent::ChildAdded) {
      auto* child_event = static_cast<QChildEvent*>(event);
      if (auto* child = qobject_cast<QWidget*>(child_event->child())) {
        QTimer::singleShot(0, child,
                           [child]() { PolishSettingsFields(child); });
      }
    }
    return QObject::eventFilter(watched, event);
  }
};

}  // namespace

void ApplyItemViewShellPalette(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
  const glass::ShellTokens shell = glass::Shell();
  QPalette pal = widget->palette();
  pal.setBrush(QPalette::Base, Qt::transparent);
  pal.setBrush(QPalette::Window, Qt::transparent);
  pal.setColor(QPalette::Text, shell.text);
  pal.setColor(QPalette::WindowText, shell.window_text);
  pal.setColor(QPalette::ButtonText, shell.button_text);
  pal.setColor(QPalette::BrightText, shell.text);
  pal.setColor(QPalette::PlaceholderText, shell.secondary_text);
  pal.setColor(QPalette::Link, shell.link);
  pal.setBrush(QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::HighlightedText, glass::ShellSelectionInk());
  pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::Inactive, QPalette::HighlightedText,
               glass::ShellSelectionInk());
  widget->setPalette(pal);
  if (auto* view = qobject_cast<QAbstractItemView*>(widget)) {
    if (QWidget* viewport = view->viewport()) {
      viewport->setPalette(pal);
    }
  }
}

void StylePanelButton(QPushButton* button, const QString& role) {
  if (button == nullptr || role.isEmpty()) {
    return;
  }
  button->setProperty("autovizPanelButton", role);
  RefreshPanelButtonStyle(button);
}

void RefreshPanelButtonStyle(QPushButton* button) {
  if (button == nullptr) {
    return;
  }
  const QString role = button->property("autovizPanelButton").toString();
  if (role == QLatin1String("primary")) {
    button->setStyleSheet(PanelPrimaryButtonStyle());
  } else if (role == QLatin1String("ghost")) {
    button->setStyleSheet(PanelGhostButtonStyle());
  }
}

QString SettingsWidgetBackgroundStyle() {
  return style::sheet(QStringLiteral("chrome/settings"), style::tokens());
}

QString PanelShellStyle(const QString& object_name) {
  const QString id = object_name.isEmpty()
                         ? QStringLiteral("AutovizPanelContent")
                         : object_name;
  QHash<QString, QString> tokens = style::tokens();
  tokens.insert(QStringLiteral("{{name}}"), id);
  return style::sheet(QStringLiteral("chrome/panel"), tokens);
}

bool ShouldSkipPanelShellBackground(const QWidget* widget) {
  if (widget == nullptr) {
    return true;
  }
  // Viewport host and frosted sidebars (custom milky card paint) own chrome.
  return widget->objectName() ==
             QString::fromLatin1(AppThemeIds::kViewportHost) ||
         widget->objectName() == QStringLiteral("Displays/DisplayPanel") ||
         widget->objectName() == QStringLiteral("Views/ViewsPanel") ||
         widget->objectName() == QStringLiteral("Selection/SelectionPanel");
}

QString CompactGroupStyle() {
  return style::sheet(QStringLiteral("widget/groups"), style::tokens());
}

QString SegmentedToggleStyle() {
  return style::sheet(QStringLiteral("widget/groups"), style::tokens());
}

QString PanelFrostedFooterStyle() {
  return style::sheet(QStringLiteral("chrome/footer"));
}

QString PanelFooterStyle() { return PanelFrostedFooterStyle(); }

QString PanelFrostedTreeStyle() {
  return style::sheet(QStringLiteral("chrome/tree"));
}

QString PanelFrostedListStyle() {
  return style::sheet(QStringLiteral("chrome/list"));
}

QString PanelFrostedHelpStyle() {
  return style::sheet(QStringLiteral("chrome/help"));
}

QString PanelPrimaryButtonStyle() {
  return style::sheet(QStringLiteral("widget/primary_button"));
}

QString PanelGhostButtonStyle() {
  return style::sheet(QStringLiteral("widget/ghost_button"));
}

QString PanelStatusBarStyle() {
  return style::sheet(QStringLiteral("chrome/status_bar"));
}

QString PanelStatusLabelStyle() {
  return style::type(style::Role::Accent, 11, 600);
}

QString PanelStatusLabelErrorStyle() {
  return style::type(style::Role::Danger, 12, 700);
}

QString PanelHintLabelStyle() { return style::type(style::Role::Muted, 11); }

QString PanelFilterLineEditStyle() {
  return style::sheet(QStringLiteral("widget/fields"), style::tokens());
}

QString PanelIconClearButtonStyle() {
  return style::sheet(QStringLiteral("widget/icon_clear_button"),
                      style::tokens());
}

QString PanelCompactButtonStyle() {
  return style::sheet(QStringLiteral("widget/compact_button"));
}

QString PanelHelpBrowserStyle() {
  return style::sheet(QStringLiteral("chrome/help_browser"), style::tokens());
}

QString PanelTreeWidgetStyle() { return PanelFrostedTreeStyle(); }

QString PanelSplitterStyle() {
  return style::sheet(QStringLiteral("chrome/layout"), style::tokens());
}

QString DockTitleBarStyle() {
  return style::sheet(QStringLiteral("chrome/dock_title_bar"));
}

QString DockTitleLabelStyle() {
  return style::sheet(QStringLiteral("chrome/dock_title_label"),
                      style::tokens());
}

QString MainWindowStatusBarStyle() {
  return style::sheet(QStringLiteral("chrome/window_status_bar"),
                      style::tokens());
}

QString PropertyInspectorTitleStyle() {
  // Horizontal inset comes from layout (PanelSettingsLayout::kOuterMargin),
  // matching the Properties dock icon column — do not add QSS padding here.
  return style::type(style::Role::Body, 12, 700) +
         QStringLiteral(" padding: 0;");
}

QString PropertyInspectorHintStyle() {
  return style::type(style::Role::Muted, 11) + QStringLiteral(" padding: 0;");
}

void ApplyPanelShell(QWidget* widget) {
  if (widget == nullptr || ShouldSkipPanelShellBackground(widget)) {
    return;
  }
  if (widget->objectName().isEmpty()) {
    widget->setObjectName(QString::fromLatin1(AppThemeIds::kPanelContent));
  }
  widget->setAutoFillBackground(false);
  widget->setAttribute(Qt::WA_StyledBackground, true);
  widget->setStyleSheet(PanelShellStyle(widget->objectName()));
}

void ApplyCompactSettingsShell(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
  widget->setAutoFillBackground(false);
  widget->setAttribute(Qt::WA_StyledBackground, true);
  widget->setStyleSheet(SettingsWidgetBackgroundStyle());
  // Defer polish until after the caller's constructor finishes building fields.
  QTimer::singleShot(0, widget, [widget]() { PolishSettingsFields(widget); });
  if (widget->property("autoviz_settings_polish_filter").isNull()) {
    widget->setProperty("autoviz_settings_polish_filter", true);
    widget->installEventFilter(new SettingsPolishFilter(widget));
  }
}

bool ShouldSkipSettingsPolish(const QWidget* field) {
  if (field == nullptr) {
    return true;
  }
  if (field->property("autoviz_skip_settings_polish").toBool()) {
    return true;
  }
  // Internal editors owned by combo/spin.
  if (qobject_cast<const QComboBox*>(field->parentWidget()) != nullptr ||
      qobject_cast<const QAbstractSpinBox*>(field->parentWidget()) != nullptr) {
    return true;
  }
  // Custom instrument cards (Teleop / Image speed chips).
  for (QWidget* p = field->parentWidget(); p != nullptr;
       p = p->parentWidget()) {
    const QString name = p->objectName();
    if (name == QLatin1String("TeleopSettingsSpeedCard") ||
        name == QLatin1String("TeleopSpeedCard") ||
        name == QLatin1String("ImageRangeCard")) {
      return true;
    }
  }
  return false;
}

void StyleCompactSettingsField(QWidget* field) {
  if (ShouldSkipSettingsPolish(field)) {
    return;
  }
  constexpr int kH = 20;
  field->setFixedHeight(kH);
  field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHash<QString, QString> tokens = style::tokens();
  tokens.insert(QStringLiteral("{{h}}"), QString::number(kH));

  const QString fields = style::sheet(QStringLiteral("widget/fields"), tokens);

  if (auto* spin = qobject_cast<QAbstractSpinBox*>(field)) {
    spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spin->setStyleSheet(fields);
    return;
  }

  if (auto* combo = qobject_cast<QComboBox*>(field)) {
    combo->setObjectName(QStringLiteral("AutovizSettingsCombo"));
    combo->setStyleSheet(fields);
    if (combo->lineEdit() != nullptr) {
      combo->lineEdit()->setFixedHeight(18);
    }
    return;
  }

  if (auto* edit = qobject_cast<QLineEdit*>(field)) {
    edit->setObjectName(QStringLiteral("AutovizSettingsLineEdit"));
    edit->setStyleSheet(fields);
  }
}

void PolishSettingsFields(QWidget* root) {
  if (root == nullptr) {
    return;
  }
  for (QLineEdit* edit : root->findChildren<QLineEdit*>()) {
    StyleCompactSettingsField(edit);
  }
  for (QComboBox* combo : root->findChildren<QComboBox*>()) {
    StyleCompactSettingsField(combo);
  }
  for (QAbstractSpinBox* spin : root->findChildren<QAbstractSpinBox*>()) {
    StyleCompactSettingsField(spin);
  }
  // Soft form labels (skip hints / section titles that already have styles).
  const QString form_label = style::type(style::Role::Muted, 12, 500);
  for (QLabel* label : root->findChildren<QLabel*>()) {
    if (label == nullptr || !label->styleSheet().isEmpty()) {
      continue;
    }
    if (!label->objectName().isEmpty()) {
      continue;
    }
    if (label->wordWrap() || label->text().size() > 40) {
      continue;
    }
    label->setStyleSheet(form_label);
  }
}

QLabel* MakeSettingsFormLabel(const QString& text, QWidget* parent) {
  auto* label = new QLabel(text, parent);
  label->setStyleSheet(style::type(style::Role::Muted, 12, 500));
  return label;
}

void ApplyCompactForm(QFormLayout* form) {
  if (form == nullptr) {
    return;
  }
  form->setContentsMargins(0, 0, 0, 0);
  form->setHorizontalSpacing(8);
  form->setVerticalSpacing(3);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
}

void ApplyCompactVBox(QVBoxLayout* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->setContentsMargins(
      PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
      PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  layout->setSpacing(PanelSettingsLayout::kOuterSpacing);
  layout->setAlignment(Qt::AlignTop);
}

void ApplyPanelToolbarLayout(QHBoxLayout* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->setContentsMargins(
      PanelChromeLayout::kToolbarMarginH, PanelChromeLayout::kToolbarMarginV,
      PanelChromeLayout::kToolbarMarginH, PanelChromeLayout::kToolbarMarginV);
  layout->setSpacing(PanelChromeLayout::kToolbarSpacing);
}

void StylePanelStatusLabel(QLabel* label, bool is_error) {
  if (label == nullptr) {
    return;
  }
  label->setStyleSheet(is_error ? PanelStatusLabelErrorStyle()
                                : PanelStatusLabelStyle());
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
  label->setStyleSheet(style::type(style::Role::Body, 13, 700));
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
  edit->setObjectName(QStringLiteral("AutovizFilterLineEdit"));
  edit->setStyleSheet(PanelFilterLineEditStyle());
}

void DetachItemViewFromStyleSheets(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
  // Drop widget-level ::item/::branch sheets. App theme likewise omits generic
  // QTreeWidget::item rules — together this avoids Qt 6.2 QCss::Selector
  // crashes while Displays trees rebuild under live Argus traffic.
  // Note: QWidget::setStyle(Fusion) still wraps in QStyleSheetStyle when the
  // app has a stylesheet, so it is not a real bypass; do not use it here.
  widget->setStyleSheet(QString());
  if (auto* view = qobject_cast<QAbstractItemView*>(widget)) {
    if (QWidget* viewport = view->viewport()) {
      viewport->setStyleSheet(QString());
    }
  }
}

void StylePanelTree(QWidget* tree) {
  if (auto* view = qobject_cast<QAbstractItemView*>(tree)) {
    StyleFrostedPanelTree(view, 0);
    return;
  }
  if (tree == nullptr) {
    return;
  }
  tree->setObjectName(QString::fromLatin1(AppThemeIds::kPanelTree));
  DetachItemViewFromStyleSheets(tree);
  ApplyItemViewShellPalette(tree);
}

void StyleFrostedPanelTree(QAbstractItemView* tree, int name_column) {
  if (tree == nullptr) {
    return;
  }
  tree->setObjectName(QString::fromLatin1(AppThemeIds::kPanelTree));
  tree->setAttribute(Qt::WA_TranslucentBackground, true);
  tree->setAutoFillBackground(false);
  tree->setAttribute(Qt::WA_MacShowFocusRect, false);
  if (tree->viewport() != nullptr) {
    tree->viewport()->setAutoFillBackground(false);
    tree->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
  }
  DetachItemViewFromStyleSheets(tree);
  ApplyItemViewShellPalette(tree);
  tree->setItemDelegateForColumn(
      name_column, new ContinuousNameColumnDelegate(name_column, tree));
}

void StyleFrostedPanelList(QAbstractItemView* list) {
  if (list == nullptr) {
    return;
  }
  list->setAttribute(Qt::WA_TranslucentBackground, true);
  list->setAutoFillBackground(false);
  list->setAttribute(Qt::WA_MacShowFocusRect, false);
  if (list->viewport() != nullptr) {
    list->viewport()->setAutoFillBackground(false);
    list->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
  }
  DetachItemViewFromStyleSheets(list);
  ApplyItemViewShellPalette(list);
  list->setItemDelegate(new ContinuousNameColumnDelegate(0, list));
}

void PaintPanelFrostedCard(QPainter& painter, const QRect& rect, qreal radius) {
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF outer = QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath card;
  card.addRoundedRect(outer, radius, radius);

  const glass::ShellTokens s = glass::Shell();
  painter.fillPath(card, s.glass_fill_deep);
  painter.setPen(QPen(s.glass_rim, 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(card);
}

void ApplyPanelToolbarChrome(QFrame* toolbar) {
  if (toolbar == nullptr) {
    return;
  }
  toolbar->setObjectName(QString::fromLatin1(AppThemeIds::kPanelToolbar));
  toolbar->setAttribute(Qt::WA_StyledBackground, true);
  toolbar->setStyleSheet(
      style::sheet(QStringLiteral("chrome/toolbar"), style::tokens()));
}

void ApplyPanelFooterChrome(QFrame* footer) {
  if (footer == nullptr) {
    return;
  }
  footer->setObjectName(QString::fromLatin1(AppThemeIds::kPanelFooter));
  footer->setAttribute(Qt::WA_StyledBackground, true);
  footer->setStyleSheet(
      style::sheet(QStringLiteral("chrome/footer_frame"), style::tokens()));
}

void ApplyPanelTitleToolsChrome(QWidget* tools) {
  if (tools == nullptr) {
    return;
  }
  tools->setObjectName(QString::fromLatin1(AppThemeIds::kPanelTitleTools));
  tools->setStyleSheet(
      style::sheet(QStringLiteral("chrome/title_tools"), style::tokens()));
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
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(PanelGhostButtonStyle());
  return button;
}

QPushButton* MakePrimaryActionButton(const QString& text, QWidget* parent) {
  auto* button = new QPushButton(text, parent);
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(PanelPrimaryButtonStyle());
  return button;
}

QPushButton* MakeDestructiveFlatActionButton(const QString& text,
                                             QWidget* parent) {
  auto* button = new QPushButton(text, parent);
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(
      style::sheet(QStringLiteral("widget/destructive_button")));
  return button;
}

void UpdateColorButton(QPushButton* button, const QColor& color) {
  if (button == nullptr) {
    return;
  }
  button->setFixedHeight(20);
  button->setMaximumWidth(96);
  if (!color.isValid()) {
    button->setText(QObject::tr("Pick"));
    button->setStyleSheet(
        style::sheet(QStringLiteral("widget/color_button_empty")));
    return;
  }
  button->setText(color.name(QColor::HexRgb));
  QHash<QString, QString> tokens;
  tokens.insert(QStringLiteral("{{color}}"), color.name(QColor::HexRgb));
  button->setStyleSheet(
      style::sheet(QStringLiteral("widget/color_button"), tokens));
}

QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title,
                                QWidget* body, bool expanded) {
  auto* section = new widget::CollapsiblePanel(parent);
  section->SetTone(style::Tone::Frost);
  // Soft expand/collapse for Liquid Glass settings panels.
  section->SetAnimationEnabled(true);
  section->SetTitle(title);
  section->SetContentWidget(body);
  section->SetExpanded(expanded);
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
