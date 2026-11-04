/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QImage>
#include <QKeySequence>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPalette>
#include <QPushButton>
#include <QSignalSpy>
#include <QStringListModel>
#include <QTest>
#include <QToolButton>
#include <algorithm>
#include <array>
#include <cmath>

#include "autoviz/common/widget/data/list_detail_view.hpp"
#include "autoviz/common/widget/feedback/info_bar.hpp"
#include "autoviz/common/widget/feedback/toast.hpp"
#include "autoviz/common/widget/foundation/theme_tokens.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/form_controller.hpp"
#include "autoviz/common/widget/input/password_box.hpp"
#include "autoviz/common/widget/input/shortcut_recorder.hpp"
#include "autoviz/common/widget/navigation/command_bar.hpp"
#include "autoviz/common/widget/navigation/navigation_rail.hpp"
#include "autoviz/common/widget/navigation/pips_pager.hpp"
#include "autoviz/common/widget/overlay/popover.hpp"

namespace autoviz::widget {
namespace {

double RelativeLuminance(const QColor& color) {
  auto linearize = [](double channel) {
    channel /= 255.0;
    return channel <= 0.04045 ? channel / 12.92
                              : std::pow((channel + 0.055) / 1.055, 2.4);
  };
  return 0.2126 * linearize(color.red()) + 0.7152 * linearize(color.green()) +
         0.0722 * linearize(color.blue());
}

double ContrastRatio(const QColor& first, const QColor& second) {
  const double lighter =
      std::max(RelativeLuminance(first), RelativeLuminance(second));
  const double darker =
      std::min(RelativeLuminance(first), RelativeLuminance(second));
  return (lighter + 0.05) / (darker + 0.05);
}

double MaximumIconContrast(const QIcon& icon, const QColor& background) {
  QImage image(24, 24, QImage::Format_ARGB32_Premultiplied);
  image.fill(background);
  QPainter painter(&image);
  icon.paint(&painter, QRect(3, 3, 18, 18));
  painter.end();

  double maximum = 1.0;
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      maximum =
          std::max(maximum, ContrastRatio(image.pixelColor(x, y), background));
    }
  }
  return maximum;
}

class WidgetCompositeTest final : public QObject {
  Q_OBJECT

 private slots:
  void CommandBarUsesNativeActionsAndDisplayModes();
  void PipsPagerSupportsKeyboardNavigationAndActivation();
  void ShortcutRecorderReportsReservedSequences();
  void FormControllerTracksValidationAndDirtyState();
  void ListDetailViewUsesOneModelBackedList();
  void ButtonVariantsShareSizingContract();
  void ChromeRendersSemanticToneAndStates();
  void DarkChromeIconsMeetNonTextContrast();
  void FeedbackNotificationsExposeLevelsAndDisabledIcons();
};

void WidgetCompositeTest::CommandBarUsesNativeActionsAndDisplayModes() {
  CommandBar bar;
  QAction* open = bar.AddCommand(QStringLiteral("Open"));
  QVERIFY(open != nullptr);
  QCOMPARE(open->text(), QStringLiteral("Open"));
  QCOMPARE(bar.actions().size(), 1);

  auto* action_button = qobject_cast<QToolButton*>(bar.widgetForAction(open));
  QVERIFY(action_button != nullptr);
  const int minimum_hit_height = GetThemeTokens(&bar).control_height_medium;
  bar.SetDisplayMode(CommandBar::DisplayMode::kTextBesideIcon);
  QCOMPARE(action_button->toolButtonStyle(), Qt::ToolButtonTextBesideIcon);
  QVERIFY(action_button->minimumSize().width() >= minimum_hit_height);
  QVERIFY(action_button->minimumSize().height() >= minimum_hit_height);
  QSize effective_size =
      action_button->sizeHint().expandedTo(action_button->minimumSize());
  QVERIFY(effective_size.width() >= minimum_hit_height);
  QVERIFY(effective_size.height() >= minimum_hit_height);

  bar.SetDisplayMode(CommandBar::DisplayMode::kIconOnly);
  QCOMPARE(bar.toolButtonStyle(), Qt::ToolButtonIconOnly);
  QCOMPARE(action_button->toolButtonStyle(), Qt::ToolButtonIconOnly);
  QVERIFY(action_button->minimumSize().width() >= minimum_hit_height);
  QVERIFY(action_button->minimumSize().height() >= minimum_hit_height);
  effective_size =
      action_button->sizeHint().expandedTo(action_button->minimumSize());
  QVERIFY(effective_size.width() >= minimum_hit_height);
  QVERIFY(effective_size.height() >= minimum_hit_height);
  bar.SetTone(style::Tone::Frost);
  QCOMPARE(bar.tone(), style::Tone::Frost);
}

void WidgetCompositeTest::PipsPagerSupportsKeyboardNavigationAndActivation() {
  PipsPager pager;
  QCOMPARE(pager.sizeHint().height(),
           GetThemeTokens(&pager).control_height_medium);
  pager.resize(240, pager.sizeHint().height());
  QSignalSpy page_count_spy(&pager, &PipsPager::CurrentPageChanged);
  pager.SetPageCount(12);
  QCOMPARE(pager.current_page(), 0);
  QCOMPARE(page_count_spy.count(), 1);
  pager.SetMaximumVisiblePips(5);
  pager.SetCurrentPage(4);
  pager.show();
  pager.setFocus();
  QCoreApplication::processEvents();

  QSignalSpy changed_spy(&pager, &PipsPager::CurrentPageChanged);
  QSignalSpy activated_spy(&pager, &PipsPager::PageActivated);
  QTest::keyClick(&pager, Qt::Key_Right);
  QCOMPARE(pager.current_page(), 5);
  QCOMPARE(changed_spy.count(), 1);
  QTest::keyClick(&pager, Qt::Key_Return);
  QCOMPARE(activated_spy.count(), 1);
  QCOMPARE(activated_spy.first().at(0).toInt(), 5);
  QVERIFY(pager.accessibleDescription().contains(QStringLiteral("6")));

  PipsPager compact_pager;
  compact_pager.SetPageCount(3);
  compact_pager.SetCurrentPage(2);
  compact_pager.resize(compact_pager.sizeHint());
  compact_pager.show();
  QCoreApplication::processEvents();
  QTest::mouseClick(&compact_pager, Qt::LeftButton, Qt::NoModifier,
                    QPoint(20, compact_pager.height() / 2));
  QCOMPARE(compact_pager.current_page(), 0);
}

void WidgetCompositeTest::ShortcutRecorderReportsReservedSequences() {
  ShortcutRecorder recorder;
  recorder.SetReservedSequences({QKeySequence(QStringLiteral("Ctrl+K"))});
  QSignalSpy validity_spy(&recorder, &ShortcutRecorder::ValidityChanged);

  recorder.SetKeySequence(QKeySequence(QStringLiteral("Ctrl+K")));
  QVERIFY(!recorder.is_valid());
  QVERIFY(recorder.editor()->accessibleDescription().contains(
      QStringLiteral("already"), Qt::CaseInsensitive));

  recorder.SetKeySequence(QKeySequence(QStringLiteral("Ctrl+L")));
  QVERIFY(recorder.is_valid());
  QCOMPARE(validity_spy.count(), 2);
}

void WidgetCompositeTest::FormControllerTracksValidationAndDirtyState() {
  QLineEdit name;
  QCheckBox terms;
  Button submit(QStringLiteral("Save"));
  FormController controller;
  controller.SetSubmitButton(&submit);
  controller.RegisterField(&name, true);
  controller.RegisterField(&terms, true);

  QVERIFY(!controller.is_valid());
  QVERIFY(!submit.isEnabled());
  QVERIFY(name.property("autovizValidationError").toBool());

  QSignalSpy dirty_spy(&controller, &FormController::DirtyChanged);
  name.setText(QStringLiteral("Atlas"));
  QVERIFY(!controller.is_valid());
  QVERIFY(!submit.isEnabled());
  terms.setChecked(true);
  QVERIFY(controller.is_valid());
  QVERIFY(controller.is_dirty());
  QVERIFY(submit.isEnabled());
  QCOMPARE(dirty_spy.count(), 1);

  controller.MarkClean();
  QVERIFY(!controller.is_dirty());
}

void WidgetCompositeTest::ListDetailViewUsesOneModelBackedList() {
  QStringListModel model({QStringLiteral("Map"), QStringLiteral("Robot")});
  ListDetailView view;
  auto* detail = new QLabel(QStringLiteral("Details"));
  view.SetDetailWidget(detail);
  view.SetModel(&model);
  QSignalSpy changed_spy(&view, &ListDetailView::CurrentIndexChanged);

  const QModelIndex robot = model.index(1, 0);
  view.SetCurrentIndex(robot);
  QCOMPARE(view.current_index(), robot);
  QCOMPARE(view.detail_widget(), detail);
  QCOMPARE(changed_spy.count(), 1);
  QVERIFY(view.accessibleDescription().contains(QStringLiteral("2")));

  QCOMPARE(view.TakeDetailWidget(), detail);
  delete detail;

  auto* ephemeral = new QLabel(QStringLiteral("Ephemeral"));
  view.SetDetailWidget(ephemeral);
  delete ephemeral;
  QCOMPARE(view.detail_widget(), nullptr);
  view.SetCurrentIndex(model.index(0, 0));
}

void WidgetCompositeTest::ButtonVariantsShareSizingContract() {
  Button button(QStringLiteral("Apply"));
  button.SetButtonSize(Button::Size::kSmall);
  const QSize small = button.sizeHint();
  button.SetButtonSize(Button::Size::kLarge);
  const QSize large = button.sizeHint();
  QVERIFY(large.height() > small.height());

  const QList<Button::Variant> variants = {
      Button::Variant::kPrimary, Button::Variant::kSecondary,
      Button::Variant::kSubtle, Button::Variant::kDanger};
  for (const Button::Variant variant : variants) {
    button.SetVariant(variant);
    QCOMPARE(button.variant(), variant);
    QVERIFY(button.sizeHint().isValid());
  }
}

void WidgetCompositeTest::ChromeRendersSemanticToneAndStates() {
  QWidget host;
  host.resize(640, 480);

  NavigationRail rail(&host);
  rail.SetItems({{QStringLiteral("Overview"), {}, 0, true},
                 {QStringLiteral("Sensors"), {}, 1, true}});
  rail.SetTone(style::Tone::Frost);
  rail.resize(220, 112);
  rail.show();
  rail.setFocus();
  QCoreApplication::processEvents();
  QCOMPARE(rail.findChildren<QToolButton*>().size(), 2);
  QVERIFY(!rail.grab().isNull());

  CommandBar commands(&host);
  commands.SetTone(style::Tone::Frost);
  commands.AddCommand(QStringLiteral("Refresh"));
  commands.resize(280, commands.sizeHint().height());
  commands.show();
  QCOMPARE(commands.tone(), style::Tone::Frost);
  QVERIFY(!commands.grab().isNull());

  InfoBar information(QStringLiteral("Connected"),
                      QStringLiteral("Receiving data"), FeedbackLevel::kSuccess,
                      &host);
  information.SetTone(style::Tone::Frost);
  information.resize(360, information.sizeHint().height());
  information.show();
  QVERIFY(!information.grab().isNull());

  Toast toast(&host);
  toast.SetMessage(QStringLiteral("Map saved"));
  toast.SetLevel(FeedbackLevel::kSuccess);
  toast.SetTone(style::Tone::Frost);
  toast.SetDurationMs(0);
  toast.resize(240, toast.sizeHint().height());
  QCOMPARE(toast.tone(), style::Tone::Frost);
  QVERIFY(!toast.grab().isNull());

  Popover popover(&host);
  popover.SetTone(style::Tone::Frost);
  popover.SetContentWidget(new QLabel(QStringLiteral("Details")));
  popover.resize(180, 80);
  QCOMPARE(popover.tone(), style::Tone::Frost);
  QVERIFY(!popover.grab().isNull());
}

void WidgetCompositeTest::DarkChromeIconsMeetNonTextContrast() {
  constexpr double kMinimumNonTextContrast = 3.0;

  InfoBar information;
  ApplyWidgetTheme(&information, style::Tone::Shell, true,
                   ThemeAppearance::kDark);
  auto* info_close = information.findChild<QPushButton*>(
      QStringLiteral("AutovizInfoBarCloseButton"));
  QVERIFY(info_close != nullptr);
  QVERIFY(MaximumIconContrast(info_close->icon(),
                              info_close->palette().color(QPalette::Button)) >=
          kMinimumNonTextContrast);

  Toast toast;
  ApplyWidgetTheme(&toast, style::Tone::Shell, true, ThemeAppearance::kDark);
  auto* toast_close =
      toast.findChild<QPushButton*>(QStringLiteral("AutovizToastCloseButton"));
  QVERIFY(toast_close != nullptr);
  QVERIFY(MaximumIconContrast(toast_close->icon(),
                              toast_close->palette().color(QPalette::Button)) >=
          kMinimumNonTextContrast);

  PasswordBox password;
  ApplyWidgetTheme(&password, style::Tone::Shell, true, ThemeAppearance::kDark);
  QVERIFY(!password.actions().isEmpty());
  QVERIFY(MaximumIconContrast(password.actions().constFirst()->icon(),
                              password.palette().color(QPalette::Base)) >=
          kMinimumNonTextContrast);
}

void WidgetCompositeTest::FeedbackNotificationsExposeLevelsAndDisabledIcons() {
  struct LevelExpectation {
    FeedbackLevel level;
    const char* accessible_name;
  };
  constexpr std::array<LevelExpectation, 4> kLevels = {{
      {FeedbackLevel::kInformation, "Information notification"},
      {FeedbackLevel::kSuccess, "Success notification"},
      {FeedbackLevel::kWarning, "Warning notification"},
      {FeedbackLevel::kError, "Error notification"},
  }};

  InfoBar information(QStringLiteral("Connection"),
                      QStringLiteral("Receiving data"));
  auto* info_icon =
      information.findChild<QLabel*>(QStringLiteral("AutovizInfoBarLevelIcon"));
  QVERIFY(info_icon != nullptr);

  Toast toast;
  toast.SetMessage(QStringLiteral("Map saved"));
  auto* toast_icon =
      toast.findChild<QLabel*>(QStringLiteral("AutovizToastLevelIcon"));
  QVERIFY(toast_icon != nullptr);

  for (const LevelExpectation& expectation : kLevels) {
    const QString expected_name =
        QString::fromLatin1(expectation.accessible_name);
    information.SetLevel(expectation.level);
    toast.SetLevel(expectation.level);
    QCOMPARE(info_icon->accessibleName(), expected_name);
    QCOMPARE(toast_icon->accessibleName(), expected_name);
    QVERIFY(information.accessibleDescription().startsWith(expected_name));
    QVERIFY(toast.accessibleDescription().startsWith(expected_name));
  }

  information.SetLevel(FeedbackLevel::kInformation);
  toast.SetLevel(FeedbackLevel::kInformation);
  const QImage enabled_info_icon =
      info_icon->pixmap(Qt::ReturnByValue).toImage();
  const QImage enabled_toast_icon =
      toast_icon->pixmap(Qt::ReturnByValue).toImage();
  QVERIFY(!enabled_info_icon.isNull());
  QVERIFY(!enabled_toast_icon.isNull());

  information.setEnabled(false);
  toast.setEnabled(false);
  const QImage disabled_info_icon =
      info_icon->pixmap(Qt::ReturnByValue).toImage();
  const QImage disabled_toast_icon =
      toast_icon->pixmap(Qt::ReturnByValue).toImage();
  QVERIFY(enabled_info_icon != disabled_info_icon);
  QVERIFY(enabled_toast_icon != disabled_toast_icon);

  information.setEnabled(true);
  toast.setEnabled(true);
  QCOMPARE(info_icon->pixmap(Qt::ReturnByValue).toImage(), enabled_info_icon);
  QCOMPARE(toast_icon->pixmap(Qt::ReturnByValue).toImage(), enabled_toast_icon);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetCompositeTest)
#include "widget_composite_test.moc"
