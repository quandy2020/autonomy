/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QKeySequence>
#include <QKeySequenceEdit>
#include <QLabel>
#include <QLineEdit>
#include <QSignalSpy>
#include <QStringListModel>
#include <QTest>

#include "autoviz/common/widget/data/list_detail_view.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/form_controller.hpp"
#include "autoviz/common/widget/input/shortcut_recorder.hpp"
#include "autoviz/common/widget/navigation/command_bar.hpp"
#include "autoviz/common/widget/navigation/pips_pager.hpp"

namespace autoviz::widget {
namespace {

class WidgetCompositeTest final : public QObject {
  Q_OBJECT

 private slots:
  void CommandBarUsesNativeActionsAndDisplayModes();
  void PipsPagerSupportsKeyboardNavigationAndActivation();
  void ShortcutRecorderReportsReservedSequences();
  void FormControllerTracksValidationAndDirtyState();
  void ListDetailViewUsesOneModelBackedList();
  void ButtonVariantsShareSizingContract();
};

void WidgetCompositeTest::CommandBarUsesNativeActionsAndDisplayModes() {
  CommandBar bar;
  QAction *open = bar.AddCommand(QStringLiteral("Open"));
  QVERIFY(open != nullptr);
  QCOMPARE(open->text(), QStringLiteral("Open"));
  QCOMPARE(bar.actions().size(), 1);

  bar.SetDisplayMode(CommandBar::DisplayMode::kIconOnly);
  QCOMPARE(bar.toolButtonStyle(), Qt::ToolButtonIconOnly);
  bar.SetTone(style::Tone::Frost);
  QCOMPARE(bar.tone(), style::Tone::Frost);
}

void WidgetCompositeTest::PipsPagerSupportsKeyboardNavigationAndActivation() {
  PipsPager pager;
  pager.resize(240, 32);
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
  auto *detail = new QLabel(QStringLiteral("Details"));
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

  auto *ephemeral = new QLabel(QStringLiteral("Ephemeral"));
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

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetCompositeTest)
#include "widget_composite_test.moc"
