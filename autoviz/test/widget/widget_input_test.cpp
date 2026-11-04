/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAbstractButton>
#include <QFont>
#include <QFrame>
#include <QImage>
#include <QLineEdit>
#include <QListView>
#include <QSignalSpy>
#include <QStringListModel>
#include <QTest>
#include <QVector3D>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/auto_suggest_box.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/form_controller.hpp"
#include "autoviz/common/widget/input/number_field.hpp"
#include "autoviz/common/widget/input/password_box.hpp"
#include "autoviz/common/widget/input/search_box.hpp"
#include "autoviz/common/widget/input/segmented_control.hpp"
#include "autoviz/common/widget/input/toggle_switch.hpp"
#include "autoviz/common/widget/input/vector3_editor.hpp"

namespace autoviz::widget {
namespace {

QImage RenderControl(QWidget* control) {
  control->ensurePolished();
  control->resize(control->sizeHint().expandedTo(QSize(220, 36)));
  QImage image(control->size(), QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  control->render(&image);
  return image;
}

class WidgetInputTest final : public QObject {
  Q_OBJECT

 private slots:
  void SearchAndPasswordFieldsKeepNativeSemantics();
  void SearchBoxSizesFollowTokensAndFonts();
  void CustomFieldsExposeDesktopInteractionStates();
  void CustomFieldsKeepRoundedMaterialsAfterRecursiveTheme();
  void ValidationErrorsRenderAndClear();
  void SegmentedControlMaintainsSelectionAcrossChanges();
  void SegmentedControlSupportsKeyboardActivation();
  void SegmentedControlUsesThemeTokens();
  void ToggleSwitchSettlesWhenHidden();
  void VectorEditorEmitsOneCoherentValue();
  void AutoSuggestDebouncesAndActivatesRows();
  void AutoSuggestActivatesRowsInsertedAsynchronously();
  void AutoSuggestDisconnectsOwnedModelBeforeChildDestruction();
  void ButtonVariantsRemainUsable();
};

void WidgetInputTest::SearchAndPasswordFieldsKeepNativeSemantics() {
  SearchBox search;
  QSignalSpy request_spy(&search, &SearchBox::SearchRequested);
  search.setText(QStringLiteral("  map frame  "));
  QTest::keyClick(&search, Qt::Key_Return);
  QCOMPARE(request_spy.count(), 1);
  QCOMPARE(request_spy.at(0).at(0).toString(), QStringLiteral("map frame"));

  QTest::keyClick(&search, Qt::Key_Escape);
  QVERIFY(search.text().isEmpty());

  PasswordBox password;
  QCOMPARE(password.echoMode(), QLineEdit::Password);
  password.SetPasswordVisible(true);
  QVERIFY(password.is_password_visible());
  QCOMPARE(password.echoMode(), QLineEdit::Normal);
  password.SetPasswordVisible(false);
  QCOMPARE(password.echoMode(), QLineEdit::Password);
}

void WidgetInputTest::SearchBoxSizesFollowTokensAndFonts() {
  SearchBox search;
  const ThemeTokens& tokens = GetThemeTokens();
  QCOMPARE(search.search_box_size(), SearchBox::Size::kMedium);

  search.SetSearchBoxSize(SearchBox::Size::kCompact);
  QCOMPARE(search.search_box_size(), SearchBox::Size::kCompact);
  const int compact_height =
      qMax(tokens.control_height_compact, search.fontMetrics().height() + 2);
  QCOMPARE(search.sizeHint().height(), compact_height);
  QCOMPARE(search.minimumSizeHint().height(), compact_height);

  search.SetSearchBoxSize(SearchBox::Size::kSmall);
  const int small_height =
      qMax(tokens.control_height_small, search.fontMetrics().height() + 2);
  QCOMPARE(search.sizeHint().height(), small_height);

  search.SetSearchBoxSize(SearchBox::Size::kMedium);
  const int medium_height =
      qMax(tokens.control_height_medium, search.fontMetrics().height() + 2);
  QCOMPARE(search.sizeHint().height(), medium_height);
  const QSize medium_size = search.sizeHint();
  search.SetSearchBoxSize(SearchBox::Size::kMedium);
  QCOMPARE(search.sizeHint(), medium_size);

  search.SetSearchBoxSize(SearchBox::Size::kLarge);
  const int large_height =
      qMax(tokens.control_height_large, search.fontMetrics().height() + 2);
  QCOMPARE(search.sizeHint().height(), large_height);
  QVERIFY(compact_height <= small_height);
  QVERIFY(small_height <= medium_height);
  QVERIFY(medium_height <= large_height);

  QFont large_font = search.font();
  large_font.setPointSizeF(qMax(large_font.pointSizeF(), 12.0) + 12.0);
  search.setFont(large_font);
  search.SetSearchBoxSize(SearchBox::Size::kCompact);
  QVERIFY(search.sizeHint().height() >= search.fontMetrics().height() + 2);
  QCOMPARE(search.minimumSizeHint().height(), search.sizeHint().height());
}

void WidgetInputTest::CustomFieldsExposeDesktopInteractionStates() {
  SearchBox search;
  search.SetSearchBoxSize(SearchBox::Size::kCompact);
  search.resize(search.sizeHint());
  QVERIFY(search.testAttribute(Qt::WA_Hover));
  QCOMPARE(search.focusPolicy(), Qt::StrongFocus);
  QVERIFY(search.minimumSizeHint().height() >=
          GetThemeTokens().control_height_compact);

  PasswordBox password;
  password.resize(password.sizeHint());
  QVERIFY(password.testAttribute(Qt::WA_Hover));
  QCOMPARE(password.focusPolicy(), Qt::StrongFocus);
  QVERIFY(password.sizeHint().height() >=
          GetThemeTokens().control_height_small);

  QImage enabled(password.size(), QImage::Format_ARGB32_Premultiplied);
  enabled.fill(Qt::transparent);
  password.render(&enabled);
  password.setEnabled(false);
  QImage disabled(password.size(), QImage::Format_ARGB32_Premultiplied);
  disabled.fill(Qt::transparent);
  password.render(&disabled);
  QVERIFY(enabled != disabled);
}

void WidgetInputTest::CustomFieldsKeepRoundedMaterialsAfterRecursiveTheme() {
  QWidget host;
  SearchBox search(&host);
  PasswordBox password(&host);
  NumberField number(&host);
  search.setText(QStringLiteral("map"));
  password.setText(QStringLiteral("secret"));
  number.setRange(0.01, 1.0);
  number.setValue(0.05);

  ApplyWidgetTheme(&host, style::Tone::Shell, true, ThemeAppearance::kLight);
  for (const QWidget* field :
       {static_cast<QWidget*>(&search), static_cast<QWidget*>(&password),
        static_cast<QWidget*>(&number)}) {
    QCOMPARE(field->palette().color(QPalette::Base).alpha(), 0);
  }
  QCOMPARE(number.buttonSymbols(), QAbstractSpinBox::NoButtons);
  QVERIFY(!number.hasFrame());
  QLineEdit* number_editor = number.findChild<QLineEdit*>();
  QVERIFY(number_editor != nullptr);
  QCOMPARE(number_editor->palette().color(QPalette::Base).alpha(), 0);

  const QImage light_search = RenderControl(&search);
  const QImage light_password = RenderControl(&password);
  const QImage light_number = RenderControl(&number);
  for (const QImage* image : {&light_search, &light_password, &light_number}) {
    QVERIFY(!image->isNull());
    const QColor corner = image->pixelColor(0, 0);
    const QColor center =
        image->pixelColor(image->width() / 2, image->height() / 2);
    QVERIFY(corner.alpha() < center.alpha());
  }

  ApplyWidgetTheme(&host, style::Tone::Shell, true, ThemeAppearance::kDark);
  QCOMPARE(search.palette().color(QPalette::Base).alpha(), 0);
  QCOMPARE(password.palette().color(QPalette::Base).alpha(), 0);
  QCOMPARE(number.palette().color(QPalette::Base).alpha(), 0);
  QCOMPARE(number_editor->palette().color(QPalette::Base).alpha(), 0);
  QVERIFY(RenderControl(&search) != light_search);
  QVERIFY(RenderControl(&password) != light_password);
  QVERIFY(RenderControl(&number) != light_number);
}

void WidgetInputTest::ValidationErrorsRenderAndClear() {
  SearchBox search;
  PasswordBox password;
  NumberField number;
  password.setText(QStringLiteral("secret"));
  number.setRange(0.01, 1.0);
  number.setValue(0.05);

  const QImage normal_search = RenderControl(&search);
  const QImage normal_password = RenderControl(&password);
  const QImage normal_number = RenderControl(&number);

  FormController form;
  form.RegisterField(&search);
  form.RegisterField(&password);
  form.RegisterField(&number);
  form.SetFieldError(&search, QStringLiteral("Unknown frame"));
  form.SetFieldError(&password, QStringLiteral("Token expired"));
  form.SetFieldError(&number, QStringLiteral("Value is out of range"));

  QVERIFY(search.property("autovizValidationError").toBool());
  QVERIFY(password.property("autovizValidationError").toBool());
  QVERIFY(number.property("autovizValidationError").toBool());
  QVERIFY(RenderControl(&search) != normal_search);
  QVERIFY(RenderControl(&password) != normal_password);
  QVERIFY(RenderControl(&number) != normal_number);

  form.ClearFieldError(&search);
  form.ClearFieldError(&password);
  form.ClearFieldError(&number);
  QVERIFY(!search.property("autovizValidationError").toBool());
  QVERIFY(!password.property("autovizValidationError").toBool());
  QVERIFY(!number.property("autovizValidationError").toBool());
  QVERIFY(RenderControl(&search) == normal_search);
  QVERIFY(RenderControl(&password) == normal_password);
  QVERIFY(RenderControl(&number) == normal_number);
}

void WidgetInputTest::SegmentedControlMaintainsSelectionAcrossChanges() {
  SegmentedControl control;
  QSignalSpy index_spy(&control, &SegmentedControl::CurrentIndexChanged);

  control.AddSegment(QStringLiteral("First"), 1);
  control.AddSegment(QStringLiteral("Second"), 2);
  QCOMPARE(control.current_index(), 0);
  QCOMPARE(control.current_data().toInt(), 1);

  control.SetCurrentIndex(1);
  QCOMPARE(control.current_data().toInt(), 2);
  control.InsertSegment(0, QStringLiteral("Inserted"), 0);
  QCOMPARE(control.current_index(), 2);
  QCOMPARE(control.current_data().toInt(), 2);

  control.RemoveSegment(2);
  QCOMPARE(control.current_index(), 1);
  QCOMPARE(control.current_data().toInt(), 1);
  control.Clear();
  QCOMPARE(control.current_index(), -1);
  QVERIFY(!index_spy.isEmpty());
}

void WidgetInputTest::SegmentedControlSupportsKeyboardActivation() {
  SegmentedControl control;
  control.AddSegment(QStringLiteral("Map"), 1);
  control.AddSegment(QStringLiteral("Graph"), 2);
  control.show();
  QCoreApplication::processEvents();

  const QList<QAbstractButton*> buttons =
      control.findChildren<QAbstractButton*>();
  QCOMPARE(buttons.size(), 2);
  QSignalSpy clicked_spy(&control, &SegmentedControl::SegmentClicked);
  QSignalSpy index_spy(&control, &SegmentedControl::CurrentIndexChanged);

  buttons.at(1)->setFocus();
  QTRY_VERIFY(buttons.at(1)->hasFocus());
  QTest::keyClick(buttons.at(1), Qt::Key_Space);
  QCOMPARE(control.current_index(), 1);
  QCOMPARE(control.current_data().toInt(), 2);
  QCOMPARE(clicked_spy.count(), 1);
  QCOMPARE(index_spy.count(), 1);

  control.SetSegmentEnabled(0, false);
  QVERIFY(!control.is_segment_enabled(0));
  buttons.at(0)->click();
  QCOMPARE(control.current_index(), 1);
  QCOMPARE(clicked_spy.count(), 1);
}

void WidgetInputTest::SegmentedControlUsesThemeTokens() {
  SegmentedControl control;
  QCOMPARE(control.property("autovizRole").toString(),
           QStringLiteral("segmentedControl"));
  QCOMPARE(control.tone(), style::Tone::Shell);
  control.AddSegment(QStringLiteral("Map"));
  control.AddSegment(QStringLiteral("Graph"));

  const QList<QAbstractButton*> buttons =
      control.findChildren<QAbstractButton*>(QString(),
                                             Qt::FindDirectChildrenOnly);
  QCOMPARE(buttons.size(), 2);
  const ThemeTokens& shell_tokens = GetThemeTokens(style::Tone::Shell);
  for (const QAbstractButton* button : buttons) {
    QCOMPARE(button->property("autovizRole").toString(),
             QStringLiteral("segment"));
    QCOMPARE(button->sizeHint().height(),
             qMax(shell_tokens.control_height_small,
                  button->fontMetrics().height() + shell_tokens.spacing_sm));
  }

  control.resize(control.sizeHint());
  control.show();
  QCoreApplication::processEvents();
  QImage shell_image(control.size(), QImage::Format_ARGB32_Premultiplied);
  shell_image.fill(Qt::transparent);
  control.render(&shell_image);

  control.SetTone(style::Tone::Frost);
  QCOMPARE(control.tone(), style::Tone::Frost);
  QCoreApplication::processEvents();
  QImage frost_image(control.size(), QImage::Format_ARGB32_Premultiplied);
  frost_image.fill(Qt::transparent);
  control.render(&frost_image);
  QVERIFY(shell_image != frost_image);
}

void WidgetInputTest::ToggleSwitchSettlesWhenHidden() {
  ToggleSwitch toggle;
  QSignalSpy toggled_spy(&toggle, &QAbstractButton::toggled);
  toggle.show();
  toggle.setFocus();
  QTRY_VERIFY(toggle.hasFocus());
  QTest::keyClick(&toggle, Qt::Key_Space);
  QCOMPARE(toggled_spy.count(), 1);
  QVERIFY(toggle.isChecked());
  toggle.hide();
  QCOMPARE(toggle.thumbPosition(), qreal{1.0});

  toggle.show();
  toggle.setChecked(false);
  toggle.setEnabled(false);
  QCOMPARE(toggle.thumbPosition(), qreal{0.0});
}

void WidgetInputTest::VectorEditorEmitsOneCoherentValue() {
  qRegisterMetaType<QVector3D>();
  Vector3Editor editor;
  QSignalSpy value_spy(&editor, &Vector3Editor::ValueChanged);

  editor.SetValue(QVector3D(1.0F, 2.0F, 3.0F));
  QCOMPARE(value_spy.count(), 1);
  QCOMPARE(editor.value(), QVector3D(1.0F, 2.0F, 3.0F));

  editor.SetRange(-1.0, 1.0);
  QCOMPARE(value_spy.count(), 2);
  QCOMPARE(editor.value(), QVector3D(1.0F, 1.0F, 1.0F));
  editor.SetValue(editor.value());
  QCOMPARE(value_spy.count(), 2);
}

void WidgetInputTest::AutoSuggestDebouncesAndActivatesRows() {
  AutoSuggestBox box;
  box.SetDebounceMs(10);
  box.resize(240, 32);
  box.show();
  box.setFocus();
  QSignalSpy query_spy(&box, &AutoSuggestBox::QueryRequested);

  QTest::keyClicks(&box, QStringLiteral("nav"));
  QTRY_COMPARE(query_spy.count(), 1);
  QCOMPARE(query_spy.at(0).at(0).toString(), QStringLiteral("nav"));

  query_spy.clear();
  box.selectAll();
  QTest::keyClicks(&box, QStringLiteral("pose"));
  QTest::keyClick(&box, Qt::Key_Return);
  QCOMPARE(query_spy.count(), 1);
  QCOMPARE(query_spy.at(0).at(0).toString(), QStringLiteral("pose"));
  QTest::qWait(20);
  QCOMPARE(query_spy.count(), 1);

  QStringListModel model({QStringLiteral("Alpha"), QStringLiteral("Beta")});
  box.SetModel(&model);
  auto* popup =
      box.findChild<QFrame*>(QStringLiteral("AutovizAutoSuggestPopup"));
  auto* list =
      box.findChild<QListView*>(QStringLiteral("AutovizAutoSuggestList"));
  QVERIFY(popup != nullptr);
  QVERIFY(list != nullptr);
  QCOMPARE(popup->frameShape(), QFrame::NoFrame);
  QVERIFY(popup->testAttribute(Qt::WA_TranslucentBackground));
  QCOMPARE(list->frameShape(), QFrame::NoFrame);
  QVERIFY(!list->autoFillBackground());
  QVERIFY(list->viewport()->testAttribute(Qt::WA_TranslucentBackground));
  QVERIFY(list->sizeHintForRow(0) >=
          GetThemeTokens(&box).control_height_medium);

  box.setText(QStringLiteral("a"));
  QSignalSpy activation_spy(&box, &AutoSuggestBox::SuggestionActivated);
  QTest::keyClick(&box, Qt::Key_Down);
  QTest::keyClick(&box, Qt::Key_Return);
  QCOMPARE(activation_spy.count(), 1);
  QCOMPARE(box.text(), QStringLiteral("Beta"));

  auto* temporary_model = new QStringListModel({QStringLiteral("Temporary")});
  box.SetModel(temporary_model);
  delete temporary_model;
  QCOMPARE(box.model(), nullptr);
}

void WidgetInputTest::AutoSuggestActivatesRowsInsertedAsynchronously() {
  AutoSuggestBox box;
  QStringListModel model;
  box.SetModel(&model);
  box.resize(240, 32);
  box.show();
  box.setFocus();
  QTRY_VERIFY(box.hasFocus());
  box.setText(QStringLiteral("ro"));

  QSignalSpy activation_spy(&box, &AutoSuggestBox::SuggestionActivated);
  QVERIFY(model.insertRow(0));
  const QModelIndex first = model.index(0, 0);
  QVERIFY(model.setData(first, QStringLiteral("Robot"), Qt::EditRole));
  QCoreApplication::processEvents();

  QTest::keyClick(&box, Qt::Key_Return);
  QCOMPARE(activation_spy.count(), 1);
  QCOMPARE(activation_spy.at(0).at(0).toModelIndex(), first);
  QCOMPARE(box.text(), QStringLiteral("Robot"));

  auto* replacement = new QStringListModel({QStringLiteral("Map")});
  box.SetModel(replacement);
  delete replacement;
  QCOMPARE(box.model(), nullptr);
  QTest::keyClick(&box, Qt::Key_Down);
  QTest::keyClick(&box, Qt::Key_Return);
  QCOMPARE(activation_spy.count(), 1);
}

void WidgetInputTest::AutoSuggestDisconnectsOwnedModelBeforeChildDestruction() {
  auto* box = new AutoSuggestBox;
  auto* owned_model =
      new QStringListModel({QStringLiteral("Owned suggestion")}, box);
  box->SetModel(owned_model);
  QCOMPARE(box->model(), owned_model);

  // The model is a younger QObject child than the popup. Destruction must
  // disconnect its destroyed signal before the popup child is deleted.
  delete box;
}

void WidgetInputTest::ButtonVariantsRemainUsable() {
  Button button(QStringLiteral("Apply"));
  button.SetVariant(Button::Variant::kPrimary);
  button.SetButtonSize(Button::Size::kLarge);
  QCOMPARE(button.variant(), Button::Variant::kPrimary);
  QCOMPARE(button.button_size(), Button::Size::kLarge);
  QVERIFY(button.sizeHint().width() >= button.minimumSizeHint().width());
  QCOMPARE(button.focusPolicy(), Qt::StrongFocus);
  QVERIFY(!button.text().isEmpty());
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetInputTest)

#include "widget_input_test.moc"
