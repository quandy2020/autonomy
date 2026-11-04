/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAction>
#include <QApplication>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QImage>
#include <QMimeData>
#include <QSignalSpy>
#include <QTest>
#include <QUrl>

#include "autoviz/common/widget/display/avatar.hpp"
#include "autoviz/common/widget/display/tag.hpp"
#include "autoviz/common/widget/feedback/progress_bar.hpp"
#include "autoviz/common/widget/input/color_picker.hpp"
#include "autoviz/common/widget/input/file_drop_zone.hpp"
#include "autoviz/common/widget/input/multi_select_combo_box.hpp"
#include "autoviz/common/widget/input/rating.hpp"
#include "autoviz/common/widget/input/tree_select.hpp"

namespace autoviz::widget {
namespace {

class WidgetExtendedComponentsTest final : public QObject {
  Q_OBJECT

 private slots:
  void ColorPickerNormalizesAlphaAndExposesValue();
  void FileDropZoneSeparatesAcceptedAndRejectedPaths();
  void MultiSelectEnforcesLimitAndReturnsData();
  void TreeSelectHandlesHierarchyAndDisabledItems();
  void RatingSupportsKeyboardAndPointerInput();
  void TagSupportsToggleAndExplicitRemoval();
  void AvatarAndProgressBarUseSemanticPresentation();
};

void WidgetExtendedComponentsTest::ColorPickerNormalizesAlphaAndExposesValue() {
  ColorPicker picker;
  QSignalSpy color_spy(&picker, &ColorPicker::ColorChanged);
  picker.SetColor(QColor(10, 20, 30, 80));
  QCOMPARE(picker.color(), QColor(10, 20, 30, 255));
  QCOMPARE(color_spy.count(), 1);
  QVERIFY(picker.accessibleDescription().contains(QStringLiteral("#0A141E")));

  picker.SetAlphaEnabled(true);
  picker.SetColor(QColor(10, 20, 30, 80));
  QCOMPARE(picker.color().alpha(), 80);
  QVERIFY(picker.accessibleDescription().contains(QStringLiteral("#500A141E")));
  QVERIFY(picker.sizeHint().width() > picker.minimumSizeHint().width());
}

void WidgetExtendedComponentsTest::
    FileDropZoneSeparatesAcceptedAndRejectedPaths() {
  FileDropZone zone;
  zone.resize(zone.sizeHint());
  zone.SetAllowedExtensions({QStringLiteral("*.BAG")});
  QCOMPARE(zone.allowed_extensions(), QStringList{QStringLiteral("bag")});

  QSignalSpy accepted_spy(&zone, &FileDropZone::FilesAccepted);
  QSignalSpy rejected_spy(&zone, &FileDropZone::FilesRejected);
  QMimeData mime_data;
  mime_data.setUrls({QUrl::fromLocalFile(QStringLiteral("/tmp/map.bag")),
                     QUrl::fromLocalFile(QStringLiteral("/tmp/readme.txt"))});
  QDragEnterEvent drag_enter(QPoint(20, 20), Qt::CopyAction, &mime_data,
                             Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(&zone, &drag_enter);
  QVERIFY(drag_enter.isAccepted());
  QDropEvent drop(QPointF(20.0, 20.0), Qt::CopyAction, &mime_data,
                  Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(&zone, &drop);

  QCOMPARE(accepted_spy.count(), 1);
  QCOMPARE(rejected_spy.count(), 1);
  QCOMPARE(accepted_spy.first().first().toStringList(),
           QStringList{QStringLiteral("/tmp/map.bag")});
  QCOMPARE(rejected_spy.first().first().toStringList(),
           QStringList{QStringLiteral("/tmp/readme.txt")});

  FileDropZone archive_zone;
  archive_zone.SetAllowedExtensions({QStringLiteral("*.tar.gz")});
  QSignalSpy archive_accepted(&archive_zone, &FileDropZone::FilesAccepted);
  QSignalSpy archive_rejected(&archive_zone, &FileDropZone::FilesRejected);
  QMimeData archive_data;
  archive_data.setUrls(
      {QUrl::fromLocalFile(QStringLiteral("/tmp/dataset.tar.gz")),
       QUrl::fromLocalFile(QStringLiteral("/tmp/dataset.gz"))});
  QDragEnterEvent archive_drag_enter(QPoint(20, 20), Qt::CopyAction,
                                     &archive_data, Qt::LeftButton,
                                     Qt::NoModifier);
  QApplication::sendEvent(&archive_zone, &archive_drag_enter);
  QDropEvent archive_drop(QPointF(20.0, 20.0), Qt::CopyAction, &archive_data,
                          Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(&archive_zone, &archive_drop);
  QCOMPARE(archive_accepted.count(), 1);
  QCOMPARE(archive_rejected.count(), 1);
}

void WidgetExtendedComponentsTest::MultiSelectEnforcesLimitAndReturnsData() {
  MultiSelectComboBox selector;
  selector.SetItems({{QStringLiteral("Camera"), 10, true},
                     {QStringLiteral("LiDAR"), 20, true},
                     {QStringLiteral("Disabled"), 30, false}});
  selector.SetMaximumSelectionCount(2);
  QSignalSpy selection_spy(&selector, &MultiSelectComboBox::SelectionChanged);
  selector.SetSelectedIndices({2, 0, 1});
  QCOMPARE(selector.selected_indices(), QVector<int>({0, 1}));
  QCOMPARE(selector.selected_data(), QVariantList({10, 20}));
  QCOMPARE(selection_spy.count(), 1);
  QVERIFY(selector.accessibleDescription().contains(QStringLiteral("2")));

  selector.SetMaximumSelectionCount(1);
  QCOMPARE(selector.selected_indices(), QVector<int>({0}));
  selector.SetItems({{QStringLiteral("Map"), 40, true}});
  QVERIFY(selector.selected_indices().isEmpty());
  QCOMPARE(selection_spy.count(), 3);
}

void WidgetExtendedComponentsTest::
    TreeSelectHandlesHierarchyAndDisabledItems() {
  TreeSelect selector;
  selector.SetItems(
      {{QStringLiteral("sensors"), QString(), QStringLiteral("Sensors"),
        QStringLiteral("group"), true, false},
       {QStringLiteral("camera"), QStringLiteral("sensors"),
        QStringLiteral("Camera"), 1, true, true},
       {QStringLiteral("imu"), QStringLiteral("sensors"), QStringLiteral("IMU"),
        3, true, true},
       {QStringLiteral("lidar"), QStringLiteral("sensors"),
        QStringLiteral("LiDAR"), 2, false, true},
       {QStringLiteral("disabled"), QString(), QStringLiteral("Disabled"),
        QVariant(), false, false},
       {QStringLiteral("hidden_child"), QStringLiteral("disabled"),
        QStringLiteral("Hidden child"), 4, true, true}});
  QSignalSpy changed_spy(&selector, &TreeSelect::CurrentItemChanged);
  QVERIFY(selector.SetCurrentId(QStringLiteral("camera")));
  QCOMPARE(selector.current_id(), QStringLiteral("camera"));
  QCOMPARE(selector.current_data().toInt(), 1);
  QCOMPARE(changed_spy.count(), 1);
  QVERIFY(selector.SetCurrentId(QStringLiteral("imu")));
  QAction* camera_action = nullptr;
  QAction* imu_action = nullptr;
  for (QAction* action : selector.findChildren<QAction*>()) {
    if (action->text() == QStringLiteral("Camera")) {
      camera_action = action;
    } else if (action->text() == QStringLiteral("IMU")) {
      imu_action = action;
    }
  }
  QVERIFY(camera_action != nullptr);
  QVERIFY(imu_action != nullptr);
  QVERIFY(!camera_action->isChecked());
  QVERIFY(imu_action->isChecked());
  QVERIFY(!selector.SetCurrentId(QStringLiteral("sensors")));
  QVERIFY(!selector.SetCurrentId(QStringLiteral("lidar")));
  QVERIFY(!selector.SetCurrentId(QStringLiteral("hidden_child")));
  QVERIFY(!selector.SetCurrentId(QStringLiteral("missing")));
}

void WidgetExtendedComponentsTest::RatingSupportsKeyboardAndPointerInput() {
  Rating rating;
  rating.resize(rating.sizeHint());
  rating.show();
  rating.setFocus();
  QSignalSpy value_spy(&rating, &Rating::ValueChanged);
  QTest::keyClick(&rating, Qt::Key_Right);
  QTest::keyClick(&rating, Qt::Key_Right);
  QCOMPARE(rating.value(), 2);
  QTest::keyClick(&rating, Qt::Key_End);
  QCOMPARE(rating.value(), rating.maximum());
  QTest::mouseClick(&rating, Qt::LeftButton, Qt::NoModifier,
                    QPoint(rating.width() / 2, rating.height() / 2));
  QCOMPARE(rating.value(), 3);
  QVERIFY(value_spy.count() >= 3);
  QVERIFY(rating.accessibleDescription().contains(QStringLiteral("3")));
  const int signal_count = value_spy.count();
  rating.SetMaximum(2);
  QCOMPARE(rating.value(), 2);
  QCOMPARE(value_spy.count(), signal_count + 1);
}

void WidgetExtendedComponentsTest::TagSupportsToggleAndExplicitRemoval() {
  Tag tag(QStringLiteral("Loop closure"));
  tag.setCheckable(true);
  tag.SetRemovable(true);
  tag.resize(tag.sizeHint());
  tag.show();
  tag.setFocus();
  QSignalSpy remove_spy(&tag, &Tag::RemoveRequested);
  QTest::keyClick(&tag, Qt::Key_Space);
  QVERIFY(tag.isChecked());
  QTest::keyClick(&tag, Qt::Key_Delete);
  QCOMPARE(remove_spy.count(), 1);
  QCOMPARE(tag.accessibleName(), QStringLiteral("Loop closure"));
}

void WidgetExtendedComponentsTest::
    AvatarAndProgressBarUseSemanticPresentation() {
  Avatar avatar;
  avatar.SetName(QStringLiteral("Autonomy Robot"));
  avatar.SetDiameter(48);
  QCOMPARE(avatar.sizeHint(), QSize(48, 48));
  QCOMPARE(avatar.accessibleName(), QStringLiteral("Autonomy Robot"));
  avatar.resize(avatar.sizeHint());
  avatar.show();
  QVERIFY(!avatar.grab().isNull());

  ProgressBar progress;
  progress.SetState(ProgressBar::State::kSuccess);
  progress.setRange(0, 100);
  progress.setValue(64);
  QCOMPARE(progress.value(), 64);
  QCOMPARE(progress.state(), ProgressBar::State::kSuccess);
  QCOMPARE(progress.palette().color(QPalette::Highlight),
           GetThemeTokens().success);
  progress.setEnabled(false);
  QCOMPARE(progress.palette().color(QPalette::Highlight),
           GetThemeTokens().disabled);
  QVERIFY(progress.sizeHint().width() >= 160);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetExtendedComponentsTest)
#include "widget_extended_components_test.moc"
