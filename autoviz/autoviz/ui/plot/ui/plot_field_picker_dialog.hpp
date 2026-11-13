/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_field_picker_dialog.hpp
 * @brief Searchable channel → numeric-field picker for Plot series.
 *
 * Concat Library–style master/detail: channel rail on the left, field cards
 * on the right, near-opaque milky plate (no viewport bleed-through).
 *
 * @see PopulatePlotChannelFieldMenu
 * @see NumericFieldPathsForMessageType
 */

#pragma once

#include <QDialog>
#include <QPoint>
#include <QString>
#include <QStringList>
#include <QVector>

class QKeyEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QMouseEvent;
class QPaintEvent;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace plot {

/**
 * @class PlotFieldPickerDialog
 * @brief Modal dialog: filter + channel rail + numeric field list.
 */
class PlotFieldPickerDialog : public QDialog {
  Q_OBJECT

 public:
  explicit PlotFieldPickerDialog(common::VisualizationManager* manager,
                                 QWidget* parent = nullptr);

  QSize sizeHint() const override;

  QString selectedChannel() const { return selected_channel_; }
  QString selectedFieldPath() const { return selected_field_path_; }

  static bool pick(QWidget* parent, common::VisualizationManager* manager,
                   QString* channel_out, QString* field_path_out);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  struct ChannelEntry {
    QString channel;
    QStringList fields;
    /** Display type id for @ref IconLoader::displayIcon (may be empty). */
    QString display_type;
  };

  void rebuildCatalog();
  void applyFilter(const QString& text);
  void onChannelChanged();
  void onFieldChanged();
  void updatePathPreview();
  void updateAddEnabled();
  void acceptCurrent();

  common::VisualizationManager* manager_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QListWidget* channel_list_ = nullptr;
  QListWidget* field_list_ = nullptr;
  QLabel* channel_caption_ = nullptr;
  QLabel* field_caption_ = nullptr;
  QLabel* status_label_ = nullptr;
  QLabel* path_preview_ = nullptr;
  QPushButton* add_button_ = nullptr;
  QVector<ChannelEntry> catalog_;
  QString selected_channel_;
  QString selected_field_path_;
  bool dragging_ = false;
  QPoint drag_offset_;
};

}  // namespace plot
}  // namespace autoviz
