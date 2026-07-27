/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDialog>

class QComboBox;
class QLineEdit;
class QPushButton;
class QStackedWidget;

namespace autoviz {
namespace integration {
class PlaybackController;
}

/** Unified Open / Import Bag / Import MCAP wizard for the Time panel. */
class ImportRecordDialog : public QDialog {
  Q_OBJECT

 public:
  enum class Mode { kOpenRecord, kImportBag, kImportMcap };

  explicit ImportRecordDialog(integration::PlaybackController* controller,
                              QWidget* parent = nullptr);

  bool recordOpened() const { return record_opened_; }

 private slots:
  void onModeChanged(int index);
  void onBrowseSource();
  void onBrowseOutput();
  void onImport();

 private:
  void setupUi();
  Mode currentMode() const;
  bool runConversion(QString* error_message);

  integration::PlaybackController* controller_ = nullptr;
  QComboBox* mode_combo_ = nullptr;
  QStackedWidget* stack_ = nullptr;
  QLineEdit* source_edit_ = nullptr;
  QLineEdit* output_edit_ = nullptr;
  QPushButton* output_browse_ = nullptr;
  bool record_opened_ = false;
};

}  // namespace autoviz
