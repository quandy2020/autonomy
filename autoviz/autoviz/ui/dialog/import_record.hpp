/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file import_record.hpp
 * @brief Unified Open / Import Bag / Import MCAP wizard for the Time panel.
 *
 * Wraps @ref OpenRecordSource() with a small UI for choosing mode, source
 * path, and (for conversions) output @c .record path.
 *
 * @see OpenRecordSource
 * @see RecordSourceKind
 * @see TimePanel
 * @see integration::PlaybackController
 */

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

/**
 * @class ImportRecordDialog
 * @brief Modal wizard to open an Autolink record or convert bag/MCAP first.
 *
 * ## Modes
 *
 * | Mode | UI | Action |
 * |------|----|--------|
 * | @ref Mode::kOpenRecord | Source path only | Open existing @c .record |
 * | @ref Mode::kImportBag | Source + output | Convert @c .bag → @c .record, then open |
 * | @ref Mode::kImportMcap | Source + output | Convert @c .mcap → @c .record, then open |
 *
 * On success, @ref recordOpened() is @c true and the PlaybackController holds
 * the opened record (playback is not started here).
 *
 * @see OpenRecordResult
 * @see OpenRecordSource()
 */
class ImportRecordDialog : public QDialog {
  Q_OBJECT

 public:
  /**
   * @enum Mode
   * @brief Wizard page / conversion mode selected in the mode combo.
   */
  enum class Mode {
    kOpenRecord,  /**< Open an existing Autolink @c .record file. */
    kImportBag,   /**< Convert ROS bag then open the resulting record. */
    kImportMcap,  /**< Convert MCAP then open the resulting record. */
  };

  /**
   * @brief Constructs the dialog and builds mode stack / path editors.
   *
   * @param controller Non-owning playback controller used to open the record
   *        after optional conversion.
   * @param parent Qt parent widget.
   */
  explicit ImportRecordDialog(integration::PlaybackController* controller,
                              QWidget* parent = nullptr);

  /**
   * @brief Whether a record was successfully opened before the dialog closed.
   *
   * @return @c true after a successful Import / Open; @c false if cancelled
   *         or conversion failed.
   */
  bool recordOpened() const { return record_opened_; }

  /**
   * @brief Prefills the source path editor (e.g. from a file-drop).
   *
   * May also infer mode via @ref ClassifyRecordSource().
   *
   * @param path Absolute or relative path to a record / bag / mcap file.
   */
  void setSourcePath(const QString& path);

 private slots:
  /**
   * @brief Mode combo changed: switches @c stack_ and shows/hides output row.
   *
   * @param index Combo index corresponding to @ref Mode.
   */
  void onModeChanged(int index);

  /**
   * @brief Browse button for the source path (file dialog).
   */
  void onBrowseSource();

  /**
   * @brief Browse button for the converted output @c .record path.
   */
  void onBrowseOutput();

  /**
   * @brief Import / Open button: runs conversion if needed, then opens.
   *
   * Sets @c record_opened_ and accepts the dialog on success; shows an error
   * and keeps the dialog open on failure.
   */
  void onImport();

 private:
  /**
   * @brief Builds mode combo, stacked pages, path editors, and buttons.
   */
  void setupUi();

  /**
   * @brief Maps the mode combo selection to a @ref Mode value.
   *
   * @return Current wizard mode.
   */
  Mode currentMode() const;

  /**
   * @brief Runs bag/MCAP conversion when not in @ref Mode::kOpenRecord.
   *
   * @param error_message Filled with a human-readable error on failure.
   * @return @c true when conversion succeeded or was not required.
   */
  bool runConversion(QString* error_message);

  /** Non-owning playback controller for open-after-convert. */
  integration::PlaybackController* controller_ = nullptr;

  /** Mode selector (Open / Import Bag / Import MCAP). */
  QComboBox* mode_combo_ = nullptr;

  /** Stacked widget for mode-specific hints / extra fields. */
  QStackedWidget* stack_ = nullptr;

  /** Source file path editor. */
  QLineEdit* source_edit_ = nullptr;

  /** Output @c .record path editor (hidden for Open mode). */
  QLineEdit* output_edit_ = nullptr;

  /** Browse button for @c output_edit_. */
  QPushButton* output_browse_ = nullptr;

  /** Set when open succeeds; read via @ref recordOpened(). */
  bool record_opened_ = false;
};

}  // namespace autoviz
