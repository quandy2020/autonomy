/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/import_record_dialog.hpp"

#include <QComboBox>
#include <QCoreApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QVBoxLayout>

#include "autoviz/integration/playback_controller.hpp"

namespace autoviz {
namespace {

QString FindBagConverterExecutable() {
  const QString primary =
      QStandardPaths::findExecutable(QStringLiteral("bag_to_record"));
  if (!primary.isEmpty()) {
    return primary;
  }
  return QStandardPaths::findExecutable(QStringLiteral("rosbag_to_record"));
}

QString McapConverterScriptPath() {
  const QString installed =
      QCoreApplication::applicationDirPath() + QStringLiteral("/../share/autonomy/autoviz/scripts/mcap_to_record.py");
  if (QFileInfo::exists(installed)) {
    return QFileInfo(installed).absoluteFilePath();
  }
  const QString dev =
      QCoreApplication::applicationDirPath() + QStringLiteral("/../../autoviz/scripts/mcap_to_record.py");
  if (QFileInfo::exists(dev)) {
    return QFileInfo(dev).absoluteFilePath();
  }
  return QString();
}

QString DefaultOutputPath(const QString& source_path,
                          ImportRecordDialog::Mode mode) {
  const QFileInfo info(source_path);
  if (mode == ImportRecordDialog::Mode::kOpenRecord) {
    return source_path;
  }
  return info.absolutePath() + QLatin1Char('/') + info.completeBaseName() + QStringLiteral(".record");
}

}  // namespace

ImportRecordDialog::ImportRecordDialog(
    integration::PlaybackController* controller, QWidget* parent)
    : QDialog(parent), controller_(controller) {
  setupUi();
}

void ImportRecordDialog::setupUi() {
  setWindowTitle(tr("Import / Open Record"));
  auto* layout = new QVBoxLayout(this);

  auto* form = new QFormLayout();
  mode_combo_ = new QComboBox(this);
  mode_combo_->addItem(tr("Open Autolink Record (.record)"),
                       static_cast<int>(Mode::kOpenRecord));
  mode_combo_->addItem(tr("Import Legacy Bag (.bag)"),
                       static_cast<int>(Mode::kImportBag));
  mode_combo_->addItem(tr("Import MCAP (.mcap)"),
                       static_cast<int>(Mode::kImportMcap));
  form->addRow(tr("Mode"), mode_combo_);
  layout->addLayout(form);

  stack_ = new QStackedWidget(this);
  auto* open_page = new QWidget(stack_);
  auto* open_layout = new QVBoxLayout(open_page);
  open_layout->addWidget(new QLabel(
      tr("Open an existing Autolink .record file for playback."), open_page));
  stack_->addWidget(open_page);

  auto* bag_page = new QWidget(stack_);
  auto* bag_layout = new QVBoxLayout(bag_page);
  bag_layout->addWidget(new QLabel(
      tr("Convert a legacy .bag file to Autolink .record using bag_to_record "
         "(Autolink developer tools), then open the result."),
      bag_page));
  stack_->addWidget(bag_page);

  auto* mcap_page = new QWidget(stack_);
  auto* mcap_layout = new QVBoxLayout(mcap_page);
  mcap_layout->addWidget(new QLabel(
      tr("Convert an .mcap recording to Autolink .record, then open the "
         "result."),
      mcap_page));
  stack_->addWidget(mcap_page);
  layout->addWidget(stack_);

  layout->addWidget(new QLabel(tr("Source file:"), this));
  auto* source_row = new QHBoxLayout();
  source_edit_ = new QLineEdit(this);
  auto* source_browse = new QPushButton(tr("Browse..."), this);
  source_row->addWidget(source_edit_, 1);
  source_row->addWidget(source_browse);
  layout->addLayout(source_row);

  layout->addWidget(new QLabel(tr("Output .record (import modes):"), this));
  auto* output_row = new QHBoxLayout();
  output_edit_ = new QLineEdit(this);
  output_browse_ = new QPushButton(tr("Browse..."), this);
  output_row->addWidget(output_edit_, 1);
  output_row->addWidget(output_browse_);
  layout->addLayout(output_row);

  auto* buttons = new QHBoxLayout();
  buttons->addStretch();
  auto* cancel = new QPushButton(tr("Cancel"), this);
  auto* run = new QPushButton(tr("Open"), this);
  run->setDefault(true);
  buttons->addWidget(cancel);
  buttons->addWidget(run);
  layout->addLayout(buttons);

  connect(mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &ImportRecordDialog::onModeChanged);
  connect(source_browse, &QPushButton::clicked, this,
          &ImportRecordDialog::onBrowseSource);
  connect(output_browse_, &QPushButton::clicked, this,
          &ImportRecordDialog::onBrowseOutput);
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
  connect(run, &QPushButton::clicked, this, &ImportRecordDialog::onImport);

  onModeChanged(mode_combo_->currentIndex());
}

ImportRecordDialog::Mode ImportRecordDialog::currentMode() const {
  return static_cast<Mode>(mode_combo_->currentData().toInt());
}

void ImportRecordDialog::onModeChanged(int index) {
  Q_UNUSED(index);
  const Mode mode = currentMode();
  stack_->setCurrentIndex(static_cast<int>(mode));
  const bool needs_output = mode != Mode::kOpenRecord;
  output_edit_->setEnabled(needs_output);
  output_browse_->setEnabled(needs_output);
  if (!needs_output) {
    output_edit_->clear();
  } else if (output_edit_->text().isEmpty() &&
             !source_edit_->text().isEmpty()) {
    output_edit_->setText(DefaultOutputPath(source_edit_->text(), mode));
  }
}

void ImportRecordDialog::onBrowseSource() {
  const Mode mode = currentMode();
  QString filter;
  if (mode == Mode::kOpenRecord) {
    filter = tr("Autolink Record (*.record);;All Files (*)");
  } else if (mode == Mode::kImportBag) {
    filter = tr("Legacy Bag (*.bag);;All Files (*)");
  } else {
    filter = tr("MCAP (*.mcap);;All Files (*)");
  }
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Select Source File"), source_edit_->text(), filter);
  if (path.isEmpty()) {
    return;
  }
  source_edit_->setText(path);
  if (mode != Mode::kOpenRecord) {
    output_edit_->setText(DefaultOutputPath(path, mode));
  }
}

void ImportRecordDialog::onBrowseOutput() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Autolink Record"), output_edit_->text(),
      tr("Autolink Record (*.record);;All Files (*)"));
  if (!path.isEmpty()) {
    output_edit_->setText(path);
  }
}

bool ImportRecordDialog::runConversion(QString* error_message) {
  const Mode mode = currentMode();
  const QString source = source_edit_->text().trimmed();
  if (source.isEmpty()) {
    *error_message = tr("Select a source file.");
    return false;
  }

  if (mode == Mode::kOpenRecord) {
    const QString suffix = QFileInfo(source).suffix().toLower();
    if (suffix == QLatin1String("bag") || suffix == QLatin1String("mcap")) {
      *error_message =
          tr("Use Import mode for .bag or .mcap files.");
      return false;
    }
    if (!controller_->openFile(source.toStdString())) {
      *error_message =
          tr("Could not open record file:\n%1").arg(source);
      return false;
    }
    record_opened_ = true;
    return true;
  }

  const QString output = output_edit_->text().trimmed();
  if (output.isEmpty()) {
    *error_message = tr("Select an output .record path.");
    return false;
  }

  if (mode == Mode::kImportBag) {
    const QString converter = FindBagConverterExecutable();
    if (converter.isEmpty()) {
      *error_message =
          tr("Could not find `bag_to_record` in PATH.\n\n"
             "Install Autolink developer tools, then run:\n"
             "  bag_to_record %1 %2")
              .arg(source, output);
      return false;
    }
    QProcess process;
    process.start(converter, {source, output});
    if (!process.waitForStarted(3000)) {
      *error_message = tr("Failed to start bag_to_record.");
      return false;
    }
    if (!process.waitForFinished(600000)) {
      process.kill();
      *error_message = tr("bag_to_record timed out.");
      return false;
    }
    if (process.exitStatus() != QProcess::NormalExit ||
        process.exitCode() != 0) {
      const QString details =
          QString::fromUtf8(process.readAllStandardError()).trimmed();
      *error_message =
          tr("bag_to_record failed (code %1).\n%2")
              .arg(process.exitCode())
              .arg(details.isEmpty() ? tr("No stderr output.") : details);
      return false;
    }
  } else {
    const QString script_path = McapConverterScriptPath();
    const QString python =
        QStandardPaths::findExecutable(QStringLiteral("python3"));
    if (script_path.isEmpty() || python.isEmpty()) {
      *error_message =
          tr("MCAP converter script or python3 not found.\n\n"
             "Convert offline with Autolink tools, then open the .record file:\n"
             "  mcap_to_record.py %1 %2")
              .arg(source, output);
      return false;
    }
    QProcess process;
    process.start(python, {script_path, source, output});
    if (!process.waitForStarted(3000)) {
      *error_message = tr("Failed to start MCAP converter.");
      return false;
    }
    if (!process.waitForFinished(600000)) {
      process.kill();
      *error_message = tr("MCAP converter timed out.");
      return false;
    }
    if (process.exitStatus() != QProcess::NormalExit ||
        process.exitCode() != 0) {
      const QString details =
          QString::fromUtf8(process.readAllStandardError()).trimmed();
      *error_message =
          tr("MCAP conversion failed.\n%1")
              .arg(details.isEmpty()
                       ? tr("See share/autonomy/autoviz/scripts/mcap_to_record.py.")
                       : details);
      return false;
    }
  }

  if (!controller_->openFile(output.toStdString())) {
    *error_message =
        tr("Converted record could not be opened:\n%1").arg(output);
    return false;
  }
  record_opened_ = true;
  return true;
}

void ImportRecordDialog::onImport() {
  QString error;
  if (!runConversion(&error)) {
    QMessageBox::warning(this, tr("Import Failed"), error);
    return;
  }
  accept();
}

}  // namespace autoviz
