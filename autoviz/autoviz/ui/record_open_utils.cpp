/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/record_open_utils.hpp"

#include <QCoreApplication>
#include <QFileInfo>
#include <QMimeData>
#include <QProcess>
#include <QStandardPaths>
#include <QUrl>

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
      QCoreApplication::applicationDirPath() +
      QStringLiteral("/../share/autonomy/autoviz/scripts/mcap_to_record.py");
  if (QFileInfo::exists(installed)) {
    return QFileInfo(installed).absoluteFilePath();
  }
  const QString dev =
      QCoreApplication::applicationDirPath() +
      QStringLiteral("/../../autoviz/scripts/mcap_to_record.py");
  if (QFileInfo::exists(dev)) {
    return QFileInfo(dev).absoluteFilePath();
  }
  return {};
}

bool RunProcess(const QString& program, const QStringList& args,
                QString* error_message, const QString& timeout_label) {
  QProcess process;
  process.start(program, args);
  if (!process.waitForStarted(3000)) {
    *error_message = QCoreApplication::translate(
        "autoviz", "Failed to start %1.").arg(timeout_label);
    return false;
  }
  if (!process.waitForFinished(600000)) {
    process.kill();
    *error_message = QCoreApplication::translate(
        "autoviz", "%1 timed out.").arg(timeout_label);
    return false;
  }
  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    const QString details =
        QString::fromUtf8(process.readAllStandardError()).trimmed();
    *error_message =
        QCoreApplication::translate("autoviz", "%1 failed (code %2).\n%3")
            .arg(timeout_label)
            .arg(process.exitCode())
            .arg(details.isEmpty()
                     ? QCoreApplication::translate("autoviz", "No stderr output.")
                     : details);
    return false;
  }
  return true;
}

}  // namespace

RecordSourceKind ClassifyRecordSource(const QString& path) {
  const QString suffix = QFileInfo(path).suffix().toLower();
  if (suffix == QLatin1String("record")) {
    return RecordSourceKind::kRecord;
  }
  if (suffix == QLatin1String("bag")) {
    return RecordSourceKind::kBag;
  }
  if (suffix == QLatin1String("mcap")) {
    return RecordSourceKind::kMcap;
  }
  return RecordSourceKind::kUnknown;
}

bool IsRecordSourcePath(const QString& path) {
  return ClassifyRecordSource(path) != RecordSourceKind::kUnknown;
}

QStringList LocalRecordSourcePaths(const QMimeData* mime) {
  QStringList paths;
  if (mime == nullptr || !mime->hasUrls()) {
    return paths;
  }
  for (const QUrl& url : mime->urls()) {
    if (!url.isLocalFile()) {
      continue;
    }
    const QString path = url.toLocalFile();
    if (IsRecordSourcePath(path)) {
      paths.push_back(path);
    }
  }
  return paths;
}

QString DefaultConvertedRecordPath(const QString& source_path) {
  const QFileInfo info(source_path);
  return info.absolutePath() + QLatin1Char('/') + info.completeBaseName() +
         QStringLiteral(".record");
}

OpenRecordResult OpenRecordSource(integration::PlaybackController* controller,
                                  const QString& path,
                                  const QString& output_record_path) {
  OpenRecordResult result;
  if (controller == nullptr) {
    result.error = QCoreApplication::translate(
        "autoviz", "Playback controller is not available.");
    return result;
  }
  const QString source = path.trimmed();
  if (source.isEmpty() || !QFileInfo::exists(source)) {
    result.error = QCoreApplication::translate(
        "autoviz", "Record file does not exist:\n%1").arg(path);
    return result;
  }

  const RecordSourceKind kind = ClassifyRecordSource(source);
  QString record_path = source;
  if (kind == RecordSourceKind::kBag || kind == RecordSourceKind::kMcap) {
    record_path = output_record_path.trimmed().isEmpty()
                      ? DefaultConvertedRecordPath(source)
                      : output_record_path;
    if (kind == RecordSourceKind::kBag) {
      const QString converter = FindBagConverterExecutable();
      if (converter.isEmpty()) {
        result.error = QCoreApplication::translate(
            "autoviz",
            "Could not find `bag_to_record` in PATH.\n\n"
            "Install Autolink developer tools, then run:\n"
            "  bag_to_record %1 %2")
                           .arg(source, record_path);
        return result;
      }
      if (!RunProcess(converter, {source, record_path}, &result.error,
                      QStringLiteral("bag_to_record"))) {
        return result;
      }
    } else {
      const QString script_path = McapConverterScriptPath();
      const QString python =
          QStandardPaths::findExecutable(QStringLiteral("python3"));
      if (script_path.isEmpty() || python.isEmpty()) {
        result.error = QCoreApplication::translate(
            "autoviz",
            "MCAP converter script or python3 not found.\n\n"
            "Convert offline with Autolink tools, then open the .record file:\n"
            "  mcap_to_record.py %1 %2")
                           .arg(source, record_path);
        return result;
      }
      if (!RunProcess(python, {script_path, source, record_path}, &result.error,
                      QStringLiteral("mcap_to_record"))) {
        return result;
      }
    }
  } else if (kind == RecordSourceKind::kUnknown) {
    result.error = QCoreApplication::translate(
        "autoviz",
        "Unsupported file type. Open an Autolink .record, or import .bag/.mcap.");
    return result;
  }

  if (!controller->openFile(record_path.toStdString())) {
    result.error = QCoreApplication::translate(
        "autoviz",
        "Could not open record file:\n%1\n\n"
        "Ensure the file is a valid Autolink .record.")
                       .arg(record_path);
    return result;
  }
  result.ok = true;
  result.record_path = record_path;
  return result;
}

}  // namespace autoviz
