/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QStringList>

class QMimeData;

namespace autoviz {
namespace integration {
class PlaybackController;
}

enum class RecordSourceKind { kRecord, kBag, kMcap, kUnknown };

RecordSourceKind ClassifyRecordSource(const QString& path);
bool IsRecordSourcePath(const QString& path);
QStringList LocalRecordSourcePaths(const QMimeData* mime);
QString DefaultConvertedRecordPath(const QString& source_path);

struct OpenRecordResult {
  bool ok = false;
  QString error;
  QString record_path;
};

/** Convert .bag/.mcap if needed, then open the Autolink .record. Does not play. */
OpenRecordResult OpenRecordSource(integration::PlaybackController* controller,
                                  const QString& path,
                                  const QString& output_record_path = QString());

}  // namespace autoviz
