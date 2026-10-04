/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file record_open.hpp
 * @brief Helpers to classify, convert, and open Autolink record sources.
 *
 * Shared by @ref ImportRecordDialog, drag-and-drop onto the Time panel, and
 * CLI / recent-file open paths. Conversion of @c .bag / @c .mcap produces a
 * @c .record that @ref integration::PlaybackController can load; this API does
 * not start playback.
 *
 * @see ImportRecordDialog
 * @see OpenRecordResult
 * @see integration::PlaybackController
 */

#pragma once

#include <QString>
#include <QStringList>

class QMimeData;

namespace autoviz {
namespace integration {
class PlaybackController;
}

/**
 * @enum RecordSourceKind
 * @brief Classified kind of a filesystem path offered as a record source.
 */
enum class RecordSourceKind {
  kRecord,   /**< Native Autolink @c .record file. */
  kBag,      /**< ROS bag requiring conversion. */
  kMcap,     /**< MCAP file requiring conversion. */
  kUnknown,  /**< Unrecognized extension / not a record source. */
};

/**
 * @brief Classifies @p path by file extension / known suffixes.
 *
 * @param path Filesystem path to inspect.
 * @return Source kind; @ref RecordSourceKind::kUnknown when not recognized.
 */
RecordSourceKind ClassifyRecordSource(const QString& path);

/**
 * @brief Returns whether @p path is a recognized record / bag / mcap source.
 *
 * @param path Filesystem path to inspect.
 * @return @c true when @ref ClassifyRecordSource() is not @c kUnknown.
 */
bool IsRecordSourcePath(const QString& path);

/**
 * @brief Extracts local filesystem paths that look like record sources from
 *        drag-and-drop MIME data.
 *
 * @param mime Qt MIME data from a drop event (may be @c nullptr).
 * @return List of absolute paths classified as record sources.
 */
QStringList LocalRecordSourcePaths(const QMimeData* mime);

/**
 * @brief Suggests a default output @c .record path next to @p source_path.
 *
 * @param source_path Input bag / mcap / record path.
 * @return Proposed destination path for conversion output.
 */
QString DefaultConvertedRecordPath(const QString& source_path);

/**
 * @struct OpenRecordResult
 * @brief Outcome of @ref OpenRecordSource().
 */
struct OpenRecordResult {
  bool ok = false;       /**< @c true when the record was opened successfully. */
  QString error;         /**< Human-readable error when @c ok is @c false. */
  QString record_path;   /**< Path actually opened (may differ after conversion). */
};

/**
 * @brief Convert @c .bag / @c .mcap if needed, then open the Autolink
 *        @c .record. Does not play.
 *
 * @param controller Playback controller that receives the opened record.
 * @param path Source path (record, bag, or mcap).
 * @param output_record_path Optional explicit conversion output; when empty,
 *        @ref DefaultConvertedRecordPath() is used for bag/mcap.
 * @return Result with @c ok / @c error / @c record_path.
 *
 * @see ImportRecordDialog
 * @see ClassifyRecordSource()
 */
OpenRecordResult OpenRecordSource(integration::PlaybackController* controller,
                                  const QString& path,
                                  const QString& output_record_path = QString());

}  // namespace autoviz
