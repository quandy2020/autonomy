/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>
#include <QStringList>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QDragEnterEvent;
class QDragLeaveEvent;
class QDropEvent;
class QMimeData;
class QPaintEvent;

namespace autoviz::widget {

// Keyboard-operable file picker that also accepts local-file drops. Extension
// filters are compared case-insensitively and never inspect file contents.
class FileDropZone final : public QAbstractButton {
  Q_OBJECT

 public:
  explicit FileDropZone(QWidget* parent = nullptr);

  QString prompt() const { return prompt_; }
  void SetPrompt(const QString& prompt);
  QStringList allowed_extensions() const { return allowed_extensions_; }
  void SetAllowedExtensions(const QStringList& extensions);
  bool multiple_files_enabled() const noexcept { return multiple_files_; }
  void SetMultipleFilesEnabled(bool enabled);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void FilesAccepted(const QStringList& local_paths);
  void FilesRejected(const QStringList& local_paths);

 protected:
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragLeaveEvent(QDragLeaveEvent* event) override;
  void dropEvent(QDropEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private slots:
  void OpenFileDialog();

 private:
  QStringList LocalPathsFromMimeData(const QMimeData* mime_data) const;
  bool IsPathAllowed(const QString& path) const;
  void SubmitPaths(const QStringList& paths);

  QString prompt_;
  QStringList allowed_extensions_;
  style::Tone tone_ = style::Tone::Shell;
  bool multiple_files_ = true;
  bool drag_active_ = false;
};

}  // namespace autoviz::widget
