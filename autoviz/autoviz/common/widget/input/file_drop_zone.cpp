/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/file_drop_zone.hpp"

#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QMimeData>
#include <QPainter>
#include <QSet>
#include <QUrl>

namespace autoviz::widget {

FileDropZone::FileDropZone(QWidget* parent) : QAbstractButton(parent) {
  setObjectName(QStringLiteral("AutovizFileDropZone"));
  setProperty("autovizRole", QStringLiteral("fileDropZone"));
  setAcceptDrops(true);
  setFocusPolicy(Qt::StrongFocus);
  setCursor(Qt::PointingHandCursor);
  prompt_ = tr("Drop files here or choose files");
  setAccessibleName(tr("File picker"));
  setAccessibleDescription(prompt_);
  connect(this, &QAbstractButton::clicked, this, &FileDropZone::OpenFileDialog);
}

void FileDropZone::SetPrompt(const QString& prompt) {
  if (prompt_ == prompt) {
    return;
  }
  prompt_ = prompt;
  setAccessibleDescription(prompt_);
  updateGeometry();
  update();
}

void FileDropZone::SetAllowedExtensions(const QStringList& extensions) {
  QStringList normalized;
  QSet<QString> seen;
  for (QString extension : extensions) {
    extension = extension.trimmed().toLower();
    while (extension.startsWith(QLatin1Char('*')) ||
           extension.startsWith(QLatin1Char('.'))) {
      extension.remove(0, 1);
    }
    if (!extension.isEmpty() && !seen.contains(extension)) {
      normalized.push_back(extension);
      seen.insert(extension);
    }
  }
  if (allowed_extensions_ == normalized) {
    return;
  }
  allowed_extensions_ = normalized;
  update();
}

void FileDropZone::SetMultipleFilesEnabled(bool enabled) {
  multiple_files_ = enabled;
}

void FileDropZone::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize FileDropZone::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  return QSize(qMax(240, fontMetrics().horizontalAdvance(prompt_) +
                             2 * tokens.spacing_lg),
               88);
}

QSize FileDropZone::minimumSizeHint() const { return QSize(160, 64); }

void FileDropZone::dragEnterEvent(QDragEnterEvent* event) {
  const QStringList paths = LocalPathsFromMimeData(event->mimeData());
  if (isEnabled() && !paths.isEmpty()) {
    drag_active_ = true;
    event->acceptProposedAction();
    update();
    return;
  }
  event->ignore();
}

void FileDropZone::dragLeaveEvent(QDragLeaveEvent* event) {
  drag_active_ = false;
  update();
  QAbstractButton::dragLeaveEvent(event);
}

void FileDropZone::dropEvent(QDropEvent* event) {
  drag_active_ = false;
  const QStringList paths = LocalPathsFromMimeData(event->mimeData());
  if (!paths.isEmpty()) {
    SubmitPaths(paths);
    event->acceptProposedAction();
  } else {
    event->ignore();
  }
  update();
}

void FileDropZone::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QColor border =
      drag_active_ || hasFocus() ? tokens.focus_ring : tokens.border;
  QColor background =
      drag_active_ ? tokens.surface_hovered : tokens.surface_raised;
  if (!isEnabled()) {
    border = tokens.disabled;
    background = tokens.surface;
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  QPen pen(border, drag_active_ || hasFocus() ? tokens.focus_ring_width : 1,
           Qt::DashLine);
  painter.setPen(pen);
  painter.setBrush(background);
  painter.drawRoundedRect(QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0),
                          tokens.radius_medium, tokens.radius_medium);
  painter.setPen(isEnabled() ? tokens.text : tokens.disabled);
  painter.drawText(rect().adjusted(tokens.spacing_lg, tokens.spacing_md,
                                   -tokens.spacing_lg, -tokens.spacing_md),
                   Qt::AlignCenter | Qt::TextWordWrap, prompt_);
}

void FileDropZone::OpenFileDialog() {
  QString filter;
  if (!allowed_extensions_.isEmpty()) {
    QStringList patterns;
    for (const QString& extension : allowed_extensions_) {
      patterns.push_back(QStringLiteral("*.%1").arg(extension));
    }
    filter = tr("Supported files (%1)").arg(patterns.join(QLatin1Char(' ')));
  }
  const QStringList paths =
      multiple_files_ ? QFileDialog::getOpenFileNames(this, tr("Choose files"),
                                                      QString(), filter)
                      : QStringList{QFileDialog::getOpenFileName(
                            this, tr("Choose file"), QString(), filter)};
  QStringList nonempty_paths;
  for (const QString& path : paths) {
    if (!path.isEmpty()) {
      nonempty_paths.push_back(path);
    }
  }
  if (!nonempty_paths.isEmpty()) {
    SubmitPaths(nonempty_paths);
  }
}

QStringList FileDropZone::LocalPathsFromMimeData(
    const QMimeData* mime_data) const {
  QStringList paths;
  if (mime_data == nullptr || !mime_data->hasUrls()) {
    return paths;
  }
  QSet<QString> seen;
  for (const QUrl& url : mime_data->urls()) {
    if (!url.isLocalFile()) {
      continue;
    }
    const QString path = url.toLocalFile();
    if (!path.isEmpty() && !seen.contains(path)) {
      paths.push_back(path);
      seen.insert(path);
      if (!multiple_files_) {
        break;
      }
    }
  }
  return paths;
}

bool FileDropZone::IsPathAllowed(const QString& path) const {
  if (allowed_extensions_.isEmpty()) {
    return true;
  }
  const QString file_name = QFileInfo(path).fileName().toLower();
  for (const QString& extension : allowed_extensions_) {
    if (file_name.endsWith(QLatin1Char('.') + extension)) {
      return true;
    }
  }
  return false;
}

void FileDropZone::SubmitPaths(const QStringList& paths) {
  QStringList accepted;
  QStringList rejected;
  for (const QString& path : paths) {
    (IsPathAllowed(path) ? accepted : rejected).push_back(path);
  }
  if (!accepted.isEmpty()) {
    emit FilesAccepted(accepted);
  }
  if (!rejected.isEmpty()) {
    emit FilesRejected(rejected);
  }
}

}  // namespace autoviz::widget
