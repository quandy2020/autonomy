/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QPlainTextEdit;

namespace autoviz {

/** Foxglove-style Problems sidebar (playback / connection diagnostics). */
class ProblemsPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ProblemsPanel(QWidget* parent = nullptr);

  void refresh();

 private:
  QPlainTextEdit* content_ = nullptr;
};

}  // namespace autoviz
