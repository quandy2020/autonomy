/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file evidence_panel.hpp
 * @brief Episode evidence triage: timeline, causal graph, BT tree, overlay.
 *
 * Loads recorder sidecar JSON (no autonomy_system dependency). Shared cursor
 * binds graph / Gantt / observations; seek maps onto Record playback.
 */

#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QPointer>
#include <QString>
#include <QWidget>

class QComboBox;
class QGraphicsView;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QSlider;
class QTabWidget;
class QToolButton;
class QTreeWidget;

namespace autoviz {
namespace common {
class VisualizationManager;
}
class PanelDockWidget;

class EvidencePanel : public QWidget {
  Q_OBJECT

 public:
  explicit EvidencePanel(common::VisualizationManager* manager,
                         QWidget* parent = nullptr);
  ~EvidencePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);
  bool openSidecarFile(const QString& path);

 signals:
  void panelRemoveRequested();
  void panelExpandRequested();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelChangeRequested(const QString& object_name);

 private slots:
  void onOpenSidecar();
  void onOpenOverlayB();
  void onCursorChanged(int value);
  void onSaveObservation();
  void onNodeActivated(const QString& node_id);

 private:
  class GanttWidget;
  class CausalView;

  bool loadSidecar(const QString& path);
  bool loadOverlay(const QString& path, QJsonObject* dest);
  void loadObservations(const QString& jsonl_path);
  void rebuildViews();
  void applyCursor(qint64 monotonic_ns, bool seek_playback);
  qint64 sliderToNs(int value) const;
  int nsToSlider(qint64 ns) const;
  void seekPlayback(qint64 monotonic_ns);
  void appendObservationRow(const QJsonObject& obs);
  QString siblingPath(const QString& sidecar, const QString& suffix) const;

  common::VisualizationManager* manager_ = nullptr;
  QPointer<QToolButton> expand_button_;

  QLabel* header_ = nullptr;
  QSlider* cursor_ = nullptr;
  QLabel* cursor_label_ = nullptr;
  QTabWidget* tabs_ = nullptr;
  GanttWidget* gantt_ = nullptr;
  CausalView* causal_ = nullptr;
  QTreeWidget* bt_tree_ = nullptr;
  QListWidget* data_list_ = nullptr;
  QPlainTextEdit* compare_ = nullptr;

  QLineEdit* obs_id_ = nullptr;
  QPlainTextEdit* obs_desc_ = nullptr;
  QComboBox* obs_node_ = nullptr;
  QLineEdit* obs_bt_ = nullptr;

  QString sidecar_path_;
  QString observations_path_;
  QJsonObject sidecar_;
  QJsonObject overlay_a_;
  QJsonObject overlay_b_;
  QJsonArray observations_;
  QJsonArray nodes_;
  QJsonArray edges_;
  QJsonArray triggers_;
  qint64 start_ns_ = 0;
  qint64 end_ns_ = 1;
  qint64 cursor_ns_ = 0;
};

}  // namespace autoviz
