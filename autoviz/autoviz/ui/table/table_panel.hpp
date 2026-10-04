/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file table_panel.hpp
 * @brief Foxglove-style Table panel — show a repeated protobuf field as rows.
 *
 * Subscribes to one channel, resolves @c field_path to a repeated field, and
 * renders each element as a table row (scalar leaves become columns).
 *
 * @see ChannelsPanel
 * @see plot::ResolveRepeatedFieldPath
 */

#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

#include "autoviz/integration/message_queue.hpp"
#include "autoviz/ui/table/table_types.hpp"

class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QLabel;
class QLineEdit;
class QPoint;
class QTableWidget;
class QTimer;
class QToolButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}
class PanelDockWidget;

namespace table_panel {

/**
 * @class TablePanel
 * @brief Live table for one channel + repeated-field path.
 *
 * Drop a Channels array path (or type @c /channel.field) to bind. Values update
 * as new messages arrive.
 */
class TablePanel : public QWidget {
  Q_OBJECT

 public:
  explicit TablePanel(common::VisualizationManager* manager,
                      QWidget* parent = nullptr);
  ~TablePanel() override;

  TablePanelConfig config() const;
  void setConfig(const TablePanelConfig& config);
  void cloneConfigFrom(const TablePanelConfig& config);

  /**
   * @brief Bind channel + array field path (from drag or editor).
   */
  void setSource(const QString& channel, const QString& field_path);

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

 signals:
  void activated();
  void configChanged();
  void panelRemoveRequested();
  void panelExpandRequested();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelChangeRequested(const QString& object_name);

 protected:
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;

 private slots:
  void onTick();
  void onPathEdited();
  void onRowFilterEdited(const QString& text);
  void onTableContextMenu(const QPoint& pos);

 private:
  void applyChromeStyles();
  void resubscribe();
  void unsubscribe();
  void renderPayload(const std::string& payload);
  void clearTable(const QString& status_hint);
  void updateStatus(const QString& text);
  void applyRowFilter();
  void emitConfigChanged();

  std::string messageTypeForChannel(const QString& channel) const;

  common::VisualizationManager* manager_ = nullptr;
  TablePanelConfig config_;

  QLineEdit* path_edit_ = nullptr;
  QLineEdit* row_filter_edit_ = nullptr;
  QLabel* status_label_ = nullptr;
  QTableWidget* table_ = nullptr;
  QTimer* tick_timer_ = nullptr;
  QToolButton* expand_button_ = nullptr;

  integration::MessageQueue payload_queue_;
  std::uint64_t subscription_id_ = 0;
  std::string active_channel_;
  std::string active_message_type_;
};

}  // namespace table_panel
}  // namespace autoviz
