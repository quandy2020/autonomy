/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_config_io.hpp"

namespace autoviz {
namespace table_panel {

common::TablePanelPersistConfig ToPersistConfig(const QString& object_name,
                                                const TablePanelConfig& config) {
  common::TablePanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.channel = config.channel.toStdString();
  persist.field_path = config.field_path.toStdString();
  persist.row_filter = config.row_filter.toStdString();
  return persist;
}

TablePanelConfig FromPersistConfig(
    const common::TablePanelPersistConfig& persist) {
  TablePanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.channel = QString::fromStdString(persist.channel);
  config.field_path = QString::fromStdString(persist.field_path);
  config.row_filter = QString::fromStdString(persist.row_filter);
  if (config.title.trimmed().isEmpty()) {
    config.title = QStringLiteral("Table");
  }
  return config;
}

}  // namespace table_panel
}  // namespace autoviz
