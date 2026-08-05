/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {
namespace table {

struct TablePanelConfig {
  QString title;
  QString channel;
  QString array_path;
};

TablePanelConfig DefaultTablePanelConfig();

}  // namespace table
}  // namespace autoviz
