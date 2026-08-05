/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <QString>

namespace autoviz {
namespace table {

struct TableColumn {
  QString name;
};

struct TableData {
  std::vector<TableColumn> columns;
  std::vector<std::vector<QString>> rows;
};

class TableFieldExtractor {
 public:
  static TableFieldExtractor& instance();

  std::optional<TableData> extract(const std::string& message_type,
                                   const std::string& payload,
                                   const std::string& array_path) const;

  std::optional<std::string> findFirstArrayPath(
      const std::string& message_type) const;

 private:
  TableFieldExtractor() = default;
};

}  // namespace table
}  // namespace autoviz
