/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>

#include <QString>

namespace autoviz {
namespace indicator {

enum class IndicatorValueKind {
  kNumeric = 0,
  kString = 1,
  kBoolean = 2,
};

struct IndicatorFieldValue {
  IndicatorValueKind kind = IndicatorValueKind::kNumeric;
  double number = 0.0;
  QString text;
  bool boolean = false;
};

class IndicatorFieldExtractor {
 public:
  static IndicatorFieldExtractor& instance();

  std::optional<IndicatorFieldValue> extract(const std::string& message_type,
                                             const std::string& payload,
                                             const std::string& field_path) const;

 private:
  IndicatorFieldExtractor() = default;
};

QString FormatIndicatorFieldValue(const IndicatorFieldValue& value);

}  // namespace indicator
}  // namespace autoviz
