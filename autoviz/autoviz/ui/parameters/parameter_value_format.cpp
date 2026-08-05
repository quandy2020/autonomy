/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/parameters/parameter_value_format.hpp"

#include <QObject>

#include <cctype>

namespace autoviz {
namespace parameters_panel {
namespace {

using autolink::Parameter;
using autolink::proto::ParamType;

std::optional<Parameter> MakeParameter(const Parameter& value) {
  return std::optional<Parameter>(Parameter(value));
}

std::string LowercaseCopy(const std::string& value) {
  std::string lower = value;
  for (char& ch : lower) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return lower;
}

}  // namespace

bool IsEditableParameterType(ParamType type) {
  return type == ParamType::BOOL || type == ParamType::INT ||
         type == ParamType::DOUBLE || type == ParamType::STRING;
}

QString FormatParameterTypeName(const Parameter& parameter) {
  switch (parameter.Type()) {
    case ParamType::BOOL:
      return QStringLiteral("bool");
    case ParamType::INT:
      return QStringLiteral("int64");
    case ParamType::DOUBLE:
      return QStringLiteral("double");
    case ParamType::STRING:
      return QStringLiteral("string");
    case ParamType::PROTOBUF: {
      const QString full_name = QString::fromStdString(parameter.TypeName());
      return full_name.isEmpty() ? QStringLiteral("protobuf") : full_name;
    }
    default:
      return QStringLiteral("unknown");
  }
}

QString FormatParameterValue(const Parameter& parameter) {
  switch (parameter.Type()) {
    case ParamType::BOOL:
      return parameter.AsBool() ? QStringLiteral("true")
                                : QStringLiteral("false");
    case ParamType::INT:
      return QString::number(parameter.AsInt64());
    case ParamType::DOUBLE:
      return QString::number(parameter.AsDouble(), 'g', 12);
    case ParamType::STRING:
      return QString::fromStdString(parameter.AsString());
    case ParamType::PROTOBUF:
      return QString::fromStdString(parameter.DebugString());
    default:
      return QStringLiteral("—");
  }
}

std::optional<Parameter> ParseEditedParameterValue(
    const Parameter& template_param, const QString& text,
    QString* error_message) {
  const std::string name = template_param.Name();
  const ParamType type = template_param.Type();
  const std::string trimmed = text.trimmed().toStdString();
  if (trimmed.empty() && type != ParamType::STRING) {
    if (error_message != nullptr) {
      *error_message = QObject::tr("Value cannot be empty.");
    }
    return std::nullopt;
  }

  switch (type) {
    case ParamType::BOOL: {
      const std::string lower = LowercaseCopy(trimmed);
      if (lower == "true" || lower == "1") {
        return MakeParameter(Parameter(name, true));
      }
      if (lower == "false" || lower == "0") {
        return MakeParameter(Parameter(name, false));
      }
      if (error_message != nullptr) {
        *error_message = QObject::tr("Expected true or false.");
      }
      return std::nullopt;
    }
    case ParamType::INT: {
      try {
        size_t index = 0;
        const long long value = std::stoll(trimmed, &index);
        if (index != trimmed.size()) {
          throw std::invalid_argument("suffix");
        }
        return MakeParameter(Parameter(name, static_cast<int64_t>(value)));
      } catch (...) {
        if (error_message != nullptr) {
          *error_message = QObject::tr("Expected an integer value.");
        }
        return std::nullopt;
      }
    }
    case ParamType::DOUBLE: {
      try {
        size_t index = 0;
        const double value = std::stod(trimmed, &index);
        if (index != trimmed.size()) {
          throw std::invalid_argument("suffix");
        }
        return MakeParameter(Parameter(name, value));
      } catch (...) {
        if (error_message != nullptr) {
          *error_message = QObject::tr("Expected a numeric value.");
        }
        return std::nullopt;
      }
    }
    case ParamType::STRING:
      return MakeParameter(Parameter(name, trimmed));
    default:
      if (error_message != nullptr) {
        *error_message = QObject::tr("This parameter type is read-only.");
      }
      return std::nullopt;
  }
}

}  // namespace parameters_panel
}  // namespace autoviz
