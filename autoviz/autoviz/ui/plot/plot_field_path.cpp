/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_field_path.hpp"

#include <cmath>

#include <QRegularExpression>

namespace autoviz {
namespace plot {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

struct ModifierToken {
  QString name;
  bool has_arg = false;
  double arg = 0.0;
};

ModifierToken ParseModifierToken(const QString& token) {
  ModifierToken parsed;
  const int colon = token.indexOf(QLatin1Char(':'));
  const int paren = token.indexOf(QLatin1Char('('));
  if (colon > 0) {
    parsed.name = token.left(colon).trimmed().toLower();
    bool ok = false;
    parsed.arg = token.mid(colon + 1).trimmed().toDouble(&ok);
    parsed.has_arg = ok;
    return parsed;
  }
  if (paren > 0 && token.endsWith(QLatin1Char(')'))) {
    parsed.name = token.left(paren).trimmed().toLower();
    bool ok = false;
    parsed.arg = token.mid(paren + 1, token.size() - paren - 2).trimmed().toDouble(&ok);
    parsed.has_arg = ok;
    return parsed;
  }
  parsed.name = token.trimmed().toLower();
  return parsed;
}

bool IsUnaryMath(const QString& name) {
  return name == QStringLiteral("abs") || name == QStringLiteral("log") ||
         name == QStringLiteral("log2") || name == QStringLiteral("log10") ||
         name == QStringLiteral("sqrt") || name == QStringLiteral("negative") ||
         name == QStringLiteral("sign") || name == QStringLiteral("degrees") ||
         name == QStringLiteral("radians") || name == QStringLiteral("floor") ||
         name == QStringLiteral("ceil") || name == QStringLiteral("round") ||
         name == QStringLiteral("sin") || name == QStringLiteral("cos") ||
         name == QStringLiteral("tan");
}

bool IsBinaryMath(const QString& name) {
  return name == QStringLiteral("add") || name == QStringLiteral("sub") ||
         name == QStringLiteral("mul") || name == QStringLiteral("div");
}

bool IsStateful(const QString& name) {
  return name == QStringLiteral("derivative") || name == QStringLiteral("delta") ||
         name == QStringLiteral("timedelta");
}

double ApplyUnary(const QString& name, double value) {
  if (name == QStringLiteral("abs")) {
    return std::abs(value);
  }
  if (name == QStringLiteral("log")) {
    return value > 0.0 ? std::log(value) : 0.0;
  }
  if (name == QStringLiteral("log2")) {
    return value > 0.0 ? std::log2(value) : 0.0;
  }
  if (name == QStringLiteral("log10")) {
    return value > 0.0 ? std::log10(value) : 0.0;
  }
  if (name == QStringLiteral("sqrt")) {
    return value >= 0.0 ? std::sqrt(value) : 0.0;
  }
  if (name == QStringLiteral("negative")) {
    return -value;
  }
  if (name == QStringLiteral("sign")) {
    if (value > 0.0) {
      return 1.0;
    }
    if (value < 0.0) {
      return -1.0;
    }
    return 0.0;
  }
  if (name == QStringLiteral("degrees")) {
    return value * kRadToDeg;
  }
  if (name == QStringLiteral("radians")) {
    return value * kDegToRad;
  }
  if (name == QStringLiteral("floor")) {
    return std::floor(value);
  }
  if (name == QStringLiteral("ceil")) {
    return std::ceil(value);
  }
  if (name == QStringLiteral("round")) {
    return std::round(value);
  }
  if (name == QStringLiteral("sin")) {
    return std::sin(value);
  }
  if (name == QStringLiteral("cos")) {
    return std::cos(value);
  }
  if (name == QStringLiteral("tan")) {
    return std::tan(value);
  }
  return value;
}

double ApplyBinary(const QString& name, double value, double arg) {
  if (name == QStringLiteral("add")) {
    return value + arg;
  }
  if (name == QStringLiteral("sub")) {
    return value - arg;
  }
  if (name == QStringLiteral("mul")) {
    return value * arg;
  }
  if (name == QStringLiteral("div")) {
    return std::abs(arg) > 1e-15 ? value / arg : 0.0;
  }
  return value;
}

}  // namespace

ParsedFieldPath ParseFieldPath(const QString& field_path) {
  ParsedFieldPath parsed;
  const int marker = field_path.indexOf(QStringLiteral(".@"));
  if (marker < 0) {
    parsed.base_path = field_path;
    return parsed;
  }
  parsed.base_path = field_path.left(marker);
  QString rest = field_path.mid(marker + 2);
  while (!rest.isEmpty()) {
    const int next = rest.indexOf(QStringLiteral(".@"));
    if (next < 0) {
      parsed.modifiers.push_back(rest);
      break;
    }
    parsed.modifiers.push_back(rest.left(next));
    rest = rest.mid(next + 2);
  }
  return parsed;
}

bool FieldPathHasArrayExpand(const QString& field_path) {
  static const QRegularExpression kExpand(QStringLiteral(R"(\[\s*:?\s*\])"));
  return kExpand.match(field_path).hasMatch();
}

bool FieldPathHasNormModifier(const QStringList& modifiers) {
  for (const QString& token : modifiers) {
    if (ParseModifierToken(token).name == QStringLiteral("norm")) {
      return true;
    }
  }
  return false;
}

QStringList StripNormModifier(const QStringList& modifiers) {
  QStringList out;
  out.reserve(modifiers.size());
  for (const QString& token : modifiers) {
    if (ParseModifierToken(token).name == QStringLiteral("norm")) {
      continue;
    }
    out.push_back(token);
  }
  return out;
}

double ApplyPlotModifiers(double raw_value, double timestamp_sec,
                          const QStringList& modifiers,
                          double* last_raw_value, double* last_timestamp_sec,
                          bool* has_last_sample) {
  double result = raw_value;
  double remember_value = raw_value;
  for (int i = 0; i < modifiers.size(); ++i) {
    ModifierToken token = ParseModifierToken(modifiers[i]);
    if (token.name == QStringLiteral("norm")) {
      continue;
    }
    if (IsUnaryMath(token.name)) {
      result = ApplyUnary(token.name, result);
      remember_value = result;
      continue;
    }
    if (IsBinaryMath(token.name)) {
      double arg = token.arg;
      bool have_arg = token.has_arg;
      if (!have_arg && i + 1 < modifiers.size()) {
        const ModifierToken next = ParseModifierToken(modifiers[i + 1]);
        bool ok = false;
        const double parsed = next.name.toDouble(&ok);
        if (ok && !next.has_arg && !IsUnaryMath(next.name) &&
            !IsBinaryMath(next.name) && !IsStateful(next.name) &&
            next.name != QStringLiteral("norm")) {
          arg = parsed;
          have_arg = true;
          ++i;
        }
      }
      if (have_arg) {
        result = ApplyBinary(token.name, result, arg);
        remember_value = result;
      }
      continue;
    }
    if (token.name == QStringLiteral("derivative")) {
      remember_value = result;
      if (last_raw_value != nullptr && last_timestamp_sec != nullptr &&
          has_last_sample != nullptr && *has_last_sample) {
        const double dt = timestamp_sec - *last_timestamp_sec;
        result = dt > 1e-9 ? (result - *last_raw_value) / dt : 0.0;
      } else {
        result = 0.0;
      }
      continue;
    }
    if (token.name == QStringLiteral("delta")) {
      remember_value = result;
      if (last_raw_value != nullptr && has_last_sample != nullptr &&
          *has_last_sample) {
        result = result - *last_raw_value;
      } else {
        result = 0.0;
      }
      continue;
    }
    if (token.name == QStringLiteral("timedelta")) {
      if (last_timestamp_sec != nullptr && has_last_sample != nullptr &&
          *has_last_sample) {
        result = timestamp_sec - *last_timestamp_sec;
      } else {
        result = 0.0;
      }
      continue;
    }
  }
  if (last_raw_value != nullptr) {
    *last_raw_value = remember_value;
  }
  if (last_timestamp_sec != nullptr) {
    *last_timestamp_sec = timestamp_sec;
  }
  if (has_last_sample != nullptr) {
    *has_last_sample = true;
  }
  return result;
}

}  // namespace plot
}  // namespace autoviz
