/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight JSON field extractors for OrbisView control / ingest paths.
 */

#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace orbisview {
namespace util {

inline std::string ExtractJsonString(const std::string& text,
                                     const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return {};
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return {};
  pos = text.find('"', pos + 1);
  if (pos == std::string::npos) return {};
  const auto end = text.find('"', pos + 1);
  if (end == std::string::npos) return {};
  return text.substr(pos + 1, end - pos - 1);
}

inline std::string ExtractJsonOp(const std::string& text) {
  return ExtractJsonString(text, "op");
}

inline double ExtractJsonNumber(const std::string& text, const std::string& key,
                                double fallback) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return fallback;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return fallback;
  ++pos;
  while (pos < text.size() &&
         (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\n')) {
    ++pos;
  }
  try {
    size_t consumed = 0;
    const double v = std::stod(text.substr(pos), &consumed);
    return consumed > 0 ? v : fallback;
  } catch (...) {
    return fallback;
  }
}

inline bool ExtractJsonBool(const std::string& text, const std::string& key,
                            bool fallback) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return fallback;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return fallback;
  const auto slice = text.substr(pos + 1, 8);
  if (slice.find("true") != std::string::npos) return true;
  if (slice.find("false") != std::string::npos) return false;
  return fallback;
}

inline std::vector<std::pair<double, double>> ExtractJsonPoints(
    const std::string& text) {
  std::vector<std::pair<double, double>> out;
  auto pos = text.find('[');
  if (pos == std::string::npos) return out;
  while (true) {
    pos = text.find('{', pos);
    if (pos == std::string::npos) break;
    const auto end = text.find('}', pos);
    if (end == std::string::npos) break;
    const auto obj = text.substr(pos, end - pos + 1);
    out.emplace_back(ExtractJsonNumber(obj, "x", 0.0),
                     ExtractJsonNumber(obj, "y", 0.0));
    pos = end + 1;
  }
  return out;
}

}  // namespace util
}  // namespace orbisview
}  // namespace autonomy
