/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/tf/tf_display_utils.hpp"

#include <algorithm>
#include <regex>

namespace autoviz {
namespace display {
namespace {

bool TryCompile(const std::string& pattern, std::regex* out,
                std::string* error_out, const char* which) {
  if (pattern.empty()) {
    return false;
  }
  try {
    *out = std::regex(pattern, std::regex::ECMAScript);
    return true;
  } catch (const std::regex_error& e) {
    if (error_out != nullptr) {
      *error_out = std::string("Invalid ") + which + " regex: " + e.what();
    }
    return false;
  }
}

// RViz FrameInfo::lerpColor(start, end, t) = start * t + end * (1 - t).
QColor LerpColor(const QColor& start, const QColor& end, double t) {
  const double u = std::clamp(t, 0.0, 1.0);
  const double v = 1.0 - u;
  return QColor::fromRgbF(
      static_cast<float>(start.redF() * u + end.redF() * v),
      static_cast<float>(start.greenF() * u + end.greenF() * v),
      static_cast<float>(start.blueF() * u + end.blueF() * v),
      static_cast<float>(start.alphaF() * u + end.alphaF() * v));
}

}  // namespace

std::vector<std::string> FilterTfFrameNames(
    const std::vector<std::string>& frames, const std::string& whitelist_regex,
    const std::string& blacklist_regex, std::string* error_out) {
  if (error_out != nullptr) {
    error_out->clear();
  }

  std::regex white;
  std::regex black;
  const bool use_white =
      TryCompile(whitelist_regex, &white, error_out, "whitelist");
  std::string black_error;
  const bool use_black =
      TryCompile(blacklist_regex, &black, &black_error, "blacklist");
  if (use_black && error_out != nullptr && error_out->empty() &&
      !black_error.empty()) {
    *error_out = black_error;
  } else if (!use_black && error_out != nullptr && error_out->empty() &&
             !black_error.empty()) {
    *error_out = black_error;
  }

  std::vector<std::string> out;
  out.reserve(frames.size());
  for (const std::string& frame : frames) {
    if (use_white && !std::regex_search(frame, white)) {
      continue;
    }
    if (use_black && std::regex_search(frame, black)) {
      continue;
    }
    out.push_back(frame);
  }
  return out;
}

TfAgeVisual TfAgeVisualForTimeout(double age_sec, double timeout_sec,
                                  const QColor& base_rgb) {
  TfAgeVisual visual;
  visual.color = base_rgb;
  visual.alpha = 1.f;
  visual.visible = true;

  if (timeout_sec <= 0.0 || age_sec < 0.0) {
    return visual;
  }
  if (age_sec > timeout_sec) {
    visual.visible = false;
    visual.alpha = 0.f;
    return visual;
  }

  // Match rviz_default_plugins::FrameInfo::updateColorForAge.
  const double one_third = timeout_sec * (1.0 / 3.0);
  if (age_sec <= one_third) {
    return visual;
  }

  const QColor grey = QColor::fromRgbF(0.7f, 0.7f, 0.7f, 1.f);
  if (age_sec <= one_third * 2.0) {
    const double t =
        std::max(0.0, (one_third * 2.0 - age_sec) / std::max(1e-9, one_third));
    visual.color = LerpColor(base_rgb, grey, t);
    visual.alpha = static_cast<float>(visual.color.alphaF());
    return visual;
  }

  const double a =
      std::max(0.0, (timeout_sec - age_sec) / std::max(1e-9, one_third));
  visual.color = grey;
  visual.alpha = static_cast<float>(std::clamp(a, 0.0, 1.0));
  visual.color.setAlphaF(visual.alpha);
  return visual;
}

}  // namespace display
}  // namespace autoviz
