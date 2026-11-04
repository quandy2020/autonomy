/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/foundation/motion_policy.hpp"

#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QStyle>
#include <QThread>
#include <atomic>
#include <optional>

namespace autoviz::widget {
namespace {

std::atomic<MotionPreference> g_preference{MotionPreference::kSystem};
std::atomic_bool g_system_reduced_motion{false};

std::optional<bool> ParseEnvironmentValue(QByteArray value) {
  value = value.trimmed().toLower();
  if (value == "1" || value == "true" || value == "yes" || value == "on" ||
      value == "reduce" || value == "reduced") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off" ||
      value == "full") {
    return false;
  }
  return std::nullopt;
}

std::optional<bool> EnvironmentPreference() {
  if (const std::optional<bool> value =
          ParseEnvironmentValue(qgetenv("AUTOVIZ_REDUCED_MOTION"));
      value.has_value()) {
    return value;
  }
  return ParseEnvironmentValue(qgetenv("PREFERS_REDUCED_MOTION"));
}

}  // namespace

MotionPreference MotionPolicy::preference() noexcept {
  return g_preference.load(std::memory_order_acquire);
}

void MotionPolicy::SetPreference(MotionPreference preference) noexcept {
  g_preference.store(preference, std::memory_order_release);
  if (preference == MotionPreference::kSystem) {
    RefreshSystemPreference();
  }
}

bool MotionPolicy::ReducedMotionEnabled() noexcept {
  switch (preference()) {
    case MotionPreference::kFullMotion:
      return false;
    case MotionPreference::kReducedMotion:
      return true;
    case MotionPreference::kSystem:
      RefreshSystemPreference();
      return g_system_reduced_motion.load(std::memory_order_acquire);
  }
  return false;
}

int MotionPolicy::ResolveDuration(int duration_ms) noexcept {
  if (duration_ms <= 0 || ReducedMotionEnabled()) {
    return 0;
  }
  return duration_ms;
}

void MotionPolicy::RefreshSystemPreference() noexcept {
  if (const std::optional<bool> environment = EnvironmentPreference();
      environment.has_value()) {
    g_system_reduced_motion.store(*environment, std::memory_order_release);
    return;
  }

  auto* application = qobject_cast<QApplication*>(QCoreApplication::instance());
  if (application == nullptr ||
      QThread::currentThread() != application->thread()) {
    return;
  }
  QStyle* style = application->style();
  if (style != nullptr) {
    const bool reduced =
        style->styleHint(QStyle::SH_Widget_Animate, nullptr, nullptr) == 0;
    g_system_reduced_motion.store(reduced, std::memory_order_release);
  }
}

}  // namespace autoviz::widget
