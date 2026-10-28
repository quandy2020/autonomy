#pragma once

#include <chrono>
#include <string>

namespace autonomy::localization::atlas {

/** @brief Nanosecond-resolution duration used by Atlas public utilities. */
using Duration = std::chrono::nanoseconds;
/** @brief System-clock timestamp with nanosecond storage. */
using TimePoint = std::chrono::time_point<std::chrono::system_clock, Duration>;

/** @brief Converts finite Unix-epoch seconds to a TimePoint. */
TimePoint TimePointFromSeconds(double seconds);
/** @brief Converts a TimePoint to fractional Unix-epoch seconds. */
double TimePointToSeconds(TimePoint time);
/** @brief Formats a timestamp as an ISO-like UTC string. */
std::string FormatTime(TimePoint time);

}  // namespace autonomy::localization::atlas
