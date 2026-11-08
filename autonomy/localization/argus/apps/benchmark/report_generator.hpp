/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_APPS_BENCHMARK_REPORT_GENERATOR_HPP_
#define ARGUS_APPS_BENCHMARK_REPORT_GENERATOR_HPP_

#include <ostream>
#include <string>

#include "argus/evaluation/trajectory_evaluator.hpp"

namespace argus {
namespace apps {

/** @brief Format trajectory evaluation metrics for CLI / CI logs. */
class ReportGenerator {
public:
    void WriteText(const relocalization::TrajectoryEvalSummary& stats,
                   std::ostream& out) const;

    /** @brief Write a minimal Markdown summary (optional path). */
    bool WriteMarkdown(const relocalization::TrajectoryEvalSummary& stats,
                       const std::string& path) const;
};

}  // namespace apps
}  // namespace argus

#endif
