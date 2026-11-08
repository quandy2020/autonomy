/*
 * Copyright 2026 The Openbot Authors
 */
/**
 * @file main.cpp
 * @brief Offline ATE vs TUM ground-truth (argus_benchmark).
 *
 * Usage:
 *   argus_benchmark <estimate.tum> <groundtruth.tum> [max_dt_sec] [--md out.md]
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "apps/benchmark/report_generator.hpp"
#include "argus/evaluation/trajectory_evaluator.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr
            << "Usage: argus_benchmark <estimate.tum> <gt.tum> [max_dt_sec] "
               "[--md out.md]\n"
            << "  TUM format: t tx ty tz qx qy qz qw\n";
        return 2;
    }
    const std::string est_path = argv[1];
    const std::string gt_path = argv[2];
    double max_dt = 0.05;
    std::string md_path;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--md" && i + 1 < argc) {
            md_path = argv[++i];
            continue;
        }
        if (a[0] != '-') {
            max_dt = std::atof(a.c_str());
        }
    }

    std::vector<argus::relocalization::TimedPose> est;
    std::vector<argus::relocalization::TimedPose> gt;
    if (!argus::relocalization::LoadTumTrajectory(est_path, &est)) {
        std::cerr << "failed to load estimate: " << est_path << "\n";
        return 1;
    }
    if (!argus::relocalization::LoadTumTrajectory(gt_path, &gt)) {
        std::cerr << "failed to load ground truth: " << gt_path << "\n";
        return 1;
    }
    const auto s =
        argus::relocalization::EvaluateTrajectories(est, gt, max_dt);
    if (!s.ok) {
        std::cerr << "eval failed: " << s.error << "\n";
        return 1;
    }

    argus::apps::ReportGenerator report;
    report.WriteText(s, std::cout);
    if (!md_path.empty() && !report.WriteMarkdown(s, md_path)) {
        std::cerr << "failed to write markdown: " << md_path << "\n";
        return 1;
    }
    return 0;
}
