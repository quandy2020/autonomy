/*
 * Copyright 2026 The Openbot Authors
 */
#include "apps/benchmark/report_generator.hpp"

#include <fstream>

namespace argus {
namespace apps {

void ReportGenerator::WriteText(
    const relocalization::TrajectoryEvalSummary& s, std::ostream& out) const {
    out << "matched_pairs=" << s.matched_pairs << "\n"
        << "ate_rmse_m=" << s.ate_rmse_m << "\n"
        << "ate_mean_m=" << s.ate_mean_m << "\n"
        << "ate_max_m=" << s.ate_max_m << "\n"
        << "rot_rmse_rad=" << s.rot_rmse_rad << "\n"
        << "rot_mean_rad=" << s.rot_mean_rad << "\n";
}

bool ReportGenerator::WriteMarkdown(
    const relocalization::TrajectoryEvalSummary& s,
    const std::string& path) const {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "# Argus trajectory benchmark\n\n"
        << "| metric | value |\n|--------|-------|\n"
        << "| matched_pairs | " << s.matched_pairs << " |\n"
        << "| ate_rmse_m | " << s.ate_rmse_m << " |\n"
        << "| ate_mean_m | " << s.ate_mean_m << " |\n"
        << "| ate_max_m | " << s.ate_max_m << " |\n"
        << "| rot_rmse_rad | " << s.rot_rmse_rad << " |\n"
        << "| rot_mean_rad | " << s.rot_mean_rad << " |\n";
    return true;
}

}  // namespace apps
}  // namespace argus
