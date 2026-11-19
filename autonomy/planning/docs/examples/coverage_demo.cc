// Documentation example for the Autonomy Fields2Cover implementation.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "autonomy/planning/planner/fields2cover/fields2cover.hpp"

namespace f2c = autonomy::planning::planner::fields2cover;
namespace fs = std::filesystem;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void WriteText(const fs::path& path, const std::string& text) {
  std::ofstream out(path);
  if (!(out << text)) throw std::runtime_error("Cannot write " + path.string());
}

f2c::F2CSwaths Flatten(const f2c::F2CRoute& route) {
  f2c::F2CSwaths result;
  for (const auto& group : route.GetVectorSwaths()) result.Append(group);
  return result;
}

void WriteSwaths(const fs::path& path, const f2c::F2CSwaths& swaths) {
  std::ofstream out(path);
  out << std::setprecision(17) << "id,x1,y1,x2,y2,width\n";
  for (const auto& swath : swaths) {
    const auto start = swath.StartPoint();
    const auto end = swath.EndPoint();
    out << swath.GetId() << ',' << start.GetX() << ',' << start.GetY() << ','
        << end.GetX() << ',' << end.GetY() << ',' << swath.GetWidth() << '\n';
  }
  Require(static_cast<bool>(out), "Cannot write swaths");
}

void Describe(std::ostream& out, const std::string& name, const f2c::F2CPath& path) {
  Require(path.size() > 0, name + ": empty path");
  size_t backward = 0;
  for (const auto& state : path) {
    Require(std::isfinite(state.point.GetX()) && std::isfinite(state.point.GetY()) &&
            std::isfinite(state.angle) && std::isfinite(state.len) && state.len >= 0,
            name + ": invalid state");
    if (state.dir == f2c::types::PathDirection::BACKWARD) ++backward;
  }
  out << name << ": states=" << path.size() << ", length_m=" << path.Length()
      << ", backward_states=" << backward << '\n';
}

int main(int argc, char** argv) {
  try {
    Require(argc <= 3, "Usage: coverage_demo [output_directory] [field.geojson]");
    const fs::path output = argc > 1 ? argv[1] : "fields2cover-output";
    fs::create_directories(output);

    f2c::F2CCell cell;
    if (argc > 2) {
      cell = f2c::Parser::ImportCellJson(argv[2]);
    } else {
      cell.ImportFromWkt(
          "POLYGON ((0 0,80 0,80 60,0 60,0 0),"
          "(32 25,32 31,38 31,38 25,32 25))");
    }
    Require(!cell.IsEmpty() && cell.Area() > 0, "Invalid or empty field");
    const f2c::F2CCells field(cell);
    f2c::F2CRobot robot(1.5, 3.0);
    robot.SetMinTurningRadius(1.0);
    robot.SetCruiseVel(1.0);
    robot.SetTurnVel(0.5);

    // Stage API: explicitly choose 3 m of headland and retain original holes.
    f2c::hg::ConstHL headland;
    const auto core = headland.GenerateHeadlands(field, 3.0);
    Require(!core.IsEmpty() && core.Area() > 0, "Headland removed the entire core");
    f2c::sg::BruteForce generator;
    f2c::obj::NSwathModified objective;
    const auto generated = generator.GenerateBestSwaths(objective, robot.GetCovWidth(), core);
    Require(generated.SizeTotal() > 0, "No generated swaths");
    const auto rings = headland.GenerateHeadlandSwaths(field, robot.GetWidth(), 2);
    f2c::rp::RoutePlannerBase route_planner;
    auto staged_route = route_planner.GenRoute(rings.at(1), generated);
    staged_route.SetField(field);
    Require(Flatten(staged_route).size() == generated.SizeTotal(), "Stage route lost swaths");
    f2c::pp::DubinsCurves staged_turn;
    const auto staged_path = f2c::pp::PathPlanning::PlanPath(robot, staged_route, staged_turn);

    // Top-level route uses the same 1.5 m * 2 = 3 m headland.
    f2c::Options options;
    options.hg_swaths = 2;
    auto route = f2c::PlanCovRoute(robot, field, options);
    Require(route.GetField().has_value(), "Top-level route did not retain the field");
    const auto all_swaths = Flatten(route);
    Require(all_swaths.size() == generated.SizeTotal(), "Top-level route lost swaths");
    const double coverage = f2c::obj::FieldCoverage().ComputeCost(core, all_swaths);
    Require(std::isfinite(coverage) && coverage >= 0.95, "Core coverage is below 95%");

    // Reuse the same route so only the turning algorithm changes.
    f2c::pp::DubinsCurves dubins;
    f2c::pp::ReedsSheppCurves reeds_shepp;
    const auto forward = f2c::pp::PathPlanning::PlanPath(robot, route, dubins);
    const auto reversible = f2c::pp::PathPlanning::PlanPath(robot, route, reeds_shepp);
    options.pp_alg = f2c::PPAlg::DUBINS;
    const auto top_level = f2c::PlanCovPath(robot, field, options);

    // Field(true) returns local coordinates; move them back to the original frame.
    const f2c::F2CPoint reference_offset(120.0, 240.0);
    const f2c::F2CField local_field(field + reference_offset, "work-area");
    auto local_result = f2c::PlanCovPath(robot, local_field, options, true);
    local_result.MoveTo(local_field.GetRefPoint());
    Require(local_result.AtStart().Distance(top_level.AtStart() + reference_offset) < 0.01,
            "Field reference point restoration disagrees with Cells API");

    WriteText(output / "field.wkt", field.ExportToWkt() + "\n");
    WriteText(output / "core.wkt", core.ExportToWkt() + "\n");
    WriteSwaths(output / "swaths.csv", all_swaths);
    WriteText(output / "dubins.path.txt", forward.SerializePath(12));
    WriteText(output / "reeds_shepp.path.txt", reversible.SerializePath(12));
    WriteText(output / "top-level-dubins.path.txt", top_level.SerializePath(12));
    std::ofstream summary(output / "summary.txt");
    summary << std::fixed << std::setprecision(6)
            << "Coordinates: local metric plane; headland=3 m; width=1.5 m; "
               "coverage_width=3 m; minimum_radius=1 m\n"
            << "field_area_m2=" << field.Area() << ", core_area_m2=" << core.Area()
            << ", swaths=" << all_swaths.size() << ", core_coverage_pct=" << coverage * 100 << '\n';
    Describe(summary, "stage Dubins", staged_path);
    Describe(summary, "Dubins", forward);
    Describe(summary, "Reeds-Shepp", reversible);
    Describe(summary, "top-level Dubins", top_level);
    Require(static_cast<bool>(summary), "Cannot write summary");
    summary.close();
    std::cout << "Example completed. Outputs: " << output << '\n';
    std::cout << std::ifstream(output / "summary.txt").rdbuf();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Coverage example failed: " << error.what() << '\n';
    return 1;
  }
}
