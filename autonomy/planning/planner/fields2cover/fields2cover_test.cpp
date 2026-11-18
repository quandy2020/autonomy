/* Copyright 2026 Autodriver contributors. Licensed under Apache-2.0. */
#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>
#include "autonomy/planning/planner/fields2cover/fields2cover.hpp"

namespace autonomy::planning::planner::fields2cover {
namespace {
void Require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}
void Near(double actual, double expected, double tolerance,
          const std::string& message) {
    Require(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance,
            message + ": actual=" + std::to_string(actual) +
                ", expected=" + std::to_string(expected));
}

types::LinearRing Ring(std::initializer_list<types::Point> points) {
    types::LinearRing ring;
    for (const auto& p : points)
        ring.AddPoint(p);
    ring.CloseRing();
    return ring;
}
types::Cell Square(double x = 0) {
    return types::Cell(Ring({{x, 0}, {x + 10, 0}, {x + 10, 10}, {x, 10}}));
}
types::Cell WithHole() {
    auto cell = Square();
    cell.AddRing(Ring({{4, 4}, {4, 6}, {6, 6}, {6, 4}}));
    return cell;
}

void InsetHasRequestedEdgeClearance() {
    auto inset = hg::ConstHL().GenerateHeadlands(types::Cells(Square()), 1);
    Near(inset.Area(), 64, 1e-6, "numeric comparison");
    Require(inset.size() == 1u, "expected equal");
    Require(!(inset.IsPointIn({.5, 5})), "expected false");
    Require(inset.IsPointIn({1.5, 5}), "expected true");
}
void SlopingInsetPreservesPrecision() {
    types::Cell cell(Ring({{0, 0}, {60, 0}, {50, 40}, {5, 35}}));
    const auto inset = types::Cells(cell).buffer(-3);
    Near(inset.Area(), 1466.15197771884, 1e-7, "sloping inset area");
}
void InsetPreservesAndExpandsHoles() {
    auto inset = types::Cells(WithHole()).buffer(-1);
    Near(inset.Area(), 48, 1e-6, "numeric comparison");
    Require(!(inset.IsPointIn({3.5, 5})), "expected false");
    Require(inset.IsPointIn({2, 5}), "expected true");
}
void CollapsedInsetIsEmpty() {
    Require(types::Cells(Square()).buffer(-6).IsEmpty(), "expected true");
}
void HoleContainmentAndDistance() {
    auto cell = WithHole();
    Require(!(cell.IsPointIn({5, 5})), "expected false");
    Require(cell.IsPointIn({2, 5}), "expected true");
    Require(cell.IsPointInBorder({4, 5}), "expected true");
    Near(types::Point(5, 5).Distance(cell), 1, 1e-9, "numeric comparison");
}
void LineClippingSplitsAtHole() {
    auto clips =
        WithHole().GetLinesInside(types::LineString({{-1, 5}, {11, 5}}));
    Require(clips.size() == 2u, "expected equal");
    double length = 0;
    for (const auto& line : clips)
        length += line.Length();
    Near(length, 8, 1e-6, "numeric comparison");
}
void DistanceToCollinearPolylineDoesNotConstructPolygon() {
    types::LineString line({{0, 0}, {0, 2}, {0, 4}, {0, 6}});
    Near(types::Point(3, 3).Distance(line), 3, 1e-9, "numeric comparison");
    Near(line.Distance(types::Point(3, 3)), 3, 1e-9, "numeric comparison");
}
void PolygonBooleanAreas() {
    auto a = types::Cells(Square()), b = types::Cells(Square(5));
    Near(a.intersection(b).Area(), 50, 1e-6, "numeric comparison");
    Near(a.UnionOp(b).Area(), 150, 1e-6, "numeric comparison");
    Near(a.difference(b).Area(), 50, 1e-6, "numeric comparison");
}
void BooleanOperationsPreserveDisconnectedComponents() {
    auto joined = types::Cells(Square()).UnionOp(Square(20));
    Require(joined.size() == 2u, "expected equal");
    Near(joined.Area(), 200, 1e-6, "numeric comparison");
    Near(joined.buffer(1).Area(), 288, 1e-6, "numeric comparison");
    types::Cell strip(Ring({{4, -1}, {6, -1}, {6, 11}, {4, 11}}));
    auto split = types::Cells(Square()).difference(strip);
    Require(split.size() == 2u, "expected equal");
    Near(split.Area(), 80, 1e-6, "numeric comparison");
}
void NonconvexIntersectionAndCoverage() {
    types::Cell cell(Ring({{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}}));
    types::Cell clip(Ring({{1, 1}, {5, 1}, {5, 5}, {1, 5}}));
    Near(types::Cells(cell).intersection(clip).Area(), 7, 1e-6,
         "numeric comparison");
    types::Swaths swaths;
    swaths.emplace_back(types::LineString({{0, 1}, {6, 1}}), 2);
    swaths.emplace_back(types::LineString({{0, 3}, {2, 3}}), 2);
    swaths.emplace_back(types::LineString({{0, 5}, {2, 5}}), 2);
    Near(obj::FieldCoverage().ComputeCost(cell, swaths), 1, 1e-6,
         "numeric comparison");
}
void CoverageCountsAllBufferedComponents() {
    types::Swaths swaths;
    swaths.emplace_back(types::LineString({{0, 1}, {10, 1}}), 2);
    swaths.emplace_back(types::LineString({{0, 9}, {10, 9}}), 2);
    Near(obj::FieldCoverage().ComputeCost(Square(), swaths), .4, 1e-6,
         "numeric comparison");
}
void WktRoundTripPreservesPrecisionAndHole() {
    types::Point point(1.23456789012345, 9.87654321098765), restored;
    restored.ImportFromWkt(point.ExportToWkt());
    Near(restored.GetX(), point.GetX(), 1e-14, "numeric comparison");
    Near(restored.GetY(), point.GetY(), 1e-14, "numeric comparison");
    types::Cell cell;
    cell.ImportFromWkt(WithHole().ExportToWkt());
    Near(cell.Area(), 96, 1e-6, "numeric comparison");
    Require(!(cell.IsPointIn({5, 5})), "expected false");
}

void CheckPath(const types::Path& path, const types::Cell& field) {
    Require(path.size() > 0u, "expected greater");
    const auto allowed = types::Cells(field).buffer(.01);
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& state = path[i];
        Require(std::isfinite(state.len), "expected true");
        Require(std::isfinite(state.angle), "expected true");
        Require(std::isfinite(state.point.GetX()), "expected true");
        Require(std::isfinite(state.point.GetY()), "expected true");
        const types::LineString segment(state.point, state.AtEnd());
        Near(types::MultiLineString::intersection(segment, allowed).Length(),
             segment.Length(), 1e-5, "segment " + std::to_string(i));
        if (i + 1 == path.size())
            continue;
        const auto& next = path[i + 1];
        Require(state.AtEnd().Distance(next.point) < .05,
                "join " + std::to_string(i));
        if (state.dir == next.dir && state.len > 1e-5) {
            const double delta = std::atan2(std::sin(next.angle - state.angle),
                                            std::cos(next.angle - state.angle));
            Require(std::fabs(delta) / state.len <= 1.02,
                    "curvature at " + std::to_string(i));
        }
    }
}

void ShortEntrySpurIsRoundedForBothAlgorithms() {
    types::Robot robot(1.5, 3);
    robot.SetMinTurningRadius(1);
    types::MultiPoint track;
    for (const auto& p : std::vector<types::Point>{
             {3, 13.5}, {2.25, 13.5}, {2.25, 34.5}, {3, 34.5}})
        track.AddPoint(p);
    types::Cell field(Ring({{0, 0}, {60, 0}, {60, 40}, {0, 40}}));
    pp::DubinsCurves dubins;
    pp::ReedsSheppCurves reeds_shepp;
    CheckPath(pp::PathPlanning::PlanPathForConnection(
                  robot, {3, 13.5}, M_PI, track, {3, 34.5}, 0, dubins),
              field);
    CheckPath(pp::PathPlanning::PlanPathForConnection(
                  robot, {3, 13.5}, M_PI, track, {3, 34.5}, 0, reeds_shepp),
              field);
}

void VerifyScenario(const std::string& name) {
    types::Cell cell;
    if (name == "trapezoid")
        cell = types::Cell(Ring({{0, 0}, {60, 0}, {50, 40}, {5, 35}}));
    else if (name == "concave")
        cell = types::Cell(Ring({{0, 0},
                                 {60, 0},
                                 {60, 40},
                                 {40, 40},
                                 {40, 20},
                                 {20, 20},
                                 {20, 40},
                                 {0, 40}}));
    else {
        cell = types::Cell(Ring({{0, 0}, {60, 0}, {60, 40}, {0, 40}}));
        if (name == "obstacle")
            cell.AddRing(Ring({{25, 14}, {25, 26}, {35, 26}, {35, 14}}));
    }
    types::Robot robot(1.5, 3);
    robot.SetMinTurningRadius(1);
    Options options;
    options.hg_swaths = 2;
    auto mainland =
        hg::ConstHL().GenerateHeadlandArea(types::Cells(cell), 1.5, 2);
    Require(!(mainland.IsEmpty()), "expected false");
    sg::BruteForce generator;
    obj::NSwathModified objective;
    size_t count = 0;
    for (const auto& piece : mainland) {
        auto swaths = generator.GenerateBestSwaths(objective, 3, piece);
        count += swaths.size();
        Require(obj::FieldCoverage().ComputeCost(piece, swaths) >= .95,
                "coverage below threshold");
        for (const auto& swath : swaths)
            Near(cell.GetLinesInside(swath.GetPath()).Length(), swath.Length(),
                 1e-5, "numeric comparison");
    }
    auto route = PlanCovRoute(robot, cell, options);
    size_t route_count = 0;
    for (const auto& group : route.GetVectorSwaths())
        route_count += group.size();
    Require(route_count == count, "expected equal");
    pp::DubinsCurves dubins;
    pp::ReedsSheppCurves reeds_shepp;
    CheckPath(pp::PathPlanning::PlanPath(robot, route, dubins), cell);
    CheckPath(pp::PathPlanning::PlanPath(robot, route, reeds_shepp), cell);
    Require(PlanCovPath(robot, cell, options).size() > 0u,
            "expected nonempty top-level path");
}

void RotatedTouchingStripsHaveExactCoverage() {
    types::Cell cell(Ring({{0,0},{60,0},{60,30},{0,30}}));
    types::Swaths swaths;
    for (int i = 0; i < 10; ++i)
        swaths.emplace_back(types::LineString({{0,1.5 + 3*i},{60,1.5 + 3*i}}),3);
    for (double angle : {.013, .417, 1.111, 2.71}) {
        auto rotated = types::Point(0,0).RotateFromPoint(angle, cell);
        types::Swaths rotated_swaths;
        for (const auto& swath : swaths)
            rotated_swaths.emplace_back(types::Point(0,0).RotateFromPoint(angle, swath.GetPath()),3);
        Near(obj::FieldCoverage().ComputeCost(rotated, rotated_swaths),1.0,1e-6,
             "rotated touching footprints must retain their total area");
    }
}
void OverlappingFootprintsAreCountedOnce() {
    types::Swaths swaths;
    swaths.emplace_back(types::LineString({{0,3},{10,3}}),4);
    swaths.emplace_back(types::LineString({{0,5},{10,5}}),4);
    Near(obj::FieldCoverage().ComputeCost(Square(), swaths),.6,1e-6,
         "overlapping 4-m strips cover 60 m2, not 80 m2");
}

void Random100Regression003() {
    types::Cell cell(Ring({{-20.584232426660236,54.190844794277545},{-69.250273880947617,91.517409644499679},{-99.325501710486634,52.305596551798956},{-82.780838583222078,39.615938163927055},{-62.963490018850408,65.453620003059228},{-43.755841061148217,50.721467294205397},{-63.573189625519888,24.883785455073223},{-50.659460256199253,14.979031701576815}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random100Regression020() {
    types::Cell cell(Ring({{24.259751166618067,23.246214605974028},{73.178102836742809,-67.777043851229067},{141.28209801272789,-31.176121816277718},{92.36374634260315,59.847136640925385}}));
    cell.AddRing(Ring({{71.823354396561072,8.0418633586924386},{70.263564274250228,14.912252935981044},{81.824494926902616,17.536940405864595},{83.384285049213474,10.666550828575996}}));
    cell.AddRing(Ring({{83.69089916349148,-25.959953246138706},{79.786498540932669,-27.366752874506577},{77.132289206071221,-27.042179874158681},{76.970205204939788,-25.138105559184869},{79.376087443030769,-22.545467462990249},{83.224196882738681,-20.477389764747148},{86.713960350251611,-19.901549003586954},{88.21247873471944,-21.087387471413681},{87.018578629872962,-23.480038170004679}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random100Regression062() {
    types::Cell cell(Ring({{-35.861492233024677,-42.077210715621625},{-43.049856223601097,-123.70309917425851},{33.54758913778025,-130.4486345479398},{40.73595312835667,-48.82274608930291}}));
    cell.AddRing(Ring({{-7.7795511323082209,-70.845380377102899},{-6.409427870572717,-61.330398724871181},{-0.65381875756270347,-62.159185812373572},{-2.023942019298218,-71.67416746460529}}));
    cell.AddRing(Ring({{-0.64744569875432489,-91.531300505047398},{-3.4986446098706665,-88.495299982450192},{-1.9228534121752219,-84.127991992363491},{1.9022380182898218,-84.46484773774813},{2.6904833346977242,-89.040344027788223}}));
    cell.AddRing(Ring({{-12.98258872112801,-108.49350862831318},{-12.514302151865252,-103.21499188206553},{-6.1496648226771455,-103.77963424710225},{-6.6179513919399042,-109.05815099334993}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random100Regression085() {
    types::Cell cell(Ring({{-33.056948973619043,3.5668107424068296},{-14.404895176538897,37.496257580080474},{-65.395290956030323,65.527236386297076},{-70.592140474619185,56.073787914474991},{-35.028624593230994,36.523436427564405},{-42.07457505095509,23.706338424466431},{-77.638090932343275,43.256689911377016},{-84.04734475311048,31.597789548623428}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random100Regression087() {
    types::Cell cell(Ring({{15.574026560009422,-13.432575239104199},{26.291005505584341,-30.200587974334578},{37.195003550286877,-42.66330053971113},{44.646495763292478,-50.411669977816914},{50.752328796443877,-44.556484650995117},{54.496042552746793,-38.430171862190065},{56.297002004662154,-30.580082279560731},{53.021866651348383,-19.065675206965924},{47.498638423312833,0.33230928403589033},{35.248561490459373,17.224680906518529},{24.391591262824427,31.510732487705244},{7.5478624376019843,24.373097341938635}}));
    Near(types::Cells(cell).buffer(-3).Area(),1587.2346473829089,1e-6,"near-collinear inset retains the vertex");
    Near(types::Point(50.136435660950667,-19.886825720646179).Distance(types::Cells(cell).buffer(-3)),0.0,1e-7,"reference inset vertex");
}

void Random500Regression029() {
    types::Cell cell(Ring({{-449.84028638456647,-574.99427256508272},{-453.50080317560287,-568.57533648599485},{-460.13412891383064,-566.07693225906746},{-464.76486496616906,-570.68426488186867},{-471.88458833738963,-572.56301357656002},{-472.60400557656567,-580.7278905002953},{-467.56320983804125,-587.1319814730839},{-461.67429149782873,-590.06202763487352},{-455.73141839537215,-590.91385117828531},{-448.71662542370552,-589.64298877913711},{-446.39628842851755,-582.16022699355005}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random500Regression244() {
    types::Cell cell(Ring({{-512.89156733044501,-442.15699804346252},{-496.71905412292574,-467.80969429199888},{-439.83387565341508,-431.94694165200127},{-444.43808035958625,-424.64379320018389},{-474.32996514917863,-443.48886495691636},{-480.53413771551942,-433.64786175131405},{-450.64225292592704,-414.80278999458164},{-456.00638886093435,-406.29424540346491}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void Random500Regression309() {
    types::Cell cell(Ring({{817.66802590343252,-109.65509983319089},{795.9350232681586,-130.55958341933319},{863.77295901012008,-201.08617787818565},{871.20034374711679,-193.9419445235867},{823.81460346542838,-144.67813878676267},{831.24866383332721,-137.5274842938166},{878.63440411501563,-186.79129003064062},{885.50596164539411,-180.18169429204335}}));
    types::Robot robot(1.5,3); robot.SetMinTurningRadius(1);
    Options options; options.hg_swaths = 2;
    auto mainland = types::Cells(cell).buffer(-3);
    sg::BruteForce generator; obj::NSwathModified objective;
    for (const auto& piece : mainland) {
        const auto swaths = generator.GenerateBestSwaths(objective,3,piece);
        Require(obj::FieldCoverage().ComputeCost(piece,swaths)>=.95,"coverage regression");
    }
    auto route = PlanCovRoute(robot,cell,options);
    Require(route.GetField().has_value(),"route retains collision domain");
    auto cloned = route.Clone();
    Near(cloned.GetField()->Area(),cell.Area(),1e-6,"clone retains field");
    pp::DubinsCurves dubins; pp::ReedsSheppCurves reeds;
    CheckPath(pp::PathPlanning::PlanPath(robot,route,dubins),cell);
    CheckPath(pp::PathPlanning::PlanPath(robot,route,reeds),cell);
}

void InvalidSwathWidthsAreRejected() {
    sg::BruteForce generator;
    for (double width : {0.0,-1.0,std::numeric_limits<double>::infinity()}) {
        bool rejected = false;
        try { generator.GenerateSwaths(0.0,width,Square()); }
        catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected,"invalid width must be rejected");
    }
}

int RunTests() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"InsetHasRequestedEdgeClearance", InsetHasRequestedEdgeClearance},
        {"SlopingInsetPreservesPrecision", SlopingInsetPreservesPrecision},
        {"InsetPreservesAndExpandsHoles", InsetPreservesAndExpandsHoles},
        {"CollapsedInsetIsEmpty", CollapsedInsetIsEmpty},
        {"HoleContainmentAndDistance", HoleContainmentAndDistance},
        {"LineClippingSplitsAtHole", LineClippingSplitsAtHole},
        {"DistanceToCollinearPolylineDoesNotConstructPolygon",
         DistanceToCollinearPolylineDoesNotConstructPolygon},
        {"PolygonBooleanAreas", PolygonBooleanAreas},
        {"BooleanOperationsPreserveDisconnectedComponents",
         BooleanOperationsPreserveDisconnectedComponents},
        {"NonconvexIntersectionAndCoverage", NonconvexIntersectionAndCoverage},
        {"CoverageCountsAllBufferedComponents",
         CoverageCountsAllBufferedComponents},
        {"WktRoundTripPreservesPrecisionAndHole",
         WktRoundTripPreservesPrecisionAndHole},
        {"ShortEntrySpurIsRoundedForBothAlgorithms",
         ShortEntrySpurIsRoundedForBothAlgorithms},
        {"Random100Regression003",Random100Regression003},
        {"Random100Regression020",Random100Regression020},
        {"Random100Regression062",Random100Regression062},
        {"Random100Regression085",Random100Regression085},
        {"Random100Regression087",Random100Regression087},
        {"Random500Regression029",Random500Regression029},
        {"Random500Regression244",Random500Regression244},
        {"Random500Regression309",Random500Regression309},
        {"RotatedTouchingStripsHaveExactCoverage",RotatedTouchingStripsHaveExactCoverage},
        {"OverlappingFootprintsAreCountedOnce",OverlappingFootprintsAreCountedOnce},
        {"InvalidSwathWidthsAreRejected",InvalidSwathWidthsAreRejected},
        {"scenario_rectangle", [] { VerifyScenario("rectangle"); }},
        {"scenario_trapezoid", [] { VerifyScenario("trapezoid"); }},
        {"scenario_concave", [] { VerifyScenario("concave"); }},
        {"scenario_obstacle", [] { VerifyScenario("obstacle"); }},
    };
    int failures = 0;
    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        } catch (const std::exception& e) {
            ++failures;
            std::cerr << "FAIL " << test.first << ": " << e.what() << '\n';
        }
    }
    std::cout << (tests.size() - failures) << "/" << tests.size()
              << " checks passed\n";
    return failures ? 1 : 0;
}
}  // namespace
}  // namespace autonomy::planning::planner::fields2cover

int main() {
    return autonomy::planning::planner::fields2cover::RunTests();
}
