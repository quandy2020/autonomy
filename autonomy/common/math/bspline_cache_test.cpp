/* Copyright 2026 The Openbot Authors (duyongquan). */
/**
 * @file bspline_cache_test.cpp
 * @brief Analytical, storage-ownership and allocation checks for shared splines.
 *
 * This standalone process counts C++ heap allocations only while evaluating
 * scalar curves. Construction, copying and output sampling intentionally remain
 * outside that window. Dynamic point algebra is not covered by this guarantee.
 */
#include "autonomy/common/math/bspline.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace {
thread_local bool count_allocations = false;  ///< Restrict accounting to the evaluation thread/window.
std::atomic<std::size_t> allocation_count{0};  ///< Allocations observed in the active scalar window.
}

/** @brief Count actual scalar-evaluation allocations, then use the system heap. */
void* operator new(std::size_t size) {
    if (count_allocations) ++allocation_count;
    if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
    throw std::bad_alloc();
}
/** @brief Match the test process's scalar heap allocation. */
void operator delete(void* memory) noexcept { std::free(memory); }
/** @brief Match sized scalar deletion emitted by C++17 compilers. */
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
/** @brief Count array allocations using the same scalar accounting. */
void* operator new[](std::size_t size) { return ::operator new(size); }
/** @brief Match array deletion in this standalone process. */
void operator delete[](void* memory) noexcept { std::free(memory); }
/** @brief Match sized array deletion. */
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

namespace {
using Curve = autonomy::common::math::ClampedCubicBSpline<double>;

/** @brief Release-build checks with explicit failure explanations. */
void Require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
/** @brief Compare analytical or independently differentiated values. */
void Near(double actual, double expected, double tolerance = 1e-8) {
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            "shared spline differs from analytical or numerical expectation");
}
/** @brief A nonuniform clamped knot vector with two distinct interior knots. */
std::vector<double> Knots() { return {0, 0, 0, 0, .1, .45, 1, 1, 1, 1}; }

/** @brief The cubic u^3 has closed-form derivatives, including both endpoints. */
void AnalyticalCubic() {
    const Curve curve({0, 0, 0, 1});
    for (int i = 0; i <= 100; ++i) {
        const double u = static_cast<double>(i) / 100;
        Near(curve.Evaluate(u), u * u * u);
        Near(curve.EvaluateDerivative(1, u), 3 * u * u);
        Near(curve.EvaluateDerivative(2, u), 6 * u);
        Near(curve.EvaluateDerivative(3, u), 6);
    }
    Near(curve.Evaluate(-1), 0); Near(curve.Evaluate(2), 1);
    Near(curve.EvaluateDerivative(3, -1), 6);
    Near(curve.EvaluateDerivative(4, .5), 0);
    Near(curve.EvaluateDerivative(-1, .5), 0);
}

/** @brief Greville controls reproduce u exactly even for nonuniform knots. */
void NonuniformLinearReproduction() {
    const Curve curve({0, 1.0 / 30, 11.0 / 60, 31.0 / 60, 49.0 / 60, 1}, Knots());
    for (int i = 0; i <= 100; ++i) {
        const double u = static_cast<double>(i) / 100;
        Near(curve.Evaluate(u), u);
        Near(curve.EvaluateDerivative(1, u), 1);
        Near(curve.EvaluateDerivative(2, u), 0);
        Near(curve.EvaluateDerivative(3, u), 0);
    }
}

/** @brief Finite differences, C2 knot continuity and right-sided cubic jerk. */
void NonuniformDerivatives() {
    const Curve curve({0, 2, 1, 4, 3, 5}, Knots());
    constexpr double h = 1e-6;
    for (double u : {.03, .07, .2, .35, .55, .8, .95}) {
        for (int order = 1; order <= 3; ++order) {
            const double numerical = (curve.EvaluateDerivative(order - 1, u + h) -
                                      curve.EvaluateDerivative(order - 1, u - h)) / (2 * h);
            Near(curve.EvaluateDerivative(order, u), numerical, 1e-5);
        }
    }
    for (double u : {.1, .45}) {
        for (int order = 0; order <= 2; ++order)
            Near(curve.EvaluateDerivative(order, u - 1e-10),
                 curve.EvaluateDerivative(order, u + 1e-10), 1e-5);
        Near(curve.EvaluateDerivative(3, u), curve.EvaluateDerivative(3, u + 1e-10));
        Require(std::abs(curve.EvaluateDerivative(3, u - 1e-10) -
                         curve.EvaluateDerivative(3, u)) > 1, "fixture lacks a cubic jerk jump");
    }
    for (int order = 1; order <= 3; ++order) {
        const auto controls = curve.DerivativeControlPoints(order);
        const auto limits = std::minmax_element(controls.begin(), controls.end());
        for (int i = 0; i <= 1000; ++i) {
            const double value = curve.EvaluateDerivative(order, static_cast<double>(i) / 1000);
            Require(value >= *limits.first - 1e-8 && value <= *limits.second + 1e-8,
                    "derivative escaped its control hull");
        }
    }
}

/** @brief Default, short and assigned curves preserve their original public semantics. */
void OwnershipAndFallback() {
    const Curve original({0, 2, 1, 4, 3, 5}, Knots());
    Curve assigned({0, 0, 0, 1});
    assigned = original;
    Curve copied(original);
    Curve moved(std::move(copied));
    Curve move_assigned;
    move_assigned = std::move(assigned);
    auto detached_controls = original.DerivativeControlPoints(2);
    detached_controls.front() = 1e10;
    for (double u : {0.0, .07, .1, .3, .45, .7, 1.0}) {
        for (int order = 0; order <= 3; ++order) {
            Near(move_assigned.EvaluateDerivative(order, u), original.EvaluateDerivative(order, u));
            Near(moved.EvaluateDerivative(order, u), original.EvaluateDerivative(order, u));
        }
    }
    move_assigned = Curve({2, 100, 6});
    Near(move_assigned.Evaluate(.25), 3);
    Near(move_assigned.EvaluateDerivative(1, .3), 4);
    Near(move_assigned.EvaluateDerivative(2, .3), 0);
    Near(Curve({5}).Evaluate(.3), 5);
    Near(Curve({5}).EvaluateDerivative(1, .3), 0);
    Near(Curve().Evaluate(.3), 0);
    Require(Curve().DerivativeControlPoints(1).empty(), "empty curve acquired derivative controls");
    Require(original.DerivativeControlPoints(4).empty(), "invalid derivative order acquired controls");
    const auto samples = original.Sample(17);
    Require(samples.size() == 17, "sample count changed");
    Near(samples.front(), 0); Near(samples.back(), 5);
    Require(original.Sample(0).size() == 2, "minimum sample count changed");
}

/** @brief Reject malformed explicit knots before caching any derivative curve. */
void InvalidKnots() {
    for (const auto& knots : std::vector<std::vector<double>>{
             {0, 0, 0, 0, .1, .1, 1, 1, 1, 1},
             {0, 0, 0, 0, .1, .45, .9, 1, 1, 1},
             {0, 0, 0, 0, .1, .45, 1, 1, 1}}) {
        bool rejected = false;
        try { Curve ignored({0, 2, 1, 4, 3, 5}, knots); }
        catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "malformed knot vector accepted");
    }
}

/** @brief Shared boundary solving reproduces endpoint derivatives for all knot layouts. */
void StartBoundary() {
    for (const auto& original : std::vector<Curve>{
             Curve({0, 2, 1, 4}), Curve({0, 2, 1, 4, 3, 5}, Knots()),
             Curve({0, 2, 1, 4, 3, 5, 4, 6})}) {
        const auto unchanged = original.control_points();
        const auto replacement = original.StartControlPoints(-2.0, .4, -.7);
        auto controls = unchanged;
        std::copy(replacement.begin(), replacement.end(), controls.begin());
        const Curve fitted(std::move(controls), original.knots());
        Near(fitted.Evaluate(0), -2.0);
        Near(fitted.EvaluateDerivative(1, 0), .4);
        Near(fitted.EvaluateDerivative(2, 0), -.7);
        Require(original.control_points() == unchanged, "boundary solver mutated its source");
        Require(fitted.knots() == original.knots(), "boundary solver changed knot layout");
        for (std::size_t i = 3; i < unchanged.size(); ++i)
            Near(fitted.control_points()[i], unchanged[i]);
        const double h = 1e-7;
        Near((fitted.EvaluateDerivative(1, h) - fitted.EvaluateDerivative(1, 0)) / h,
             -.7, 1e-3);
    }
    const auto reject = [](auto operation) {
        bool rejected = false;
        try { operation(); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "invalid cubic boundary was accepted");
    };
    const Curve cubic({0, 0, 0, 1});
    const auto infinity = std::numeric_limits<double>::infinity();
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    reject([&] { cubic.StartControlPoints(nan, 0, 0); });
    reject([&] { cubic.StartControlPoints(0, infinity, 0); });
    reject([&] { cubic.StartControlPoints(0, 0, nan); });
    reject([] { Curve({0, 1, 2}).StartControlPoints(0, 0, 0); });
    reject([] { Curve().StartControlPoints(0, 0, 0); });
    bool overflow = false;
    const double maximum = std::numeric_limits<double>::max();
    try { cubic.StartControlPoints(maximum, maximum, 0); }
    catch (const std::overflow_error&) { overflow = true; }
    Require(overflow, "overflowing boundary controls were accepted");
}

/** @brief Hot scalar evaluation must not allocate after curve construction. */
void AllocationFreeEvaluation() {
    const Curve uniform({0, 2, 1, 4, 3, 5, 4, 6});
    const Curve nonuniform({0, 2, 1, 4, 3, 5}, Knots());
    allocation_count.store(0);
    count_allocations = true;
    double checksum = 0;
    for (int i = 0; i < 10000; ++i) {
        const double u = static_cast<double>(i % 101) / 100;
        for (int order = 0; order <= 3; ++order) {
            checksum += uniform.EvaluateDerivative(order, u);
            checksum += nonuniform.EvaluateDerivative(order, u);
        }
    }
    count_allocations = false;
    Require(std::isfinite(checksum), "hot evaluation produced a nonfinite checksum");
    const auto count = allocation_count.load();
    std::cout << "scalar hot-loop allocations: " << count << '\n';
    Require(count == 0, "scalar derivative evaluation still allocates spline storage");
}

/** @brief Const evaluation can read the same cached curve from multiple threads. */
void ConcurrentEvaluation() {
    const Curve curve({0, 0, 0, 1});
    std::array<double, 4> totals{};
    std::array<std::thread, 4> threads;
    for (std::size_t i = 0; i < threads.size(); ++i) {
        threads[i] = std::thread([&, i] {
            for (int n = 0; n < 1000; ++n)
                totals[i] += curve.Evaluate(.5) + curve.EvaluateDerivative(1, .5) +
                             curve.EvaluateDerivative(2, .5) + curve.EvaluateDerivative(3, .5);
        });
    }
    for (auto& thread : threads) thread.join();
    for (double total : totals) Near(total, 9875);
}
}

/** @brief Run actual common/math evaluation and report a bounded failure. */
int main() {
    try {
        AnalyticalCubic(); NonuniformLinearReproduction(); NonuniformDerivatives();
        OwnershipAndFallback(); InvalidKnots(); StartBoundary();
        AllocationFreeEvaluation(); ConcurrentEvaluation();
        std::cout << "shared B-spline cache checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        count_allocations = false;
        std::cerr << error.what() << '\n';
        return 1;
    }
}
