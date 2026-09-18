//
// Created by xiang on 24-4-25.
//

#include "optimization_algorithm.hpp"
#include "core/graph/optimizer.hpp"
#include "core/graph/vertex.hpp"
#include "core/solver/solver.hpp"

namespace lightning::miao {

OptimizationAlgorithm::~OptimizationAlgorithm() {}

bool OptimizationAlgorithm::Init() {
    assert(optimizer_ && "optimizer not set");

    bool useSchur = false;
    for (const auto &v : optimizer_->ActiveVertices()) {
        if (v->Marginalized()) {
            useSchur = true;
            break;
        }
    }

    if (useSchur) {
        if (solver_->SupportsSchur()) {
            solver_->SetSchur(true);
        }
    } else {
        if (solver_->SupportsSchur()) {
            solver_->SetSchur(false);
        }
    }

    return solver_->Init(optimizer_);
}

void OptimizationAlgorithm::SetOptimizer(Optimizer *optimizer) { optimizer_ = optimizer; }

}  // namespace lightning::miao