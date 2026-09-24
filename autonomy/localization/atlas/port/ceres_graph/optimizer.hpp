/*
 * Copyright 2026 The Openbot Authors
 *
 * Ceres SE3 pose graph. Same call shape as the previous graph optimizer
 * (vertex / relative edge / prior / height prior), so loop closing and PGO
 * keep their structure while the solver stays Ceres.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_PORT_CERES_GRAPH_OPTIMIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_PORT_CERES_GRAPH_OPTIMIZER_HPP_

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "ceres/ceres.h"

#include "autonomy/localization/atlas/port/common/eigen_types.hpp"

namespace atlas_lio {
namespace miao {

enum class AlgorithmType { GAUSS_NEWTON = 0, LEVENBERG_MARQUARDT, DOGLEG };

enum class LinearSolverType {
    LINEAR_SOLVER_DENSE,
    LINEAR_SOLVER_SPARSE_EIGEN,
    LINEAR_SOLVER_PCG,
};

struct OptimizerConfig {
    OptimizerConfig() = default;
    explicit OptimizerConfig(
        AlgorithmType algo_type,
        LinearSolverType linear_type = LinearSolverType::LINEAR_SOLVER_DENSE,
        bool is_dense = true)
        : algo_type_(algo_type),
          is_dense_(is_dense),
          linear_solver_type_(linear_type) {}

    AlgorithmType algo_type_ = AlgorithmType::LEVENBERG_MARQUARDT;
    LinearSolverType linear_solver_type_ =
        LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN;
    bool is_dense_ = true;
    bool incremental_mode_ = false;
    int max_vertex_size_ = -1;
    bool parallel_ = true;
    double eps_chi2_ = 1e-4;
};

enum class KernelType { kCauchy, kHuber };

class RobustKernel {
public:
    virtual ~RobustKernel() = default;
    virtual void SetDelta(double delta) { delta_ = delta; }
    double Delta() const { return delta_; }
    virtual KernelType kernel_type() const = 0;

private:
    double delta_ = 1.0;
};

class RobustKernelCauchy : public RobustKernel {
public:
    KernelType kernel_type() const override { return KernelType::kCauchy; }
};

class RobustKernelHuber : public RobustKernel {
public:
    KernelType kernel_type() const override { return KernelType::kHuber; }
};

class Vertex {
public:
    virtual ~Vertex() = default;
    virtual void SetId(int id) { id_ = id; }
    virtual int GetId() const { return id_; }
    virtual void SetFixed(bool fixed) { fixed_ = fixed; }
    bool fixed() const { return fixed_; }
    virtual double* mutable_block() = 0;
    virtual void SyncToParameter() = 0;
    virtual void SyncFromParameter() = 0;

private:
    int id_ = -1;
    bool fixed_ = false;
};

inline void Se3ToBlock(const SE3& pose, double* data) {
    Eigen::AngleAxisd aa(pose.rotationMatrix());
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    if (std::isfinite(aa.angle()) && aa.angle() > 1e-12) {
        w = aa.axis() * aa.angle();
    }
    data[0] = w.x();
    data[1] = w.y();
    data[2] = w.z();
    data[3] = pose.translation().x();
    data[4] = pose.translation().y();
    data[5] = pose.translation().z();
}

inline SE3 BlockToSe3(const double* data) {
    const Eigen::Vector3d w(data[0], data[1], data[2]);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const double n = w.norm();
    if (n > 1e-12 && std::isfinite(n)) {
        rotation = Eigen::AngleAxisd(n, w / n).toRotationMatrix();
    }
    return SE3(SO3(rotation), Eigen::Vector3d(data[3], data[4], data[5]));
}

class VertexSE3 : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetEstimate(const SE3& pose) {
        estimate_ = pose;
        Se3ToBlock(estimate_, data_);
    }

    SE3 Estimate() const { return estimate_; }

    double* mutable_block() override { return data_; }
    void SyncToParameter() override { Se3ToBlock(estimate_, data_); }
    void SyncFromParameter() override { estimate_ = BlockToSe3(data_); }

private:
    SE3 estimate_;
    double data_[6] = {0, 0, 0, 0, 0, 0};
};

class Edge {
public:
    virtual ~Edge() = default;
    virtual void SetVertex(std::size_t i, std::shared_ptr<Vertex> vertex) = 0;
    virtual Vertex* GetVertex(std::size_t i) const = 0;
    virtual void ComputeError() = 0;
    virtual double Chi2() const = 0;
    virtual void AddTo(ceres::Problem* problem) = 0;

    void SetLevel(int level) { level_ = level; }
    int Level() const { return level_; }
    std::shared_ptr<RobustKernel> GetRobustKernel() const { return kernel_; }
    void SetRobustKernel(std::shared_ptr<RobustKernel> kernel) {
        kernel_ = std::move(kernel);
    }

protected:
    ceres::LossFunction* MakeLoss() const {
        if (!kernel_) {
            return nullptr;
        }
        const double delta = std::max(kernel_->Delta(), 1e-9);
        if (kernel_->kernel_type() == KernelType::kHuber) {
            return new ceres::HuberLoss(std::sqrt(delta));
        }
        return new ceres::CauchyLoss(std::sqrt(delta));
    }

    int level_ = 0;
    std::shared_ptr<RobustKernel> kernel_;
};

inline Eigen::Matrix<double, 6, 6> SqrtInformation(
    const Mat6d& information) {
    Eigen::LLT<Mat6d> llt(information);
    if (llt.info() == Eigen::Success) {
        return llt.matrixU();
    }
    Mat6d root = Mat6d::Zero();
    for (int i = 0; i < 6; ++i) {
        root(i, i) = std::sqrt(std::max(0.0, information(i, i)));
    }
    return root;
}

template <typename T>
void DecodePose(const T* pose, Eigen::Matrix<T, 3, 3>* rotation,
                Eigen::Matrix<T, 3, 1>* translation) {
    const Eigen::Matrix<T, 3, 1> w(pose[0], pose[1], pose[2]);
    const T n = w.norm();
    *rotation = Eigen::Matrix<T, 3, 3>::Identity();
    if (n > T(1e-12)) {
        *rotation = Eigen::AngleAxis<T>(n, w / n).toRotationMatrix();
    }
    *translation = Eigen::Matrix<T, 3, 1>(pose[3], pose[4], pose[5]);
}

// g2o-style relative SE3: translation first, then quaternion xyz.
// Measurement is T_i^{-1} * T_j.
template <typename T>
Eigen::Matrix<T, 6, 1> RelativeError(
    const Eigen::Matrix<T, 3, 3>& R_i, const Eigen::Matrix<T, 3, 1>& t_i,
    const Eigen::Matrix<T, 3, 3>& R_j, const Eigen::Matrix<T, 3, 1>& t_j,
    const Eigen::Matrix3d& R_meas, const Eigen::Vector3d& t_meas) {
    const Eigen::Matrix<T, 3, 3> R12 = R_i.transpose() * R_j;
    const Eigen::Matrix<T, 3, 1> t12 = R_i.transpose() * (t_j - t_i);
    const Eigen::Matrix<T, 3, 3> R_m = R_meas.template cast<T>();
    const Eigen::Matrix<T, 3, 1> t_m = t_meas.template cast<T>();
    const Eigen::Matrix<T, 3, 3> R_delta = R_m.transpose() * R12;
    const Eigen::Matrix<T, 3, 1> t_delta = R_m.transpose() * (t12 - t_m);
    Eigen::Quaternion<T> q(R_delta);
    if (q.w() < T(0)) {
        q.coeffs() *= T(-1);
    }
    Eigen::Matrix<T, 6, 1> error;
    error.template head<3>() = t_delta;
    error(3) = q.x();
    error(4) = q.y();
    error(5) = q.z();
    return error;
}

template <typename T>
Eigen::Matrix<T, 6, 1> PriorError(const Eigen::Matrix<T, 3, 3>& rotation,
                                  const Eigen::Matrix<T, 3, 1>& translation,
                                  const Eigen::Matrix3d& R_meas,
                                  const Eigen::Vector3d& t_meas) {
    const Eigen::Matrix<T, 3, 3> R_delta =
        rotation.transpose() * R_meas.template cast<T>();
    Eigen::AngleAxis<T> aa(R_delta);
    Eigen::Matrix<T, 3, 1> w = Eigen::Matrix<T, 3, 1>::Zero();
    if (aa.angle() > T(1e-12)) {
        w = aa.axis() * aa.angle();
    }
    Eigen::Matrix<T, 6, 1> error;
    error.template head<3>() = translation - t_meas.template cast<T>();
    error.template tail<3>() = w;
    return error;
}

class EdgeSE3 : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetVertex(std::size_t i, std::shared_ptr<Vertex> vertex) override {
        if (i < 2) {
            vertices_[i] = std::move(vertex);
        }
    }

    Vertex* GetVertex(std::size_t i) const override {
        return i < 2 ? vertices_[i].get() : nullptr;
    }

    void SetMeasurement(const SE3& measurement) {
        R_meas_ = measurement.rotationMatrix();
        t_meas_ = measurement.translation();
    }

    void SetInformation(const Mat6d& information) {
        information_ = information;
        sqrt_info_ = SqrtInformation(information_);
    }

    void ComputeError() override { chi2_ = Chi2(); }

    double Chi2() const override {
        if (!vertices_[0] || !vertices_[1]) {
            return 0.0;
        }
        const SE3 from =
            static_cast<const VertexSE3*>(vertices_[0].get())->Estimate();
        const SE3 to =
            static_cast<const VertexSE3*>(vertices_[1].get())->Estimate();
        const Eigen::Matrix<double, 6, 1> error = RelativeError<double>(
            from.rotationMatrix(), from.translation(), to.rotationMatrix(),
            to.translation(), R_meas_, t_meas_);
        return error.dot(information_ * error);
    }

    void AddTo(ceres::Problem* problem) override {
        if (level_ != 0 || problem == nullptr || !vertices_[0] ||
            !vertices_[1]) {
            return;
        }
        auto* cost =
            new ceres::AutoDiffCostFunction<Cost, 6, 6, 6>(new Cost(*this));
        problem->AddResidualBlock(cost, MakeLoss(), vertices_[0]->mutable_block(),
                                   vertices_[1]->mutable_block());
    }

private:
    struct Cost {
        explicit Cost(const EdgeSE3& edge)
            : R_meas(edge.R_meas_),
              t_meas(edge.t_meas_),
              sqrt_info(edge.sqrt_info_) {}

        template <typename T>
        bool operator()(const T* const pose_i, const T* const pose_j,
                        T* residuals) const {
            Eigen::Matrix<T, 3, 3> R_i;
            Eigen::Matrix<T, 3, 3> R_j;
            Eigen::Matrix<T, 3, 1> t_i;
            Eigen::Matrix<T, 3, 1> t_j;
            DecodePose(pose_i, &R_i, &t_i);
            DecodePose(pose_j, &R_j, &t_j);
            const Eigen::Matrix<T, 6, 1> error =
                RelativeError<T>(R_i, t_i, R_j, t_j, R_meas, t_meas);
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals);
            residual = sqrt_info.template cast<T>() * error;
            return true;
        }

        Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_meas = Eigen::Vector3d::Zero();
        Mat6d sqrt_info = Mat6d::Identity();
    };

    std::shared_ptr<Vertex> vertices_[2];
    Eigen::Matrix3d R_meas_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_meas_ = Eigen::Vector3d::Zero();
    Mat6d information_ = Mat6d::Identity();
    Mat6d sqrt_info_ = Mat6d::Identity();
    double chi2_ = 0.0;
};

class EdgeSE3Prior : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetVertex(std::size_t i, std::shared_ptr<Vertex> vertex) override {
        if (i == 0) {
            vertex_ = std::move(vertex);
        }
    }

    Vertex* GetVertex(std::size_t i) const override {
        return i == 0 ? vertex_.get() : nullptr;
    }

    void SetMeasurement(const SE3& measurement) {
        R_meas_ = measurement.rotationMatrix();
        t_meas_ = measurement.translation();
    }

    void SetInformation(const Mat6d& information) {
        information_ = information;
        sqrt_info_ = SqrtInformation(information_);
    }

    void ComputeError() override { chi2_ = Chi2(); }

    double Chi2() const override {
        if (!vertex_) {
            return 0.0;
        }
        const SE3 pose =
            static_cast<const VertexSE3*>(vertex_.get())->Estimate();
        const Eigen::Matrix<double, 6, 1> error = PriorError<double>(
            pose.rotationMatrix(), pose.translation(), R_meas_, t_meas_);
        return error.dot(information_ * error);
    }

    void AddTo(ceres::Problem* problem) override {
        if (level_ != 0 || problem == nullptr || !vertex_) {
            return;
        }
        auto* cost =
            new ceres::AutoDiffCostFunction<Cost, 6, 6>(new Cost(*this));
        problem->AddResidualBlock(cost, MakeLoss(), vertex_->mutable_block());
    }

private:
    struct Cost {
        explicit Cost(const EdgeSE3Prior& edge)
            : R_meas(edge.R_meas_),
              t_meas(edge.t_meas_),
              sqrt_info(edge.sqrt_info_) {}

        template <typename T>
        bool operator()(const T* const pose, T* residuals) const {
            Eigen::Matrix<T, 3, 3> rotation;
            Eigen::Matrix<T, 3, 1> translation;
            DecodePose(pose, &rotation, &translation);
            const Eigen::Matrix<T, 6, 1> error =
                PriorError<T>(rotation, translation, R_meas, t_meas);
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals);
            residual = sqrt_info.template cast<T>() * error;
            return true;
        }

        Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_meas = Eigen::Vector3d::Zero();
        Mat6d sqrt_info = Mat6d::Identity();
    };

    std::shared_ptr<Vertex> vertex_;
    Eigen::Matrix3d R_meas_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_meas_ = Eigen::Vector3d::Zero();
    Mat6d information_ = Mat6d::Identity();
    Mat6d sqrt_info_ = Mat6d::Identity();
    double chi2_ = 0.0;
};

class EdgeHeightPrior : public Edge {
public:
    void SetVertex(std::size_t i, std::shared_ptr<Vertex> vertex) override {
        if (i == 0) {
            vertex_ = std::move(vertex);
        }
    }

    Vertex* GetVertex(std::size_t i) const override {
        return i == 0 ? vertex_.get() : nullptr;
    }

    void SetMeasurement(double measurement) { measurement_ = measurement; }

    void SetInformation(const Mat1d& information) {
        information_ = std::max(0.0, information(0, 0));
    }

    void ComputeError() override { chi2_ = Chi2(); }

    double Chi2() const override {
        if (!vertex_) {
            return 0.0;
        }
        const SE3 pose =
            static_cast<const VertexSE3*>(vertex_.get())->Estimate();
        const double error = pose.translation().z() - measurement_;
        return error * information_ * error;
    }

    void AddTo(ceres::Problem* problem) override {
        if (level_ != 0 || problem == nullptr || !vertex_) {
            return;
        }
        auto* cost =
            new ceres::AutoDiffCostFunction<Cost, 1, 6>(new Cost(*this));
        problem->AddResidualBlock(cost, MakeLoss(), vertex_->mutable_block());
    }

private:
    struct Cost {
        explicit Cost(const EdgeHeightPrior& edge)
            : measurement(edge.measurement_),
              sqrt_info(std::sqrt(edge.information_)) {}

        template <typename T>
        bool operator()(const T* const pose, T* residuals) const {
            residuals[0] = T(sqrt_info) * (pose[5] - T(measurement));
            return true;
        }

        double measurement = 0.0;
        double sqrt_info = 1.0;
    };

    std::shared_ptr<Vertex> vertex_;
    double measurement_ = 0.0;
    double information_ = 1.0;
    double chi2_ = 0.0;
};

class Optimizer {
public:
    bool AddVertex(std::shared_ptr<Vertex> vertex) {
        if (!vertex) {
            return false;
        }
        vertices_[vertex->GetId()] = vertex;
        return true;
    }

    bool AddEdge(std::shared_ptr<Edge> edge) {
        if (!edge) {
            return false;
        }
        edges_.insert(std::move(edge));
        return true;
    }

    std::shared_ptr<Vertex> GetVertex(int id) const {
        const auto it = vertices_.find(id);
        return it == vertices_.end() ? nullptr : it->second;
    }

    const std::set<std::shared_ptr<Edge>>& GetEdges() const { return edges_; }

    bool InitializeOptimization(int /*level*/ = 0) { return true; }

    void SetVerbose(bool verbose) { verbose_ = verbose; }

    void Clear() {
        edges_.clear();
        vertices_.clear();
    }

    int Optimize(int iterations) {
        ceres::Problem problem;
        std::vector<Vertex*> used;
        for (const auto& edge : edges_) {
            if (!edge || edge->Level() != 0) {
                continue;
            }
            edge->AddTo(&problem);
        }
        if (problem.NumResidualBlocks() == 0) {
            return 0;
        }
        for (const auto& item : vertices_) {
            Vertex* vertex = item.second.get();
            if (vertex == nullptr) {
                continue;
            }
            vertex->SyncToParameter();
            if (problem.HasParameterBlock(vertex->mutable_block()) &&
                vertex->fixed()) {
                problem.SetParameterBlockConstant(vertex->mutable_block());
            }
        }
        ceres::Solver::Options options;
        options.max_num_iterations = std::max(1, iterations);
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = verbose_;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        for (const auto& item : vertices_) {
            if (item.second &&
                problem.HasParameterBlock(item.second->mutable_block())) {
                item.second->SyncFromParameter();
            }
        }
        return summary.iterations.size();
    }

private:
    std::map<int, std::shared_ptr<Vertex>> vertices_;
    std::set<std::shared_ptr<Edge>> edges_;
    bool verbose_ = false;
};

template <int /*pose_dim*/ = -1, int /*landmark_dim*/ = -1>
std::shared_ptr<Optimizer> SetupOptimizer(OptimizerConfig /*options*/) {
    return std::make_shared<Optimizer>();
}

}  // namespace miao
}  // namespace atlas_lio

#endif  // AUTONOMY_LOCALIZATION_ATLAS_PORT_CERES_GRAPH_OPTIMIZER_HPP_
