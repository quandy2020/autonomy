/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "atlas/common/optimization/optimizer.hpp"
#include "autolink/common/log.hpp"

#include <cmath>
#include <complex>
#include <cstddef>
#include <memory>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

#include "atlas/common/math/converter.hpp"
#include "atlas/common/optimization/g2o_types.hpp"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include <mutex>

#include "atlas/common/optimization/optimizable_types.hpp"

namespace autonomy::localization::atlas {
namespace {

template <typename Container>
std::size_t CountMapPointObservations(const Container& map_points) {
    std::size_t observation_count = 0;
    for (MapPoint* map_point : map_points) {
        if (map_point == nullptr)
            continue;
        const int observations = map_point->Observations();
        if (observations > 0)
            observation_count += static_cast<std::size_t>(observations);
    }
    return observation_count;
}

template <typename BlockSolver, template <typename> class LinearSolver,
          typename Algorithm>
Algorithm* CreateOptimizationAlgorithm() {
    auto linear_solver =
        std::make_unique<LinearSolver<typename BlockSolver::PoseMatrixType>>();
    auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
    return new Algorithm(std::move(block_solver));
}

bool IsFiniteRotation(const Eigen::Matrix3d& rotation) {
    if (!rotation.allFinite())
        return false;
    const Eigen::Matrix3d orthogonality_error =
        rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
    return orthogonality_error.norm() <= 1e-3 &&
           std::abs(rotation.determinant() - 1.0) <= 1e-3;
}

bool IsNonIncreasingCost(double initial_cost, double optimized_cost) {
    if (!std::isfinite(initial_cost) || !std::isfinite(optimized_cost))
        return false;
    const double tolerance = std::max(1e-9, std::abs(initial_cost) * 1e-6);
    return optimized_cost <= initial_cost + tolerance;
}

}  // namespace

bool Optimizer::IsRelativeGeometryUpdateReasonable(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& optimized, double maximum_scale_ratio) {
    if (current.size() != optimized.size() ||
        !std::isfinite(maximum_scale_ratio) || maximum_scale_ratio < 1.0) {
        return false;
    }
    if (current.empty()) {
        return true;
    }

    Eigen::Vector3d current_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d optimized_centroid = Eigen::Vector3d::Zero();
    for (std::size_t index = 0; index < current.size(); ++index) {
        if (!current[index].allFinite() || !optimized[index].allFinite()) {
            return false;
        }
        current_centroid += current[index];
        optimized_centroid += optimized[index];
    }
    if (current.size() < 2) {
        return true;
    }
    current_centroid /= static_cast<double>(current.size());
    optimized_centroid /= static_cast<double>(optimized.size());

    double current_squared_extent = 0.0;
    double optimized_squared_extent = 0.0;
    for (std::size_t index = 0; index < current.size(); ++index) {
        current_squared_extent +=
            (current[index] - current_centroid).squaredNorm();
        optimized_squared_extent +=
            (optimized[index] - optimized_centroid).squaredNorm();
    }
    const double current_extent =
        std::sqrt(current_squared_extent / static_cast<double>(current.size()));
    const double optimized_extent = std::sqrt(
        optimized_squared_extent / static_cast<double>(optimized.size()));
    if (!std::isfinite(current_extent) || !std::isfinite(optimized_extent)) {
        return false;
    }

    const double reference_extent = std::max(current_extent, optimized_extent);
    const double degeneracy_tolerance =
        std::max(1e-12, reference_extent * 1e-12);
    if (reference_extent <= degeneracy_tolerance) {
        return true;
    }
    if (current_extent <= degeneracy_tolerance ||
        optimized_extent <= degeneracy_tolerance) {
        return false;
    }

    const double scale_ratio = std::max(current_extent, optimized_extent) /
                               std::min(current_extent, optimized_extent);
    return std::isfinite(scale_ratio) && scale_ratio <= maximum_scale_ratio;
}

bool Optimizer::IsRelativeMagnitudeUpdateReasonable(
    const std::vector<Eigen::Vector3d>& current,
    const std::vector<Eigen::Vector3d>& optimized,
    double maximum_magnitude_ratio) {
    if (current.size() != optimized.size() ||
        !std::isfinite(maximum_magnitude_ratio) ||
        maximum_magnitude_ratio < 1.0) {
        return false;
    }
    if (current.empty()) {
        return true;
    }

    double current_squared_magnitude = 0.0;
    double optimized_squared_magnitude = 0.0;
    for (std::size_t index = 0; index < current.size(); ++index) {
        if (!current[index].allFinite() || !optimized[index].allFinite()) {
            return false;
        }
        current_squared_magnitude += current[index].squaredNorm();
        optimized_squared_magnitude += optimized[index].squaredNorm();
    }
    const double current_magnitude = std::sqrt(
        current_squared_magnitude / static_cast<double>(current.size()));
    const double optimized_magnitude = std::sqrt(
        optimized_squared_magnitude / static_cast<double>(optimized.size()));
    if (!std::isfinite(current_magnitude) ||
        !std::isfinite(optimized_magnitude)) {
        return false;
    }

    const double reference_magnitude =
        std::max(current_magnitude, optimized_magnitude);
    const double degeneracy_tolerance =
        std::max(1e-12, reference_magnitude * 1e-12);
    if (reference_magnitude <= degeneracy_tolerance) {
        return true;
    }
    if (current_magnitude <= degeneracy_tolerance ||
        optimized_magnitude <= degeneracy_tolerance) {
        return false;
    }

    const double magnitude_ratio =
        std::max(current_magnitude, optimized_magnitude) /
        std::min(current_magnitude, optimized_magnitude);
    return std::isfinite(magnitude_ratio) &&
           magnitude_ratio <= maximum_magnitude_ratio;
}

bool sortByVal(const std::pair<MapPoint*, int>& a,
               const std::pair<MapPoint*, int>& b) {
    return (a.second < b.second);
}

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations,
                                       bool* pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const std::vector<KeyFrame*>& vpKFs,
                                 const std::vector<MapPoint*>& vpMP,
                                 int nIterations, bool* pbStopFlag,
                                 const unsigned long nLoopKF,
                                 const bool bRobust) {
    std::vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    Map* pMap = vpKFs[0]->GetMap();

    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_6_3,
                                    g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    const std::size_t expected_edge_count = CountMapPointObservations(vpMP);

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZ*>
        vpEdgesMono;
    vpEdgesMono.reserve(expected_edge_count);

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZToBody*>
        vpEdgesBody;
    vpEdgesBody.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(expected_edge_count);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(expected_edge_count);

    // Set KeyFrame vertices

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        autonomy::common::math::SE3<float> Tcw = pKF->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
            maxKFid = pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for (std::size_t i = 0; i < vpMP.size(); i++) {
        MapPoint* pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMP->GetObservations();

        int nEdges = 0;
        // SET EDGES
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end(); mit++) {
            KeyFrame* pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid)
                continue;
            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(pKF->mnId) == nullptr)
                continue;
            nEdges++;

            const int leftIndex = std::get<0>(mit->second);

            if (leftIndex != -1 &&
                pKF->mvuRight[std::get<0>(mit->second)] < 0) {
                const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                    new ::autonomy::localization::atlas::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMP);
            } else if (leftIndex != -1 &&
                       pKF->mvuRight[leftIndex] >= 0)  // Stereo observation
            {
                const cv::KeyPoint& kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->mvuRight[std::get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                if (bRobust) {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMP);
            }

            if (pKF->mpCamera2) {
                int rightIndex = std::get<1>(mit->second);

                if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()) {
                    rightIndex -= pKF->NLeft;

                    Eigen::Matrix<double, 2, 1> obs;
                    cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                    obs << kp.pt.x, kp.pt.y;

                    ::autonomy::localization::atlas::EdgeSE3ProjectXYZToBody*
                        e = new ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZToBody();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    autonomy::common::math::SE3f Trl =
                        pKF->GetRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                           Trl.translation().cast<double>());

                    e->pCamera = pKF->mpCamera2;

                    optimizer.addEdge(e);
                    vpEdgesBody.push_back(e);
                    vpEdgeKFBody.push_back(pKF);
                    vpMapPointEdgeBody.push_back(pMP);
                }
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        } else {
            vbNotIncludedMP[i] = false;
        }
    }

    // Optimize!
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    ADEBUG << "Bundle adjustment optimization finished";

    // Recover optimized data
    // Keyframes
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 =
            static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == pMap->GetOriginKF()->mnId) {
            pKF->SetPose(autonomy::common::math::SE3f(
                SE3quat.rotation().cast<float>(),
                SE3quat.translation().cast<float>()));
        } else {
            pKF->mTcwGBA = autonomy::common::math::SE3d(SE3quat.rotation(),
                                                        SE3quat.translation())
                               .cast<float>();
            pKF->mnBAGlobalForKF = nLoopKF;

            autonomy::common::math::SE3f mTwc = pKF->GetPoseInverse();
            autonomy::common::math::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
            Eigen::Vector3f vector_dist = mTcGBA_c.translation();
            double dist = vector_dist.norm();
            if (dist > 1) {
                int numMonoBadPoints = 0, numMonoOptPoints = 0;
                int numStereoBadPoints = 0, numStereoOptPoints = 0;
                std::vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

                for (std::size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend;
                     i2++) {
                    ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                        vpEdgesMono[i2];
                    MapPoint* pMP = vpMapPointEdgeMono[i2];
                    KeyFrame* pKFedge = vpEdgeKFMono[i2];

                    if (pKF != pKFedge) {
                        continue;
                    }

                    if (pMP->isBad())
                        continue;

                    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                        numMonoBadPoints++;

                    } else {
                        numMonoOptPoints++;
                        vpMonoMPsOpt.push_back(pMP);
                    }
                }

                for (std::size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend;
                     i2++) {
                    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
                    MapPoint* pMP = vpMapPointEdgeStereo[i2];
                    KeyFrame* pKFedge = vpEdgeKFMono[i2];

                    if (pKF != pKFedge) {
                        continue;
                    }

                    if (pMP->isBad())
                        continue;

                    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                        numStereoBadPoints++;
                    } else {
                        numStereoOptPoints++;
                        vpStereoMPsOpt.push_back(pMP);
                    }
                }
            }
        }
    }

    // Points
    for (std::size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + maxKFid + 1));

        if (nLoopKF == pMap->GetOriginKF()->mnId) {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

bool Optimizer::FullInertialBA(Map* pMap, int its, const bool bFixLocal,
                               const long unsigned int nLoopId,
                               bool* pbStopFlag, bool bInit, float priorG,
                               float priorA, Eigen::VectorXd* vSingVal,
                               bool* bHess) {
    if (pMap == nullptr) {
        AERROR << "Full inertial BA rejected a null map" << std::endl;
        return false;
    }
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
    if (vpKFs.empty()) {
        AERROR << "Full inertial BA rejected an empty map" << std::endl;
        return false;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFrame* pIncKF = nullptr;
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF = pKFi;
        bool bFixed = false;
        if (bFixLocal) {
            bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
                     (pKFi->mnBAFixedForKF >= (maxKFid - 1));
            if (!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit) {
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit && pIncKF == nullptr)
        return false;

    if (bInit) {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4 * maxKFid + 2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4 * maxKFid + 3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if (bFixLocal) {
        if (nNonFixed < 3)
            return false;
    }

    // IMU links
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        if (!pKFi->mPrevKF) {
            AINFO << "NOT INERTIAL LINK TO PREVIOUS FRAME!";
            continue;
        }

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                continue;
            if (pKFi->bImu && pKFi->mPrevKF->bImu) {
                pKFi->mpImuPreintegrated->SetNewBias(
                    pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex* VP1 =
                    optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 =
                    optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

                g2o::HyperGraph::Vertex* VG1 = nullptr;
                g2o::HyperGraph::Vertex* VA1 = nullptr;
                g2o::HyperGraph::Vertex* VG2 = nullptr;
                g2o::HyperGraph::Vertex* VA2 = nullptr;
                if (!bInit) {
                    VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) +
                                           2);
                    VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) +
                                           3);
                    VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                    VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
                } else {
                    VG1 = optimizer.vertex(4 * maxKFid + 2);
                    VA1 = optimizer.vertex(4 * maxKFid + 3);
                }

                g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 =
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

                if (!bInit) {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 ||
                        !VA2) {
                        AINFO << "Error" << VP1 << ", " << VV1 << ", " << VG1
                              << ", " << VA1 << ", " << VP2 << ", " << VV2
                              << ", " << VG2 << ", " << VA2 << std::endl;
                        continue;
                    }
                } else {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
                        AINFO << "Error" << VP1 << ", " << VV1 << ", " << VG1
                              << ", " << VA1 << ", " << VP2 << ", " << VV2
                              << std::endl;
                        continue;
                    }
                }

                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(
                    0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(
                    1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(
                    2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(
                    3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(
                    4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(
                    5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit) {
                    EdgeGyroRW* egr = new EdgeGyroRW();
                    egr->setVertex(0, VG1);
                    egr->setVertex(1, VG2);
                    Eigen::Matrix3d InfoG =
                        pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                            .cast<double>()
                            .inverse();
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0, VA1);
                    ear->setVertex(1, VA2);
                    Eigen::Matrix3d InfoA =
                        pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                            .cast<double>()
                            .inverse();
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            } else
                AINFO << pKFi->mnId << " or " << pKFi->mPrevKF->mnId
                      << " no imu" << std::endl;
        }
    }

    if (bInit) {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);

        // Add prior to comon biases
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc* epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA;  //
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro* epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG;  //
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    const unsigned long iniMPid = maxKFid * 5;

    std::vector<bool> vbNotIncludedMP(vpMPs.size(), false);

    for (std::size_t i = 0; i < vpMPs.size(); i++) {
        MapPoint* pMP = vpMPs[i];
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMP->GetObservations();

        bool bAllFixed = true;

        // Set edges
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnId > maxKFid)
                continue;

            if (!pKFi->isBad()) {
                const int leftIndex = std::get<0>(mit->second);
                cv::KeyPoint kpUn;

                if (leftIndex != -1 &&
                    pKFi->mvuRight[std::get<0>(mit->second)] <
                        0)  // Monocular observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex* VP =
                        dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKFi->mnId));
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                } else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >=
                                                  0)  // stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex* VP =
                        dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKFi->mnId));
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if (pKFi->mpCamera2) {  // Monocular right observation
                    int rightIndex = std::get<1>(mit->second);

                    if (rightIndex != -1 &&
                        rightIndex < pKFi->mvKeysRight.size()) {
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double, 2, 1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex* VP =
                            dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(pKFi->mnId));
                        if (bAllFixed)
                            if (!VP->fixed())
                                bAllFixed = false;

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if (bAllFixed) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return false;

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    const double initial_cost = optimizer.activeRobustChi2();
    const int completed_iterations = optimizer.optimize(its);
    optimizer.computeActiveErrors();
    const double optimized_cost = optimizer.activeRobustChi2();
    if (completed_iterations <= 0 ||
        !IsNonIncreasingCost(initial_cost, optimized_cost) ||
        (pbStopFlag != nullptr && *pbStopFlag)) {
        AERROR << "Full inertial BA failed validation: iterations="
               << completed_iterations << ", initial_cost=" << initial_cost
               << ", optimized_cost=" << optimized_cost << std::endl;
        return false;
    }

    std::vector<Eigen::Vector3d> current_keyframe_centers;
    std::vector<Eigen::Vector3d> optimized_keyframe_centers;
    std::vector<Eigen::Vector3d> current_velocities;
    std::vector<Eigen::Vector3d> optimized_velocities;
    current_keyframe_centers.reserve(vpKFs.size());
    optimized_keyframe_centers.reserve(vpKFs.size());
    current_velocities.reserve(vpKFs.size());
    optimized_velocities.reserve(vpKFs.size());
    for (KeyFrame* keyframe : vpKFs) {
        if (keyframe->mnId > maxKFid)
            continue;
        const auto* pose_vertex =
            static_cast<const VertexPose*>(optimizer.vertex(keyframe->mnId));
        if (pose_vertex == nullptr || pose_vertex->estimate().Rcw.empty() ||
            pose_vertex->estimate().tcw.empty()) {
            AERROR << "Full inertial BA produced a missing pose estimate"
                   << std::endl;
            return false;
        }
        for (std::size_t camera_index = 0;
             camera_index < pose_vertex->estimate().Rcw.size();
             ++camera_index) {
            if (!IsFiniteRotation(pose_vertex->estimate().Rcw[camera_index]) ||
                !pose_vertex->estimate().tcw[camera_index].allFinite()) {
                AERROR << "Full inertial BA produced a non-finite pose"
                       << std::endl;
                return false;
            }
        }
        current_keyframe_centers.push_back(
            keyframe->GetPoseInverse().translation().cast<double>());
        optimized_keyframe_centers.push_back(
            -pose_vertex->estimate().Rcw[0].transpose() *
            pose_vertex->estimate().tcw[0]);
        if (!keyframe->bImu)
            continue;
        const auto* velocity_vertex = static_cast<const VertexVelocity*>(
            optimizer.vertex(maxKFid + 3 * keyframe->mnId + 1));
        const auto bias_base_id =
            bInit ? 4 * maxKFid : maxKFid + 3 * keyframe->mnId;
        const auto* gyro_bias_vertex = static_cast<const VertexGyroBias*>(
            optimizer.vertex(bias_base_id + 2));
        const auto* acc_bias_vertex = static_cast<const VertexAccBias*>(
            optimizer.vertex(bias_base_id + 3));
        if (velocity_vertex == nullptr || gyro_bias_vertex == nullptr ||
            acc_bias_vertex == nullptr ||
            !velocity_vertex->estimate().allFinite() ||
            !gyro_bias_vertex->estimate().allFinite() ||
            !acc_bias_vertex->estimate().allFinite()) {
            AERROR << "Full inertial BA produced a non-finite inertial state"
                   << std::endl;
            return false;
        }
        current_velocities.push_back(keyframe->GetVelocity().cast<double>());
        optimized_velocities.push_back(velocity_vertex->estimate());
    }
    std::vector<Eigen::Vector3d> current_map_points;
    std::vector<Eigen::Vector3d> optimized_map_points;
    current_map_points.reserve(vpMPs.size());
    optimized_map_points.reserve(vpMPs.size());
    for (std::size_t index = 0; index < vpMPs.size(); ++index) {
        if (vbNotIncludedMP[index])
            continue;
        const auto* point_vertex = static_cast<const g2o::VertexPointXYZ*>(
            optimizer.vertex(vpMPs[index]->mnId + iniMPid + 1));
        if (point_vertex == nullptr || !point_vertex->estimate().allFinite()) {
            AERROR << "Full inertial BA produced a non-finite map point"
                   << std::endl;
            return false;
        }
        current_map_points.push_back(
            vpMPs[index]->GetWorldPos().cast<double>());
        optimized_map_points.push_back(point_vertex->estimate());
    }

    constexpr double kMaximumRelativeStateScale = 20.0;
    if (!IsRelativeGeometryUpdateReasonable(current_keyframe_centers,
                                            optimized_keyframe_centers,
                                            kMaximumRelativeStateScale) ||
        !IsRelativeGeometryUpdateReasonable(current_map_points,
                                            optimized_map_points,
                                            kMaximumRelativeStateScale) ||
        !IsRelativeMagnitudeUpdateReasonable(current_velocities,
                                             optimized_velocities,
                                             kMaximumRelativeStateScale)) {
        AERROR << "Full inertial BA rejected a catastrophic relative state "
                  "scale change"
               << std::endl;
        return false;
    }

    // Recover optimized data
    // Keyframes
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if (nLoopId == 0) {
            autonomy::common::math::SE3f Tcw(
                VP->estimate().Rcw[0].cast<float>(),
                VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
        } else {
            pKFi->mTcwGBA = autonomy::common::math::SE3f(
                VP->estimate().Rcw[0].cast<float>(),
                VP->estimate().tcw[0].cast<float>());
            pKFi->mnBAGlobalForKF = nLoopId;
        }
        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            if (nLoopId == 0) {
                pKFi->SetVelocity(VV->estimate().cast<float>());
            } else {
                pKFi->mVwbGBA = VV->estimate().cast<float>();
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit) {
                VG = static_cast<VertexGyroBias*>(
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VA = static_cast<VertexAccBias*>(
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            } else {
                VG = static_cast<VertexGyroBias*>(
                    optimizer.vertex(4 * maxKFid + 2));
                VA = static_cast<VertexAccBias*>(
                    optimizer.vertex(4 * maxKFid + 3));
            }

            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
            if (nLoopId == 0) {
                pKFi->SetNewBias(b);
            } else {
                pKFi->mBiasGBA = b;
            }
        }
    }

    // Points
    for (std::size_t i = 0; i < vpMPs.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMPs[i];
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));

        if (nLoopId == 0) {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopId;
        }
    }

    pMap->IncreaseChangeIndex();
    return true;
}

int Optimizer::PoseOptimization(Frame* pFrame) {
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_6_3,
                                    g2o::LinearSolverDense,
                                    g2o::OptimizationAlgorithmLevenberg>();
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    autonomy::common::math::SE3<float> Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZOnlyPose*>
        vpEdgesMono;
    std::vector<
        ::autonomy::localization::atlas::EdgeSE3ProjectXYZOnlyPoseToBody*>
        vpEdgesMono_FHR;
    std::vector<std::size_t> vnIndexEdgeMono, vnIndexEdgeRight;
    vpEdgesMono.reserve(N);
    vpEdgesMono_FHR.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeRight.reserve(N);

    std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    std::vector<std::size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                // Conventional SLAM
                if (!pFrame->mpCamera2) {
                    // Monocular observation
                    if (pFrame->mvuRight[i] < 0) {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZOnlyPose* e =
                                new ::autonomy::localization::atlas::
                                    EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->mpCamera;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    } else  // Stereo observation
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                        const float& kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose* e =
                            new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info =
                            Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
                // SLAM with respect a rigid body
                else {
                    nInitialCorrespondences++;

                    cv::KeyPoint kpUn;

                    if (i < pFrame->Nleft) {  // Left camera observation
                        kpUn = pFrame->mvKeys[i];

                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZOnlyPose* e =
                                new ::autonomy::localization::atlas::
                                    EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->mpCamera;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    } else {
                        kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        pFrame->mvbOutlier[i] = false;

                        ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZOnlyPoseToBody* e =
                                new ::autonomy::localization::atlas::
                                    EdgeSE3ProjectXYZOnlyPoseToBody();

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->mpCamera2;
                        e->Xw = pMP->GetWorldPos().cast<double>();

                        e->mTrl = g2o::SE3Quat(pFrame->GetRelativePoseTrl()
                                                   .unit_quaternion()
                                                   .cast<double>(),
                                               pFrame->GetRelativePoseTrl()
                                                   .translation()
                                                   .cast<double>());

                        optimizer.addEdge(e);

                        vpEdgesMono_FHR.push_back(e);
                        vnIndexEdgeRight.push_back(i);
                    }
                }
            }
        }
    }

    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    for (std::size_t it = 0; it < 4; it++) {
        Tcw = pFrame->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            ::autonomy::localization::atlas::EdgeSE3ProjectXYZOnlyPose* e =
                vpEdgesMono[i];

            const std::size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (std::size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
            ::autonomy::localization::atlas::EdgeSE3ProjectXYZOnlyPoseToBody*
                e = vpEdgesMono_FHR[i];

            const std::size_t idx = vnIndexEdgeRight[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const std::size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                e->setLevel(0);
                pFrame->mvbOutlier[idx] = false;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    autonomy::common::math::SE3<float> pose(
        SE3quat_recov.rotation().cast<float>(),
        SE3quat_recov.translation().cast<float>());
    pFrame->SetPose(pose);

    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag,
                                      Map* pMap, int& num_fixedKF,
                                      int& num_OptKF, int& num_MPs,
                                      int& num_edges) {
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const std::vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    num_fixedKF = 0;
    std::list<MapPoint*> lLocalMapPoints;
    std::set<MapPoint*> sNumObsMP;
    for (std::list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        if (pKFi->mnId == pMap->GetInitKFid()) {
            num_fixedKF = 1;
        }
        std::vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                              vend = vpMPs.end();
             vit != vend; vit++) {
            MapPoint* pMP = *vit;
            if (pMP)
                if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
    // Local Keyframes
    std::list<KeyFrame*> lFixedCameras;
    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        std::map<KeyFrame*, std::tuple<int, int>> observations =
            (*lit)->GetObservations();
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;

    if (num_fixedKF == 0) {
        AINFO
            << "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted";
        return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_6_3,
                                    g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();
    if (pMap->IsInertial())
        solver->setUserLambdaInit(100.0);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // DEBUG LBA
    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // Set Local KeyFrame vertices
    for (std::list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        autonomy::common::math::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = lLocalKeyFrames.size();

    // Set Fixed KeyFrame vertices
    for (std::list<KeyFrame*>::iterator lit = lFixedCameras.begin(),
                                        lend = lFixedCameras.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        autonomy::common::math::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }

    // Set MapPoint vertices
    const std::size_t expected_edge_count =
        CountMapPointObservations(lLocalMapPoints);

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZ*>
        vpEdgesMono;
    vpEdgesMono.reserve(expected_edge_count);

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZToBody*>
        vpEdgesBody;
    vpEdgesBody.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(expected_edge_count);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(expected_edge_count);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    int nEdges = 0;

    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMP->GetObservations();

        // Set edges
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                const int leftIndex = std::get<0>(mit->second);

                // Monocular observation
                if (leftIndex != -1 &&
                    pKFi->mvuRight[std::get<0>(mit->second)] < 0) {
                    const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                        new ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZ();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                } else if (leftIndex != -1 &&
                           pKFi->mvuRight[std::get<0>(mit->second)] >=
                               0)  // Stereo observation
                {
                    const cv::KeyPoint& kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur =
                        pKFi->mvuRight[std::get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e =
                        new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info =
                        Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                if (pKFi->mpCamera2) {
                    int rightIndex = std::get<1>(mit->second);

                    if (rightIndex != -1) {
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ::autonomy::localization::atlas::
                            EdgeSE3ProjectXYZToBody* e = new ::autonomy::
                                localization::atlas::EdgeSE3ProjectXYZToBody();

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(id)));
                        e->setVertex(
                            1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float& invSigma2 =
                            pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        autonomy::common::math::SE3f Trl =
                            pKFi->GetRelativePoseTrl();
                        e->mTrl =
                            g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                         Trl.translation().cast<double>());

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }
    num_edges = nEdges;

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                     vpEdgesStereo.size());

    // Check inlier observations
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    for (std::size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
        ::autonomy::localization::atlas::EdgeSE3ProjectXYZToBody* e =
            vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFBody[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (std::size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    // Keyframes
    for (std::list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 =
            static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        autonomy::common::math::SE3f Tiw(SE3quat.rotation().cast<float>(),
                                         SE3quat.translation().cast<float>());
        pKFi->SetPose(Tiw);
    }

    // Points
    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    const LoopCloser::KeyFrameAndPose& NonCorrectedSim3,
    const LoopCloser::KeyFrameAndPose& CorrectedSim3,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections,
    const bool& bFixScale) {
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_7_3,
                                    g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid +
                                                                     1);
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        nMaxKFid + 1);
    std::vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

    std::vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1);  // For debugging
    Eigen::Vector3d z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minFeat = 100;

    // Set KeyFrame vertices
    for (std::size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopCloser::KeyFrameAndPose::const_iterator it =
            CorrectedSim3.find(pKF);

        if (it != CorrectedSim3.end()) {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        } else {
            autonomy::common::math::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if (pKF->mnId == pMap->GetInitKFid())
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);
        vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec;  // For debugging

        vpVertices[nIDi] = VSim3;
    }

    std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    int count_loop = 0;
    for (std::map<KeyFrame*, std::set<KeyFrame*>>::const_iterator
             mit = LoopConnections.begin(),
             mend = LoopConnections.end();
         mit != mend; mit++) {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const std::set<KeyFrame*>& spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for (std::set<KeyFrame*>::const_iterator sit = spConnections.begin(),
                                                 send = spConnections.end();
             sit != send; sit++) {
            const long unsigned int nIDj = (*sit)->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
                pKF->GetWeight(*sit) < minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);
            count_loop++;
            sInsertedEdges.insert(
                std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
        }
    }

    // Set normal edges
    for (std::size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopCloser::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3.find(pKF);

        if (iti != NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if (pParentKF) {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopCloser::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3.find(pParentKF);

            if (itj != NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const std::set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for (std::set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                                 send = sLoopEdges.end();
             sit != send; sit++) {
            KeyFrame* pLKF = *sit;
            if (pLKF->mnId < pKF->mnId) {
                g2o::Sim3 Slw;

                LoopCloser::KeyFrameAndPose::const_iterator itl =
                    NonCorrectedSim3.find(pLKF);

                if (itl != NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const std::vector<KeyFrame*> vpConnectedKFs =
            pKF->GetCovisiblesByWeight(minFeat);
        for (std::vector<KeyFrame*>::const_iterator vit =
                 vpConnectedKFs.begin();
             vit != vpConnectedKFs.end(); vit++) {
            KeyFrame* pKFn = *vit;
            if (pKFn && pKFn != pParentKF &&
                !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/) {
                if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
                    if (sInsertedEdges.count(
                            std::make_pair(std::min(pKF->mnId, pKFn->mnId),
                                           std::max(pKF->mnId, pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopCloser::KeyFrameAndPose::const_iterator itn =
                        NonCorrectedSim3.find(pKFn);

                    if (itn != NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                      optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                      optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }

        // Inertial edges if inertial
        if (pKF->bImu && pKF->mPrevKF) {
            g2o::Sim3 Spw;
            LoopCloser::KeyFrameAndPose::const_iterator itp =
                NonCorrectedSim3.find(pKF->mPrevKF);
            if (itp != NonCorrectedSim3.end())
                Spw = itp->second;
            else
                Spw = vScw[pKF->mPrevKF->mnId];

            g2o::Sim3 Spi = Spw * Swi;
            g2o::EdgeSim3* ep = new g2o::EdgeSim3();
            ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(pKF->mPrevKF->mnId)));
            ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                 optimizer.vertex(nIDi)));
            ep->setMeasurement(Spi);
            ep->information() = matLambda;
            optimizer.addEdge(ep);
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 =
            static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();

        autonomy::common::math::SE3f Tiw(
            CorrectedSiw.rotation().cast<float>(),
            CorrectedSiw.translation().cast<float>() / s);
        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (std::size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        MapPoint* pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        int nIDr;
        if (pMP->mnCorrectedByKF == pCurKF->mnId) {
            nIDr = pMP->mnCorrectedReference;
        } else {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }

    // TODO Check this changeindex
    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
    KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
    std::vector<KeyFrame*>& vpFixedCorrectedKFs,
    std::vector<KeyFrame*>& vpNonFixedKFs,
    std::vector<MapPoint*>& vpNonCorrectedMPs) {
    AINFO << "Opt_Essential: There are " + std::to_string(vpFixedKFs.size()) +
                 " KFs fixed in the merged map";
    ADEBUG << "Opt_Essential: There are " +
                  std::to_string(vpFixedCorrectedKFs.size()) +
                  " KFs fixed in the old map";
    ADEBUG << "Opt_Essential: There are " +
                  std::to_string(vpNonFixedKFs.size()) +
                  " KFs non-fixed in the merged map";
    ADEBUG << "Opt_Essential: There are " +
                  std::to_string(vpNonCorrectedMPs.size()) +
                  " MPs non-corrected in the merged map";

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_7_3,
                                    g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    Map* pMap = pCurKF->GetMap();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid +
                                                                     1);
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        nMaxKFid + 1);
    std::vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

    std::vector<bool> vpGoodPose(nMaxKFid + 1);
    std::vector<bool> vpBadPose(nMaxKFid + 1);

    const int minFeat = 100;

    for (KeyFrame* pKFi : vpFixedKFs) {
        if (pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        autonomy::common::math::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[nIDi] = Siw.inverse();
        VSim3->setEstimate(Siw);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = false;
    }
    ADEBUG << "Opt_Essential: vpFixedKFs loaded";

    std::set<unsigned long> sIdKF;
    for (KeyFrame* pKFi : vpFixedCorrectedKFs) {
        if (pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        autonomy::common::math::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[nIDi] = Siw.inverse();
        VSim3->setEstimate(Siw);

        autonomy::common::math::SE3d Tcw_bef =
            pKFi->mTcwBefMerge.cast<double>();
        vScw[nIDi] =
            g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = true;
    }

    for (KeyFrame* pKFi : vpNonFixedKFs) {
        if (pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        if (sIdKF.count(
                nIDi))  // It has already added in the corrected merge KFs
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        autonomy::common::math::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        VSim3->setFixed(false);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = false;
        vpBadPose[nIDi] = true;
    }

    std::vector<KeyFrame*> vpKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                  vpNonFixedKFs.size());
    vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
    vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(),
                 vpFixedCorrectedKFs.end());
    vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
    std::set<KeyFrame*> spKFs(vpKFs.begin(), vpKFs.end());

    const Eigen::Matrix<double, 7, 7> matLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    for (KeyFrame* pKFi : vpKFs) {
        int num_connections = 0;
        const int nIDi = pKFi->mnId;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if (vpGoodPose[nIDi])
            correctedSwi = vCorrectedSwc[nIDi];
        if (vpBadPose[nIDi])
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKFi = pKFi->GetParent();

        // Spanning tree edge
        if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
            int nIDj = pParentKFi->mnId;

            g2o::Sim3 Sjw;
            bool bHasRelation = false;

            if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
                Sjw = vCorrectedSwc[nIDj].inverse();
                bHasRelation = true;
            } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
                Sjw = vScw[nIDj];
                bHasRelation = true;
            }

            if (bHasRelation) {
                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3* e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
                num_connections++;
            }
        }

        // Loop edges
        const std::set<KeyFrame*> sLoopEdges = pKFi->GetLoopEdges();
        for (std::set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                                 send = sLoopEdges.end();
             sit != send; sit++) {
            KeyFrame* pLKF = *sit;
            if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
                g2o::Sim3 Slw;
                bool bHasRelation = false;

                if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) {
                    Slw = vCorrectedSwc[pLKF->mnId].inverse();
                    bHasRelation = true;
                } else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId]) {
                    Slw = vScw[pLKF->mnId];
                    bHasRelation = true;
                }

                if (bHasRelation) {
                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3* el = new g2o::EdgeSim3();
                    el->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                      optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                      optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                    num_connections++;
                }
            }
        }

        // Covisibility graph edges
        const std::vector<KeyFrame*> vpConnectedKFs =
            pKFi->GetCovisiblesByWeight(minFeat);
        for (std::vector<KeyFrame*>::const_iterator vit =
                 vpConnectedKFs.begin();
             vit != vpConnectedKFs.end(); vit++) {
            KeyFrame* pKFn = *vit;
            if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
                !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end()) {
                if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {
                    g2o::Sim3 Snw = vScw[pKFn->mnId];
                    bool bHasRelation = false;

                    if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) {
                        Snw = vCorrectedSwc[pKFn->mnId].inverse();
                        bHasRelation = true;
                    } else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId]) {
                        Snw = vScw[pKFn->mnId];
                        bHasRelation = true;
                    }

                    if (bHasRelation) {
                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3* en = new g2o::EdgeSim3();
                        en->setVertex(
                            1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pKFn->mnId)));
                        en->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                        num_connections++;
                    }
                }
            }
        }

        if (num_connections == 0) {
            ADEBUG << "Opt_Essential: KF " + std::to_string(pKFi->mnId) +
                          " has 0 connections";
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (KeyFrame* pKFi : vpNonFixedKFs) {
        if (pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 =
            static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();
        autonomy::common::math::SE3d Tiw(CorrectedSiw.rotation(),
                                         CorrectedSiw.translation() / s);

        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (MapPoint* pMPi : vpNonCorrectedMPs) {
        if (pMPi->isBad())
            continue;

        KeyFrame* pRefKF = pMPi->GetReferenceKeyFrame();
        while (pRefKF->isBad()) {
            if (!pRefKF) {
                ADEBUG << "MP " + std::to_string(pMPi->mnId) +
                              " without a valid reference KF";
                break;
            }

            pMPi->EraseObservation(pRefKF);
            pRefKF = pMPi->GetReferenceKeyFrame();
        }

        if (vpBadPose[pRefKF->mnId]) {
            autonomy::common::math::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
            autonomy::common::math::SE3f Twr = pRefKF->GetPoseInverse();

            Eigen::Vector3f eigCorrectedP3Dw =
                Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
            pMPi->SetWorldPos(eigCorrectedP3Dw);

            pMPi->UpdateNormalAndDepth();
        } else {
            AINFO << "ERROR: MapPoint has a reference KF from another map"
                  << std::endl;
        }
    }
}

int Optimizer::OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2,
                            std::vector<MapPoint*>& vpMatches1,
                            g2o::Sim3& g2oS12, const float th2,
                            const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian,
                            const bool bAllPoints) {
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverDense,
                                    g2o::OptimizationAlgorithmLevenberg>();
    optimizer.setAlgorithm(solver);

    // Camera poses
    const Eigen::Matrix3f R1w = pKF1->GetRotation();
    const Eigen::Vector3f t1w = pKF1->GetTranslation();
    const Eigen::Matrix3f R2w = pKF2->GetRotation();
    const Eigen::Vector3f t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    ::autonomy::localization::atlas::VertexSim3Expmap* vSim3 =
        new ::autonomy::localization::atlas::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->pCamera1 = pKF1->mpCamera;
    vSim3->pCamera2 = pKF2->mpCamera;
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const std::vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    std::vector<::autonomy::localization::atlas::EdgeSim3ProjectXYZ*> vpEdges12;
    std::vector<::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ*>
        vpEdges21;
    std::vector<std::size_t> vnIndexEdge;
    std::vector<bool> vbIsInKF2;

    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);
    vbIsInKF2.reserve(2 * N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;
    int nBadMPs = 0;
    int nInKF2 = 0;
    int nOutKF2 = 0;
    int nMatchWithoutMP = 0;

    std::vector<int> vIdsOnlyInKF2;

    for (int i = 0; i < N; i++) {
        if (!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2 * i + 1;
        const int id2 = 2 * (i + 1);

        const int i2 = std::get<0>(pMP2->GetIndexInKeyFrame(pKF2));

        Eigen::Vector3f P3D1c;
        Eigen::Vector3f P3D2c;

        if (pMP1 && pMP2) {
            if (!pMP1->isBad() && !pMP2->isBad()) {
                g2o::VertexPointXYZ* vPoint1 = new g2o::VertexPointXYZ();
                Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
                P3D1c = R1w * P3D1w + t1w;
                vPoint1->setEstimate(P3D1c.cast<double>());
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w * P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            } else {
                nBadMPs++;
                continue;
            }
        } else {
            nMatchWithoutMP++;

            // TODO The 3D position in KF1 doesn't exist
            if (!pMP2->isBad()) {
                g2o::VertexPointXYZ* vPoint2 = new g2o::VertexPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w * P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);

                vIdsOnlyInKF2.push_back(id2);
            }
            continue;
        }

        if (i2 < 0 && !bAllPoints) {
            ADEBUG << "    Remove point -> i2: " + std::to_string(i2) +
                          "; bAllPoints: " + std::to_string(bAllPoints);
            continue;
        }

        if (P3D2c(2) < 0) {
            ADEBUG << "Sim3: Z coordinate is negative";
            continue;
        }

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double, 2, 1> obs1;
        const cv::KeyPoint& kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        ::autonomy::localization::atlas::EdgeSim3ProjectXYZ* e12 =
            new ::autonomy::localization::atlas::EdgeSim3ProjectXYZ();

        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double, 2, 1> obs2;
        cv::KeyPoint kpUn2;
        bool inKF2;
        if (i2 >= 0) {
            kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;
            inKF2 = true;

            nInKF2++;
        } else {
            float invz = 1 / P3D2c(2);
            float x = P3D2c(0) * invz;
            float y = P3D2c(1) * invz;

            obs2 << x, y;
            kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

            inKF2 = false;
            nOutKF2++;
        }

        ::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ* e21 =
            new ::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);

        vbIsInKF2.push_back(inKF2);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad = 0;
    int nBadOutKF2 = 0;
    for (std::size_t i = 0; i < vpEdges12.size(); i++) {
        ::autonomy::localization::atlas::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ* e21 =
            vpEdges21[i];
        if (!e12 || !e21)
            continue;

        if (e12->chi2() > th2 || e21->chi2() > th2) {
            std::size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(nullptr);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i] = static_cast<
                ::autonomy::localization::atlas::EdgeSim3ProjectXYZ*>(nullptr);
            vpEdges21[i] = static_cast<
                ::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ*>(
                nullptr);
            nBad++;

            if (!vbIsInKF2[i]) {
                nBadOutKF2++;
            }
            continue;
        }

        // Check if remove the robust adjustment improve the result
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    int nMoreIterations;
    if (nBad > 0)
        nMoreIterations = 10;
    else
        nMoreIterations = 5;

    if (nCorrespondences - nBad < 10)
        return 0;

    // Optimize again only with inliers
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for (std::size_t i = 0; i < vpEdges12.size(); i++) {
        ::autonomy::localization::atlas::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ::autonomy::localization::atlas::EdgeInverseSim3ProjectXYZ* e21 =
            vpEdges21[i];
        if (!e12 || !e21)
            continue;

        e12->computeError();
        e21->computeError();

        if (e12->chi2() > th2 || e21->chi2() > th2) {
            std::size_t idx = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint*>(nullptr);
        } else {
            nIn++;
        }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov =
        static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nIn;
}

void Optimizer::LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                int& num_fixedKF, int& num_OptKF, int& num_MPs,
                                int& num_edges, bool bLarge, bool bRecInit) {
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt = 10;
    int opt_it = 10;
    if (bLarge) {
        maxOpt = 25;
        opt_it = 4;
    }
    const int Nd =
        std::min(static_cast<int>(pCurrentMap->KeyFramesInMap()) - 2, maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    std::vector<KeyFrame*> vpOptimizableKFs;
    const std::vector<KeyFrame*> vpNeighsKFs =
        pKF->GetVectorCovisibleKeyFrames();
    std::list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for (int i = 1; i < Nd; i++) {
        if (vpOptimizableKFs.back()->mPrevKF) {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        } else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    std::list<MapPoint*> lLocalMapPoints;
    for (int i = 0; i < N; i++) {
        std::vector<MapPoint*> vpMPs =
            vpOptimizableKFs[i]->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                              vend = vpMPs.end();
             vit != vend; vit++) {
            MapPoint* pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    std::list<KeyFrame*> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF) {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    } else {
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
        if (lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFrame* pKFi = vpNeighsKFs[i];
        if (pKFi->mnBALocalForKF == pKF->mnId ||
            pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
            lpOptVisKFs.push_back(pKFi);

            std::vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                                  vend = vpMPs.end();
                 vit != vend; vit++) {
                MapPoint* pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId) {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        std::map<KeyFrame*, std::tuple<int, int>> observations =
            (*lit)->GetObservations();
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()) {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if (lFixedKeyFrames.size() >= maxFixKF)
            break;
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

    if (bLarge) {
        auto* solver =
            CreateOptimizationAlgorithm<g2o::BlockSolverX,
                                        g2o::LinearSolverEigen,
                                        g2o::OptimizationAlgorithmLevenberg>();
        solver->setUserLambdaInit(
            1e-2);  // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    } else {
        auto* solver =
            CreateOptimizationAlgorithm<g2o::BlockSolverX,
                                        g2o::LinearSolverEigen,
                                        g2o::OptimizationAlgorithmLevenberg>();
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }

    // Set Local temporal KeyFrame vertices
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local visual KeyFrame vertices
    for (std::list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                        itEnd = lpOptVisKFs.end();
         it != itEnd; it++) {
        KeyFrame* pKFi = *it;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // Set Fixed KeyFrame vertices
    for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                        lend = lFixedKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if (pKFi->bImu)  // This should be done only for keyframe just before
                         // temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    std::vector<EdgeInertial*> vei(N, nullptr);
    std::vector<EdgeGyroRW*> vegr(N, nullptr);
    std::vector<EdgeAccRW*> vear(N, nullptr);

    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if (!pKFi->mPrevKF) {
            AINFO << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << std::endl;
            continue;
        }
        if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VG1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
            g2o::HyperGraph::Vertex* VA1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
            g2o::HyperGraph::Vertex* VA2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

            if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                AERROR << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", "
                       << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2
                       << ", " << VA2 << std::endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(
                0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(
                1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(
                2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(
                3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(
                4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(
                5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if (i == N - 1 || bRecInit) {
                // All inertial residuals are included without robust cost
                // function, but not that one linking the last optimizable
                // keyframe inside of the local window and the first fixed
                // keyframe out. The information matrix for this measurement is
                // also downweighted. This is done to avoid accumulating error
                // due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if (i == N - 1)
                    vei[i]->setInformation(vei[i]->information() * 1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, VG1);
            vegr[i]->setVertex(1, VG2);
            Eigen::Matrix3d InfoG =
                pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, VA1);
            vear[i]->setVertex(1, VA2);
            Eigen::Matrix3d InfoA =
                pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[i]->setInformation(InfoA);

            optimizer.addEdge(vear[i]);
        } else
            AINFO << "ERROR building inertial edge" << std::endl;
    }

    // Set MapPoint vertices
    const std::size_t expected_edge_count =
        CountMapPointObservations(lLocalMapPoints);

    // Mono
    std::vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(expected_edge_count);

    // Stereo
    std::vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(expected_edge_count);

    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid * 5;

    std::map<int, int> mVisEdges;
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                        lend = lFixedKeyFrames.end();
         lit != lend; lit++) {
        mVisEdges[(*lit)->mnId] = 0;
    }

    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMP->GetObservations();

        // Create visual constraints
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId)
                continue;

            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                const int leftIndex = std::get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if (leftIndex != -1)  // Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 =
                        pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if (pKFi->mpCamera2) {
                    int rightIndex = std::get<1>(mit->second);

                    if (rightIndex != -1) {
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(id)));
                        e->setVertex(
                            1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float& invSigma2 =
                            pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }

    // AINFO << "Total map points: " << lLocalMapPoints.size() << endl;
    for (std::map<int, int>::iterator mit = mVisEdges.begin(),
                                      mend = mVisEdges.end();
         mit != mend; mit++) {
        assert(mit->second >= 3);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it);  // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth < 10.f;

        if (pMP->isBad())
            continue;

        if ((e->chi2() > chi2Mono2 && !bClose) ||
            (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    // Stereo
    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > chi2Stereo2) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex and erase outliers
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    // TODO: Some convergence problems have been detected here
    if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge)  // bGN)
    {
        AINFO << "FAIL LOCAL-INERTIAL BA!!!!" << std::endl;
        return;
    }

    if (!vToErase.empty()) {
        for (std::size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                        lend = lFixedKeyFrames.end();
         lit != lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        autonomy::common::math::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                                         VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;

        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    // Local visual KeyFrame
    for (std::list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                        itEnd = lpOptVisKFs.end();
         it != itEnd; it++) {
        KeyFrame* pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        autonomy::common::math::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                                         VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;
    }

    // Points
    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H,
                                       const int& start, const int& end) {
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end - start + 1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
    }
    if (a > 0 && c > 0) {
        Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
    }
    if (c > 0) {
        Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        Hn.block(a + c, a + c, b, b),
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
        svd.singularValues();
    for (int i = 0; i < b; ++i) {
        if (singularValues_inv(i) > 1e-6)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                            svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) =
        Hn.block(0, 0, a + c, a + c) -
        Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0) {
        res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0) {
        res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
        res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
    }

    res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    return res;
}

bool Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg,
                                     double& scale, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, bool bMono,
                                     Eigen::MatrixXd& covInertial,
                                     bool bFixedVel, bool bGauss, float priorG,
                                     float priorA) {
    if (pMap == nullptr)
        return false;
    ADEBUG << "inertial optimization";
    int its = 200;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    if (vpKFs.empty())
        return false;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();

    if (priorG != 0.f)
        solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid + (pKFi->mnId) + 1);
        if (bFixedVel)
            VV->setFixed(true);
        else
            VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid * 2 + 2);
    if (bFixedVel)
        VG->setFixed(true);
    else
        VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid * 2 + 3);
    if (bFixedVel)
        VA->setFixed(true);
    else
        VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(maxKFid * 2 + 4);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(maxKFid * 2 + 5);
    VS->setFixed(!bMono);  // Fixed for stereo case
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    std::vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    std::vector<std::pair<KeyFrame*, KeyFrame*>> vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());
    // AINFO << "build optimization graph" << std::endl;

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                continue;
            if (!pKFi->mpImuPreintegrated)
                AINFO << "Not preintegrated measurement" << std::endl;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
            if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
                AINFO << "Error" << VP1 << ", " << VV1 << ", " << VG << ", "
                      << VA << ", " << VP2 << ", " << VV2 << ", " << VGDir
                      << ", " << VS << std::endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,
                          dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(std::make_pair(pKFi->mPrevKF, pKFi));
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    const double initial_cost = optimizer.activeRobustChi2();
    const int completed_iterations = optimizer.optimize(its);
    optimizer.computeActiveErrors();
    const double optimized_cost = optimizer.activeRobustChi2();

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
    const Eigen::Vector3d optimized_bg = VG->estimate();
    const Eigen::Vector3d optimized_ba = VA->estimate();
    const double optimized_scale = VS->estimate();
    const Eigen::Matrix3d optimized_rwg = VGDir->estimate().Rwg;
    if (completed_iterations <= 0 ||
        !IsNonIncreasingCost(initial_cost, optimized_cost) ||
        !std::isfinite(optimized_scale) || optimized_scale <= 0.0 ||
        optimized_scale > 100.0 || !IsFiniteRotation(optimized_rwg) ||
        !optimized_bg.allFinite() || optimized_bg.norm() > 1.0 ||
        !optimized_ba.allFinite() || optimized_ba.norm() > 20.0) {
        AERROR << "Inertial optimization failed validation: iterations="
               << completed_iterations << ", initial_cost=" << initial_cost
               << ", optimized_cost=" << optimized_cost
               << ", scale=" << optimized_scale
               << ", gyro_bias_norm=" << optimized_bg.norm()
               << ", acc_bias_norm=" << optimized_ba.norm() << std::endl;
        return false;
    }
    for (KeyFrame* keyframe : vpKFs) {
        if (keyframe->mnId > maxKFid)
            continue;
        const auto* velocity_vertex = static_cast<const VertexVelocity*>(
            optimizer.vertex(maxKFid + keyframe->mnId + 1));
        if (velocity_vertex == nullptr ||
            !velocity_vertex->estimate().allFinite() ||
            velocity_vertex->estimate().norm() > 1e4) {
            AERROR << "Inertial optimization produced an invalid velocity"
                   << std::endl;
            return false;
        }
    }

    Vector6d vb;
    vb << optimized_bg, optimized_ba;
    bg = optimized_bg;
    ba = optimized_ba;
    scale = optimized_scale;
    Rwg = optimized_rwg;

    IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

    // Keyframes velocities and biases
    const int N = vpKFs.size();
    for (std::size_t i = 0; i < N; i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(
            optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
        Eigen::Vector3d Vw = VV->estimate();  // Velocity is scaled after
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        } else
            pKFi->SetNewBias(b);
    }
    return true;
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, float priorG,
                                     float priorA) {
    int its = 200;  // Check number of iterations
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();
    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid + (pKFi->mnId) + 1);
        VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid * 2 + 2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid * 2 + 3);
    VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
    VGDir->setId(maxKFid * 2 + 4);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(1.0);
    VS->setId(maxKFid * 2 + 5);
    VS->setFixed(true);  // Fixed since scale is obtained from already well
                         // initialized map
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    std::vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    std::vector<std::pair<KeyFrame*, KeyFrame*>> vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());

    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                continue;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid * 2 + 2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid * 2 + 3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid * 2 + 5);
            if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
                AINFO << "Error" << VP1 << ", " << VV1 << ", " << VG << ", "
                      << VA << ", " << VP2 << ", " << VV2 << ", " << VGDir
                      << ", " << VS << std::endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,
                          dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(std::make_pair(pKFi->mPrevKF, pKFi));
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();

    IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

    // Keyframes velocities and biases
    const int N = vpKFs.size();
    for (std::size_t i = 0; i < N; i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(
            optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
        Eigen::Vector3d Vw = VV->estimate();
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        } else
            pKFi->SetNewBias(b);
    }
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg,
                                     double& scale) {
    int its = 10;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmGaussNewton>();
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (all variables are fixed)
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid + 1 + (pKFi->mnId));
        VV->setFixed(true);
        optimizer.addVertex(VV);

        // Vertex of fixed biases
        VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
        VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
        VG->setFixed(true);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(vpKFs.front());
        VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
        VA->setFixed(true);
        optimizer.addVertex(VA);
    }

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(4 * (maxKFid + 1));
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(4 * (maxKFid + 1) + 1);
    VS->setFixed(false);
    optimizer.addVertex(VS);

    // Graph edges
    int count_edges = 0;
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                continue;

            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex((maxKFid + 1) + pKFi->mnId);
            g2o::HyperGraph::Vertex* VG =
                optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VA =
                optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VGDir =
                optimizer.vertex(4 * (maxKFid + 1));
            g2o::HyperGraph::Vertex* VS =
                optimizer.vertex(4 * (maxKFid + 1) + 1);
            if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
                AINFO << "Error" + std::to_string(VP1->id()) + ", " +
                             std::to_string(VV1->id()) + ", " +
                             std::to_string(VG->id()) + ", " +
                             std::to_string(VA->id()) + ", " +
                             std::to_string(VP2->id()) + ", " +
                             std::to_string(VV2->id()) + ", " +
                             std::to_string(VGDir->id()) + ", " +
                             std::to_string(VS->id());

                continue;
            }
            count_edges++;
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,
                          dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ei->setRobustKernel(rk);
            rk->setDelta(1.f);
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(its);
    optimizer.computeActiveErrors();
    float err_end = optimizer.activeRobustChi2();
    // Recover optimized data
    scale = VS->estimate();
    Rwg = VGDir->estimate().Rwg;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF,
                                      std::vector<KeyFrame*> vpAdjustKF,
                                      std::vector<KeyFrame*> vpFixedKF,
                                      bool* pbStopFlag) {
    bool bShowImages = false;

    std::vector<MapPoint*> vpMPs;

    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolver_6_3,
                                    g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    std::set<KeyFrame*> spKeyFrameBA;

    Map* pCurrentMap = pMainKF->GetMap();

    // Set fixed KeyFrame vertices
    int numInsertedPoints = 0;
    for (KeyFrame* pKFi : vpFixedKF) {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
            AINFO << "ERROR LBA: KF is bad or is not in the current map";
            continue;
        }

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        autonomy::common::math::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;

        std::set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for (MapPoint* pMPi : spViewMPs) {
            if (pMPi)
                if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

                    if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge = pMainKF->mnId;
                        numInsertedPoints++;
                    }
        }

        spKeyFrameBA.insert(pKFi);
    }

    // Set non fixed Keyframe vertices
    std::set<KeyFrame*> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0;
    for (KeyFrame* pKFi : vpAdjustKF) {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            continue;

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        autonomy::common::math::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;

        std::set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for (MapPoint* pMPi : spViewMPs) {
            if (pMPi) {
                if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
                    if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge = pMainKF->mnId;
                        numInsertedPoints++;
                    }
                }
            }
        }

        spKeyFrameBA.insert(pKFi);
    }

    const std::size_t expected_edge_count = CountMapPointObservations(vpMPs);

    std::vector<::autonomy::localization::atlas::EdgeSE3ProjectXYZ*>
        vpEdgesMono;
    vpEdgesMono.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(expected_edge_count);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(expected_edge_count);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    std::map<KeyFrame*, int> mpObsKFs;
    std::map<KeyFrame*, int> mpObsFinalKFs;
    std::map<MapPoint*, int> mpObsMPs;
    for (unsigned int i = 0; i < vpMPs.size(); ++i) {
        MapPoint* pMPi = vpMPs[i];
        if (pMPi->isBad())
            continue;

        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
        const int id = pMPi->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMPi->GetObservations();
        int nEdges = 0;
        // SET EDGES
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end(); mit++) {
            KeyFrame* pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid ||
                pKF->mnBALocalForMerge != pMainKF->mnId ||
                !pKF->GetMapPoint(std::get<0>(mit->second)))
                continue;

            nEdges++;

            const cv::KeyPoint& kpUn = pKF->mvKeysUn[std::get<0>(mit->second)];

            if (pKF->mvuRight[std::get<0>(mit->second)] < 0)  // Monocular
            {
                mpObsMPs[pMPi]++;
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                    new ::autonomy::localization::atlas::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);

                mpObsKFs[pKF]++;
            } else  // RGBD or Stereo
            {
                mpObsMPs[pMPi] += 2;
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->mvuRight[std::get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber3D);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    std::map<unsigned long int, int> mWrongObsKF;
    if (bDoMore) {
        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
                badMonoMP++;
            }
            e->setRobustKernel(0);
        }

        for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                e->setLevel(1);
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        AINFO << "[BA]: First optimization(Huber), there are " +
                     std::to_string(badMonoMP) + " monocular and " +
                     std::to_string(badStereoMP) + " stereo bad edges";

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    std::set<MapPoint*> spErasedMPs;
    std::set<KeyFrame*> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    ADEBUG << "[BA]: Second optimization, there are " +
                  std::to_string(badMonoMP) + " monocular and " +
                  std::to_string(badStereoMP) + " sterero bad edges";

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (std::size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    for (unsigned int i = 0; i < vpMPs.size(); ++i) {
        MapPoint* pMPi = vpMPs[i];
        if (pMPi->isBad())
            continue;

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMPi->GetObservations();
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end(); mit++) {
            KeyFrame* pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid ||
                pKF->mnBALocalForKF != pMainKF->mnId ||
                !pKF->GetMapPoint(std::get<0>(mit->second)))
                continue;

            if (pKF->mvuRight[std::get<0>(mit->second)] < 0)  // Monocular
            {
                mpObsFinalKFs[pKF]++;
            } else  // RGBD or Stereo
            {
                mpObsFinalKFs[pKF]++;
            }
        }
    }

    // Recover optimized data
    // Keyframes
    for (KeyFrame* pKFi : vpAdjustKF) {
        if (pKFi->isBad())
            continue;

        g2o::VertexSE3Expmap* vSE3 =
            static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        autonomy::common::math::SE3f Tiw(SE3quat.rotation().cast<float>(),
                                         SE3quat.translation().cast<float>());

        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        std::vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
        std::vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

        for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            ::autonomy::localization::atlas::EdgeSE3ProjectXYZ* e =
                vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if (pKFi != pKFedge) {
                continue;
            }

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);

            } else {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }
        }

        for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if (pKFi != pKFedge) {
                continue;
            }

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            } else {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }

        pKFi->SetPose(Tiw);
    }

    // Points
    for (MapPoint* pMPi : vpMPs) {
        if (pMPi->isBad())
            continue;

        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMPi->mnId + maxKFid + 1));
        pMPi->SetWorldPos(vPoint->estimate().cast<float>());
        pMPi->UpdateNormalAndDepth();
    }
}

void Optimizer::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF,
                                bool* pbStopFlag, Map* pMap,
                                LoopCloser::KeyFrameAndPose& corrPoses) {
    const int Nd = 6;
    const unsigned long maxKFid = pCurrKF->mnId;

    std::vector<KeyFrame*> vpOptimizableKFs;
    vpOptimizableKFs.reserve(2 * Nd);

    // For cov KFS, inertial parameters are not optimized
    const int maxCovKF = 30;
    std::vector<KeyFrame*> vpOptimizableCovKFs;
    vpOptimizableCovKFs.reserve(maxCovKF);

    // Add sliding window for current KF
    vpOptimizableKFs.push_back(pCurrKF);
    pCurrKF->mnBALocalForKF = pCurrKF->mnId;
    for (int i = 1; i < Nd; i++) {
        if (vpOptimizableKFs.back()->mPrevKF) {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        } else
            break;
    }

    std::list<KeyFrame*> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF) {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
    } else {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    vpOptimizableKFs.push_back(pMergeKF);
    pMergeKF->mnBALocalForKF = pCurrKF->mnId;

    // Previous KFs
    for (int i = 1; i < (Nd / 2); i++) {
        if (vpOptimizableKFs.back()->mPrevKF) {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        } else
            break;
    }

    // We fix just once the old map
    if (vpOptimizableKFs.back()->mPrevKF) {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
    } else {
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Next KFs
    if (pMergeKF->mNextKF) {
        vpOptimizableKFs.push_back(pMergeKF->mNextKF);
        vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    }

    while (vpOptimizableKFs.size() < (2 * Nd)) {
        if (vpOptimizableKFs.back()->mNextKF) {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        } else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by optimizable keyframes
    std::list<MapPoint*> lLocalMapPoints;
    std::map<MapPoint*, int> mLocalObs;
    for (int i = 0; i < N; i++) {
        std::vector<MapPoint*> vpMPs =
            vpOptimizableKFs[i]->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                              vend = vpMPs.end();
             vit != vend; vit++) {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be
            // added several times to lLocalMapPoints
            MapPoint* pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pCurrKF->mnId) {
                        mLocalObs[pMP] = 1;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pCurrKF->mnId;
                    } else {
                        mLocalObs[pMP]++;
                    }
        }
    }

    std::vector<std::pair<MapPoint*, int>> pairs;
    pairs.reserve(mLocalObs.size());
    for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
        pairs.push_back(*itr);
    std::sort(pairs.begin(), pairs.end(), sortByVal);

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
    // Local Keyframes
    int i = 0;
    for (std::vector<std::pair<MapPoint*, int>>::iterator lit = pairs.begin(),
                                                          lend = pairs.end();
         lit != lend; lit++, i++) {
        std::map<KeyFrame*, std::tuple<int, int>> observations =
            lit->first->GetObservations();
        if (i >= maxCovKF)
            break;
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
                pKFi->mnBAFixedForKF !=
                    pCurrKF->mnId)  // If optimizable or already included...
            {
                pKFi->mnBALocalForKF = pCurrKF->mnId;
                if (!pKFi->isBad()) {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }

    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();

    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Set Local KeyFrame vertices
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local cov keyframes vertices
    int Ncov = vpOptimizableCovKFs.size();
    for (int i = 0; i < Ncov; i++) {
        KeyFrame* pKFi = vpOptimizableCovKFs[i];

        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Fixed KeyFrame vertices
    for (std::list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                        lend = lFixedKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    std::vector<EdgeInertial*> vei(N, nullptr);
    std::vector<EdgeGyroRW*> vegr(N, nullptr);
    std::vector<EdgeAccRW*> vear(N, nullptr);
    for (int i = 0; i < N; i++) {
        // AINFO << "inserting inertial edge " << i << endl;
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if (!pKFi->mPrevKF) {
            AWARN << "No inertial link to the previous frame";
            continue;
        }
        if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VG1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
            g2o::HyperGraph::Vertex* VA1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
            g2o::HyperGraph::Vertex* VA2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

            if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                AERROR << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", "
                       << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2
                       << ", " << VA2 << std::endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(
                0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(
                1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(
                2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(
                3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(
                4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(
                5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            // TODO Uncomment
            g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(rki);
            rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, VG1);
            vegr[i]->setVertex(1, VG2);
            Eigen::Matrix3d InfoG =
                pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, VA1);
            vear[i]->setVertex(1, VA2);
            Eigen::Matrix3d InfoA =
                pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[i]->setInformation(InfoA);
            optimizer.addEdge(vear[i]);
        } else
            AINFO << "ERROR building inertial edge";
    }

    AINFO << "end inserting inertial edges";

    // Set MapPoint vertices
    const std::size_t expected_edge_count =
        CountMapPointObservations(lLocalMapPoints);

    // Mono
    std::vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(expected_edge_count);

    // Stereo
    std::vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(expected_edge_count);

    std::vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(expected_edge_count);

    std::vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(expected_edge_count);

    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid * 5;

    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        if (!pMP)
            continue;

        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const std::map<KeyFrame*, std::tuple<int, int>> observations =
            pMP->GetObservations();

        // Create visual constraints
        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (!pKFi)
                continue;

            if ((pKFi->mnBALocalForKF != pCurrKF->mnId) &&
                (pKFi->mnBAFixedForKF != pCurrKF->mnId))
                continue;

            if (pKFi->mnId > maxKFid) {
                continue;
            }

            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(pKFi->mnId) == nullptr)
                continue;

            if (!pKFi->isBad()) {
                const cv::KeyPoint& kpUn =
                    pKFi->mvKeysUn[std::get<0>(mit->second)];

                if (pKFi->mvuRight[std::get<0>(mit->second)] <
                    0)  // Monocular observation
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else  // stereo observation
                {
                    const float kp_ur =
                        pKFi->mvuRight[std::get<0>(mit->second)];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float& invSigma2 =
                        pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > chi2Mono2) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    // Stereo
    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > chi2Stereo2) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(std::make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex and erase outliers
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    if (!vToErase.empty()) {
        for (std::size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    // Keyframes
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        autonomy::common::math::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                                         VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        autonomy::common::math::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses[pKFi] = g2oSiw;

        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    for (int i = 0; i < Ncov; i++) {
        KeyFrame* pKFi = vpOptimizableCovKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        autonomy::common::math::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                                         VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        autonomy::common::math::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses[pKFi] = g2oSiw;

        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    // Points
    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame* pFrame,
                                                    bool bRecInit) {
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverDense,
                                    g2o::OptimizationAlgorithmGaussNewton>();
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(solver);

    int nInitialMonoCorrespondences = 0;
    int nInitialStereoCorrespondences = 0;
    int nInitialCorrespondences = 0;

    // Set Frame vertex
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    // Set MapPoint vertices
    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft != -1);

    std::vector<EdgeMonoOnlyPose*> vpEdgesMono;
    std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    std::vector<std::size_t> vnIndexEdgeMono;
    std::vector<std::size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                cv::KeyPoint kpUn;

                // Left monocular observation
                if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                    if (i < Nleft)  // pair left-right
                        kpUn = pFrame->mvKeys[i];
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e =
                        new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if (!bRight) {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e =
                        new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 =
                        pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float& invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if (bRight && i >= Nleft) {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e =
                        new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }
    }
    nInitialCorrespondences =
        nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    KeyFrame* pKF = pFrame->mpLastKeyFrame;
    VertexPose* VPk = new VertexPose(pKF);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);
    VertexVelocity* VVk = new VertexVelocity(pKF);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);
    VertexGyroBias* VGk = new VertexGyroBias(pKF);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);
    VertexAccBias* VAk = new VertexAccBias(pKF);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);

    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0, VGk);
    egr->setVertex(1, VG);
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                .cast<double>()
                                .inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0, VAk);
    ear->setVertex(1, VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                .cast<double>()
                                .inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
    float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

    int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers = 0;
    for (std::size_t it = 0; it < 4; it++) {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        float chi2close = 1.5 * chi2Mono[it];

        // For monocular observations
        for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const std::size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

            if ((chi2 > chi2Mono[it] && !bClose) ||
                (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBadMono++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        // For stereo observations
        for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const std::size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);  // not included in next optimization
                nBadStereo++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        nInliers = nInliersMono + nInliersStereo;
        nBad = nBadMono + nBadStereo;

        if (optimizer.edges().size() < 10) {
            break;
        }
    }

    // If not too much tracks, recover not too bad points
    if ((nInliers < 30) && !bRecInit) {
        nBad = 0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        for (std::size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
            const std::size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2() < chi2MonoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
        for (std::size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend;
             i++) {
            const std::size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2() < chi2StereoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
    }

    // Recover optimized pose, velocity and biases
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                               VP->estimate().twb.cast<float>(),
                               VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // Recover Hessian, marginalize keyFframe states and generate new prior for
    // frame
    Eigen::Matrix<double, 15, 15> H;
    H.setZero();

    H.block<9, 9>(0, 0) += ei->GetHessian2();
    H.block<3, 3>(9, 9) += egr->GetHessian2();
    H.block<3, 3>(12, 12) += ear->GetHessian2();

    int tot_in = 0, tot_out = 0;
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const std::size_t idx = vnIndexEdgeMono[i];

        if (!pFrame->mvbOutlier[idx]) {
            H.block<6, 6>(0, 0) += e->GetHessian();
            tot_in++;
        } else
            tot_out++;
    }

    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const std::size_t idx = vnIndexEdgeStereo[i];

        if (!pFrame->mvbOutlier[idx]) {
            H.block<6, 6>(0, 0) += e->GetHessian();
            tot_in++;
        } else
            tot_out++;
    }

    pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,
                                          VP->estimate().twb, VV->estimate(),
                                          VG->estimate(), VA->estimate(), H);

    return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit) {
    g2o::SparseOptimizer optimizer;
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverDense,
                                    g2o::OptimizationAlgorithmGaussNewton>();
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    int nInitialMonoCorrespondences = 0;
    int nInitialStereoCorrespondences = 0;
    int nInitialCorrespondences = 0;

    // Set Current Frame vertex
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    // Set MapPoint vertices
    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft != -1);

    std::vector<EdgeMonoOnlyPose*> vpEdgesMono;
    std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    std::vector<std::size_t> vnIndexEdgeMono;
    std::vector<std::size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                cv::KeyPoint kpUn;
                // Left monocular observation
                if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
                    if (i < Nleft)  // pair left-right
                        kpUn = pFrame->mvKeys[i];
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e =
                        new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if (!bRight) {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e =
                        new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 =
                        pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float& invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if (bRight && i >= Nleft) {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e =
                        new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }
    }

    nInitialCorrespondences =
        nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    // Set Previous Frame Vertex
    Frame* pFp = pFrame->mpPrevFrame;

    VertexPose* VPk = new VertexPose(pFp);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    VertexVelocity* VVk = new VertexVelocity(pFp);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    VertexGyroBias* VGk = new VertexGyroBias(pFp);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    VertexAccBias* VAk = new VertexAccBias(pFp);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);

    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0, VGk);
    egr->setVertex(1, VG);
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                .cast<double>()
                                .inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0, VAk);
    ear->setVertex(1, VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                .cast<double>()
                                .inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    if (!pFp->mpcpi)
        AINFO << "pFp->mpcpi does not exist!!!\nPrevious Frame " +
                     std::to_string(pFp->mnId);

    EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

    ep->setVertex(0, VPk);
    ep->setVertex(1, VVk);
    ep->setVertex(2, VGk);
    ep->setVertex(3, VAk);
    g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
    ep->setRobustKernel(rkp);
    rkp->setDelta(5);
    optimizer.addEdge(ep);

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers = 0;
    for (std::size_t it = 0; it < 4; it++) {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        float chi2close = 1.5 * chi2Mono[it];

        for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const std::size_t idx = vnIndexEdgeMono[i];
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if ((chi2 > chi2Mono[it] && !bClose) ||
                (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBadMono++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const std::size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBadStereo++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        nInliers = nInliersMono + nInliersStereo;
        nBad = nBadMono + nBadStereo;

        if (optimizer.edges().size() < 10) {
            break;
        }
    }

    if ((nInliers < 30) && !bRecInit) {
        nBad = 0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        for (std::size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
            const std::size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2() < chi2MonoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
        for (std::size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend;
             i++) {
            const std::size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2() < chi2StereoOut)
                pFrame->mvbOutlier[idx] = false;
            else
                nBad++;
        }
    }

    nInliers = nInliersMono + nInliersStereo;

    // Recover optimized pose, velocity and biases
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                               VP->estimate().twb.cast<float>(),
                               VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // Recover Hessian, marginalize previous frame states and generate new prior
    // for frame
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    H.block<24, 24>(0, 0) += ei->GetHessian();

    Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    H.block<15, 15>(0, 0) += ep->GetHessian();

    int tot_in = 0, tot_out = 0;
    for (std::size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const std::size_t idx = vnIndexEdgeMono[i];

        if (!pFrame->mvbOutlier[idx]) {
            H.block<6, 6>(15, 15) += e->GetHessian();
            tot_in++;
        } else
            tot_out++;
    }

    for (std::size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const std::size_t idx = vnIndexEdgeStereo[i];

        if (!pFrame->mvbOutlier[idx]) {
            H.block<6, 6>(15, 15) += e->GetHessian();
            tot_in++;
        } else
            tot_out++;
    }

    H = Marginalize(H, 0, 14);

    pFrame->mpcpi = new ConstraintPoseImu(
        VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
        VA->estimate(), H.block<15, 15>(15, 15));
    delete pFp->mpcpi;
    pFp->mpcpi = nullptr;

    return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(
    Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    const LoopCloser::KeyFrameAndPose& NonCorrectedSim3,
    const LoopCloser::KeyFrameAndPose& CorrectedSim3,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    auto* solver =
        CreateOptimizationAlgorithm<g2o::BlockSolverX, g2o::LinearSolverEigen,
                                    g2o::OptimizationAlgorithmLevenberg>();

    optimizer.setAlgorithm(solver);

    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid +
                                                                     1);
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        nMaxKFid + 1);

    std::vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;
    // Set KeyFrame vertices
    for (std::size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
            continue;

        VertexPose4DoF* V4DoF;

        const int nIDi = pKF->mnId;

        LoopCloser::KeyFrameAndPose::const_iterator it =
            CorrectedSim3.find(pKF);

        if (it != CorrectedSim3.end()) {
            vScw[nIDi] = it->second;
            const g2o::Sim3 Swc = it->second.inverse();
            Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
            Eigen::Vector3d twc = Swc.translation();
            V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
        } else {
            autonomy::common::math::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vScw[nIDi] = Siw;
            V4DoF = new VertexPose4DoF(pKF);
        }

        if (pKF == pLoopKF)
            V4DoF->setFixed(true);

        V4DoF->setId(nIDi);
        V4DoF->setMarginalized(false);

        optimizer.addVertex(V4DoF);
        vpVertices[nIDi] = V4DoF;
    }
    std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

    // Edge used in posegraph has still 6Dof, even if updates of camera poses
    // are just in 4DoF
    Eigen::Matrix<double, 6, 6> matLambda =
        Eigen::Matrix<double, 6, 6>::Identity();
    matLambda(0, 0) = 1e3;
    matLambda(1, 1) = 1e3;
    matLambda(0, 0) = 1e3;

    // Set Loop edges
    Edge4DoF* e_loop;
    for (std::map<KeyFrame*, std::set<KeyFrame*>>::const_iterator
             mit = LoopConnections.begin(),
             mend = LoopConnections.end();
         mit != mend; mit++) {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const std::set<KeyFrame*>& spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];

        for (std::set<KeyFrame*>::const_iterator sit = spConnections.begin(),
                                                 send = spConnections.end();
             sit != send; sit++) {
            const long unsigned int nIDj = (*sit)->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
                pKF->GetWeight(*sit) < minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sij = Siw * Sjw.inverse();
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3) = 1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDi)));

            e->information() = matLambda;
            e_loop = e;
            optimizer.addEdge(e);

            sInsertedEdges.insert(
                std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
        }
    }

    // 1. Set normal edges
    for (std::size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Siw;

        // Use noncorrected poses for posegraph edges
        LoopCloser::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3.find(pKF);

        if (iti != NonCorrectedSim3.end())
            Siw = iti->second;
        else
            Siw = vScw[nIDi];

        // 1.1.0 Spanning tree edge
        KeyFrame* pParentKF = static_cast<KeyFrame*>(nullptr);
        if (pParentKF) {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Swj;

            LoopCloser::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3.find(pParentKF);

            if (itj != NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj = vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3) = 1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.1.1 Inertial edges
        KeyFrame* prevKF = pKF->mPrevKF;
        if (prevKF) {
            int nIDj = prevKF->mnId;

            g2o::Sim3 Swj;

            LoopCloser::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3.find(prevKF);

            if (itj != NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj = vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3) = 1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.2 Loop edges
        const std::set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for (std::set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(),
                                                 send = sLoopEdges.end();
             sit != send; sit++) {
            KeyFrame* pLKF = *sit;
            if (pLKF->mnId < pKF->mnId) {
                g2o::Sim3 Swl;

                LoopCloser::KeyFrameAndPose::const_iterator itl =
                    NonCorrectedSim3.find(pLKF);

                if (itl != NonCorrectedSim3.end())
                    Swl = itl->second.inverse();
                else
                    Swl = vScw[pLKF->mnId].inverse();

                g2o::Sim3 Sil = Siw * Swl;
                Eigen::Matrix4d Til;
                Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
                Til.block<3, 1>(0, 3) = Sil.translation();
                Til(3, 3) = 1.;

                Edge4DoF* e = new Edge4DoF(Til);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(nIDi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                    optimizer.vertex(pLKF->mnId)));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }
        }

        // 1.3 Covisibility graph edges
        const std::vector<KeyFrame*> vpConnectedKFs =
            pKF->GetCovisiblesByWeight(minFeat);
        for (std::vector<KeyFrame*>::const_iterator vit =
                 vpConnectedKFs.begin();
             vit != vpConnectedKFs.end(); vit++) {
            KeyFrame* pKFn = *vit;
            if (pKFn && pKFn != pParentKF && pKFn != prevKF &&
                pKFn != pKF->mNextKF && !pKF->hasChild(pKFn) &&
                !sLoopEdges.count(pKFn)) {
                if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
                    if (sInsertedEdges.count(
                            std::make_pair(std::min(pKF->mnId, pKFn->mnId),
                                           std::max(pKF->mnId, pKFn->mnId))))
                        continue;

                    g2o::Sim3 Swn;

                    LoopCloser::KeyFrameAndPose::const_iterator itn =
                        NonCorrectedSim3.find(pKFn);

                    if (itn != NonCorrectedSim3.end())
                        Swn = itn->second.inverse();
                    else
                        Swn = vScw[pKFn->mnId].inverse();

                    g2o::Sim3 Sin = Siw * Swn;
                    Eigen::Matrix4d Tin;
                    Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
                    Tin.block<3, 1>(0, 3) = Sin.translation();
                    Tin(3, 3) = 1.;
                    Edge4DoF* e = new Edge4DoF(Tin);
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(nIDi)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFn->mnId)));
                    e->information() = matLambda;
                    optimizer.addEdge(e);
                }
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (std::size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        VertexPose4DoF* Vi =
            static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
        Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
        Eigen::Vector3d ti = Vi->estimate().tcw[0];

        g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

        autonomy::common::math::SE3d Tiw(CorrectedSiw.rotation(),
                                         CorrectedSiw.translation());
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (std::size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        MapPoint* pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        int nIDr;

        KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
}

}  // namespace autonomy::localization::atlas
