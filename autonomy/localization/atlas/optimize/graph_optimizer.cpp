#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/optimize/line_geometry_util.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/optimize/graph_optimizer.hpp"
#include "autonomy/localization/atlas/optimize/terminate_action.hpp"
#include "autonomy/localization/atlas/optimize/internal/sim3/shot_vertex.hpp"
#include "autonomy/localization/atlas/optimize/internal/sim3/graph_opt_edge.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

namespace autonomy::localization::atlas {
namespace optimize {

graph_optimizer::graph_optimizer(const YAML::Node& yaml_node, data::map_database* map_db, const bool fix_scale)
    : map_db_(map_db),
      fix_scale_(fix_scale),
      min_num_shared_lms_(yaml_node["min_num_shared_lms"].as<unsigned int>(100)) {}

void graph_optimizer::optimize(const std::shared_ptr<data::keyframe>& loop_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm,
                               const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                               const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                               const std::map<std::shared_ptr<data::keyframe>, std::set<std::shared_ptr<data::keyframe>>>& loop_connections,
                               std::unordered_map<unsigned int, unsigned int>& found_lm_to_ref_keyfrm_id) const {
    // 1. Construct an optimizer

    auto linear_solver = autonomy::localization::atlas::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
    auto block_solver = autonomy::localization::atlas::make_unique<g2o::BlockSolver_7_3>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    auto terminateAction = new terminate_action;
    terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);
    optimizer.setAlgorithm(algorithm);

    // 2. Add vertices

    const auto all_keyfrms = curr_keyfrm->graph_node_->get_keyframes_from_root();
    std::unordered_set<unsigned int> already_found_landmark_ids;
    std::vector<std::shared_ptr<data::landmark>> all_lms;
    for (const auto& keyfrm : all_keyfrms) {
        for (const auto& lm : keyfrm->get_landmarks()) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            if (already_found_landmark_ids.count(lm->id_)) {
                continue;
            }

            already_found_landmark_ids.insert(lm->id_);
            all_lms.push_back(lm);
        }
    }

    // Transform the pre-modified poses of all the keyframes to Sim3, and save them
    eigen_alloc_unord_map<unsigned int, g2o::Sim3> Sim3s_cw;
    // Save the added vertices
    std::unordered_map<unsigned int, internal::sim3::shot_vertex*> vertices;

    for (auto keyfrm : all_keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = new internal::sim3::shot_vertex();

        const auto id = keyfrm->id_;

        // BEFORE optimization, check if the poses have been already modified
        const auto iter = pre_corrected_Sim3s.find(keyfrm);
        if (iter != pre_corrected_Sim3s.end()) {
            // BEFORE optimization, set the already-modified poses for verices
            Sim3s_cw[id] = iter->second;
            keyfrm_vtx->setEstimate(iter->second);
        }
        else {
            // Transform an unmodified pose to Sim3, and set it for a vertex
            const Mat33_t rot_cw = keyfrm->get_rot_cw();
            const Vec3_t trans_cw = keyfrm->get_trans_cw();
            const g2o::Sim3 Sim3_cw(rot_cw, trans_cw, 1.0);

            Sim3s_cw[id] = Sim3_cw;
            keyfrm_vtx->setEstimate(Sim3_cw);
        }

        // Fix the loop keyframe or root keyframe
        if (*keyfrm == *loop_keyfrm || *keyfrm == *curr_keyfrm || keyfrm->graph_node_->is_spanning_root()) {
            keyfrm_vtx->setFixed(true);
        }

        // Set the vertex to the optimizer
        keyfrm_vtx->setId(id);
        keyfrm_vtx->fix_scale_ = fix_scale_;

        optimizer.addVertex(keyfrm_vtx);
        vertices[id] = keyfrm_vtx;
    }

    // 3. Add edges

    // Save keyframe pairs which the edge is inserted between
    std::set<std::pair<unsigned int, unsigned int>> inserted_edge_pairs;

    // Function to add a constraint edge
    const auto insert_edge =
        [&optimizer, &vertices, &inserted_edge_pairs](unsigned int id1, unsigned int id2, const g2o::Sim3& Sim3_21) {
            auto edge = new internal::sim3::graph_opt_edge();
            edge->setVertex(0, vertices.at(id1));
            edge->setVertex(1, vertices.at(id2));
            edge->setMeasurement(Sim3_21);

            edge->information() = MatRC_t<7, 7>::Identity();

            optimizer.addEdge(edge);
            inserted_edge_pairs.insert(std::make_pair(std::min(id1, id2), std::max(id1, id2)));
        };

    // Add loop edges only over the number of shared landmarks threshold
    for (const auto& loop_connection : loop_connections) {
        auto keyfrm = loop_connection.first;
        const auto& connected_keyfrms = loop_connection.second;

        const auto id1 = keyfrm->id_;
        const g2o::Sim3& Sim3_1w = Sim3s_cw.at(id1);
        const g2o::Sim3 Sim3_w1 = Sim3_1w.inverse();

        for (auto connected_keyfrm : connected_keyfrms) {
            const auto id2 = connected_keyfrm->id_;

            // Except the current vs loop edges,
            // Add the loop edges only over the minimum number of shared landmarks threshold
            if (!(id1 == curr_keyfrm->id_ && id2 == loop_keyfrm->id_)
                && keyfrm->graph_node_->get_num_shared_landmarks(connected_keyfrm) < min_num_shared_lms_) {
                continue;
            }

            // Compute the relative camera pose
            const g2o::Sim3& Sim3_2w = Sim3s_cw.at(id2);
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // Add a constraint edge
            insert_edge(id1, id2, Sim3_21);
        }
    }

    // Add non-loop-connected edges
    for (auto keyfrm : all_keyfrms) {
        // Select one pose of the keyframe pair
        const auto id1 = keyfrm->id_;

        // Use only non-modified poses in the covisibility information
        // (Both camera poses should be non-modified in order to compute the relative pose correctly)
        const auto iter1 = non_corrected_Sim3s.find(keyfrm);
        const g2o::Sim3 Sim3_w1 = ((iter1 != non_corrected_Sim3s.end()) ? iter1->second : Sim3s_cw.at(id1)).inverse();

        auto parent_node = keyfrm->graph_node_->get_spanning_parent();
        if (parent_node) {
            const auto id2 = parent_node->id_;

            // Use only non-modified poses in the covisibility information
            // (Both camera poses should be nop-modified in order to compute the relative pose correctly)
            const auto iter2 = non_corrected_Sim3s.find(parent_node);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // Compute the relative camera pose
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // Add a constraint edge
            insert_edge(id1, id2, Sim3_21);
        }

        // Add all the loop edges with any number of shared landmarks
        const auto loop_edges = keyfrm->graph_node_->get_loop_edges();
        for (auto connected_keyfrm : loop_edges) {
            const auto id2 = connected_keyfrm->id_;

            // Avoid duplication
            if (id1 <= id2) {
                continue;
            }

            // Use only non-modified poses in the covisibility information
            // (Both camera poses should be nop-modified in order to compute the relative pose correctly)
            const auto iter2 = non_corrected_Sim3s.find(connected_keyfrm);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // Compute the relative camera pose
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // Add a constraint edge
            insert_edge(id1, id2, Sim3_21);
        }

        // Add the covisibility information over the minimum number of shared landmarks threshold
        const auto connected_keyfrms = keyfrm->graph_node_->get_covisibilities_over_min_num_shared_lms(min_num_shared_lms_);
        for (auto connected_keyfrm : connected_keyfrms) {
            // null check
            if (!connected_keyfrm || !parent_node) {
                continue;
            }
            // Exclude parent-child edges because they've been already inserted
            if (*connected_keyfrm == *parent_node
                || keyfrm->graph_node_->has_spanning_child(connected_keyfrm)) {
                continue;
            }
            // Exclude any edges associated to the loop because they've been already inserted
            if (static_cast<bool>(loop_edges.count(connected_keyfrm))) {
                continue;
            }

            if (connected_keyfrm->will_be_erased()) {
                continue;
            }

            const auto id2 = connected_keyfrm->id_;

            // Avoid duplication
            if (id1 <= id2) {
                continue;
            }
            if (static_cast<bool>(inserted_edge_pairs.count(std::make_pair(std::min(id1, id2), std::max(id1, id2))))) {
                continue;
            }

            // Use only non-modified poses in the covisibility information
            // (Both camera poses should be nop-modified in order to compute the relative pose correctly)
            const auto iter2 = non_corrected_Sim3s.find(connected_keyfrm);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // Compute the relative camera pose
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // Add a constraint edge
            insert_edge(id1, id2, Sim3_21);
        }
    }

    // 4. Perform a pose graph optimization

    optimizer.initializeOptimization();
    optimizer.optimize(50);

    delete terminateAction;

    // 5. Update the camera poses and point-cloud

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // For modification of a point-cloud, save the post-modified poses of all the keyframes
        std::unordered_map<unsigned int, g2o::Sim3> corrected_Sim3s_wc;

        for (auto keyfrm : all_keyfrms) {
            const auto id = keyfrm->id_;

            auto keyfrm_vtx = static_cast<internal::sim3::shot_vertex*>(optimizer.vertex(id));

            const g2o::Sim3& corrected_Sim3_cw = keyfrm_vtx->estimate();
            const float s = corrected_Sim3_cw.scale();
            const Mat33_t rot_cw = corrected_Sim3_cw.rotation().toRotationMatrix();
            const Vec3_t trans_cw = corrected_Sim3_cw.translation() / s;

            const Mat44_t cam_pose_cw = util::converter::to_eigen_pose(rot_cw, trans_cw);
            keyfrm->set_pose_cw(cam_pose_cw);

            corrected_Sim3s_wc[id] = corrected_Sim3_cw.inverse();
        }

        // Update the point-cloud
        for (const auto& lm : all_lms) {
            if (lm->will_be_erased()) {
                continue;
            }

            const auto id = (found_lm_to_ref_keyfrm_id.count(lm->id_))
                                ? found_lm_to_ref_keyfrm_id.at(lm->id_)
                                : lm->get_ref_keyframe()->id_;

            const g2o::Sim3& Sim3_cw = Sim3s_cw.at(id);
            const g2o::Sim3& corrected_Sim3_wc = corrected_Sim3s_wc.at(id);

            const Vec3_t pos_w = lm->get_pos_in_world();
            const Vec3_t corrected_pos_w = corrected_Sim3_wc.map(Sim3_cw.map(pos_w));

            lm->set_pos_in_world(corrected_pos_w);
            lm->update_mean_normal_and_obs_scale_variance();
        }

        if (map_db_ && map_db_->use_line_tracking()) {
            std::unordered_set<unsigned int> already_found_line_ids;
            std::vector<std::shared_ptr<data::landmark_line>> all_lms_line;
            for (const auto& keyfrm : all_keyfrms) {
                for (const auto& lm_line : keyfrm->get_landmarks_line()) {
                    if (!lm_line || lm_line->will_be_erased() || already_found_line_ids.count(lm_line->id_)) {
                        continue;
                    }
                    already_found_line_ids.insert(lm_line->id_);
                    all_lms_line.push_back(lm_line);
                }
            }

            std::unordered_set<unsigned int> keyframe_ids;
            for (const auto& keyfrm : all_keyfrms) {
                keyframe_ids.insert(keyfrm->id_);
            }

            for (const auto& lm_line : all_lms_line) {
                if (lm_line->will_be_erased()) {
                    continue;
                }
                const auto ref_keyfrm = lm_line->get_ref_keyframe();
                if (!ref_keyfrm || !keyframe_ids.count(ref_keyfrm->id_)) {
                    lm_line->prepare_for_erasing(map_db_);
                    continue;
                }

                const auto id = (lm_line->loop_fusion_identifier_ == curr_keyfrm->id_)
                                    ? lm_line->ref_keyfrm_id_in_loop_fusion_
                                    : ref_keyfrm->id_;
                const g2o::Sim3& Sim3_cw = Sim3s_cw.at(id);
                const g2o::Sim3& corrected_Sim3_wc = corrected_Sim3s_wc.at(id);

                const double scale_cw = Sim3_cw.scale();
                const Mat33_t rot_cw = Sim3_cw.rotation().toRotationMatrix();
                const Vec3_t trans_cw = Sim3_cw.translation();
                const double scale_wc = corrected_Sim3_wc.scale();
                const Mat33_t rot_wc = corrected_Sim3_wc.rotation().toRotationMatrix();
                const Vec3_t trans_wc = corrected_Sim3_wc.translation();

                const Vec6_t pos_w = lm_line->get_pluecker_coord();
                const Vec6_t step1 = transform_pluecker_with_sim3(pos_w, rot_cw, trans_cw, scale_cw);
                const Vec6_t corrected_pos_w = transform_pluecker_with_sim3(step1, rot_wc, trans_wc, scale_wc);

                lm_line->set_pluecker_coord_without_update_endpoints(corrected_pos_w);
                Vec6_t updated_pose_w;
                if (line_endpoint_trimming(lm_line, corrected_pos_w, updated_pose_w)) {
                    lm_line->set_pos_in_world_without_update_pluecker(updated_pose_w);
                    lm_line->update_information();
                } else {
                    lm_line->prepare_for_erasing(map_db_);
                }
            }
        }
    }
}

} // namespace optimize
}  // namespace autonomy::localization::atlas
