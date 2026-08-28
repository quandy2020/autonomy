#include "autonomy/localization/atlas/optimize/line_geometry_util.hpp"

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"

namespace autonomy::localization::atlas::optimize {

Mat33_t line_skew(const Vec3_t& t) {
    Mat33_t S;
    S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
    return S;
}

Vec6_t transform_pluecker_with_sim3(const Vec6_t& pluecker,
                                    const Mat33_t& rot_cw,
                                    const Vec3_t& trans_cw,
                                    const double scale_cw) {
    Mat66_t sim3_cw_pluecker = Mat66_t::Zero();
    sim3_cw_pluecker.block<3, 3>(0, 0) = scale_cw * rot_cw;
    sim3_cw_pluecker.block<3, 3>(3, 3) = rot_cw;
    sim3_cw_pluecker.block<3, 3>(0, 3) = line_skew(trans_cw) * rot_cw;
    return sim3_cw_pluecker * pluecker;
}

bool line_endpoint_trimming(const std::shared_ptr<data::landmark_line>& local_lm_line,
                            const Vec6_t& pluecker_coord,
                            Vec6_t& updated_pose_w) {
    const auto ref_kf = local_lm_line->get_ref_keyframe();
    if (!ref_kf) {
        return false;
    }
    const int idx = local_lm_line->get_index_in_keyframe(ref_kf);
    if (idx < 0 || static_cast<size_t>(idx) >= ref_kf->line_obs_.keylines.size()) {
        return false;
    }

    const auto& keyline = ref_kf->line_obs_.keylines.at(static_cast<size_t>(idx));
    const cv::Point2f sp = keyline.getStartPoint();
    const cv::Point2f ep = keyline.getEndPoint();

    const auto* camera = static_cast<const camera::perspective*>(ref_kf->camera_);
    const Mat44_t cam_pose_wc = ref_kf->get_pose_wc();
    const Mat33_t rot_cw = cam_pose_wc.block<3, 3>(0, 0);
    const Vec3_t trans_cw = cam_pose_wc.block<3, 1>(0, 3);

    Mat66_t transformation_line_cw = Mat66_t::Zero();
    transformation_line_cw.block<3, 3>(0, 0) = rot_cw;
    transformation_line_cw.block<3, 3>(3, 3) = rot_cw;
    transformation_line_cw.block<3, 3>(0, 3) = line_skew(trans_cw) * rot_cw;

    Mat33_t K;
    K << camera->fy_, 0.0, 0.0,
        0.0, camera->fx_, 0.0,
        -camera->fy_ * camera->cx_, -camera->fx_ * camera->cy_, camera->fx_ * camera->fy_;

    const Vec3_t reproj_line_function = K * (transformation_line_cw * pluecker_coord).block<3, 1>(0, 0);
    const double l1 = reproj_line_function(0);
    const double l2 = reproj_line_function(1);
    const double l3 = reproj_line_function(2);
    if (std::abs(l1) < 1e-12 || std::abs(l2) < 1e-12) {
        return false;
    }

    const double x_sp_closet = -(sp.y - (l2 / l1) * sp.x + (l3 / l2)) * ((l1 * l2) / (l1 * l1 + l2 * l2));
    const double y_sp_closet = -(l1 / l2) * x_sp_closet - (l3 / l2);
    const double x_ep_closet = -(ep.y - (l2 / l1) * ep.x + (l3 / l2)) * ((l1 * l2) / (l1 * l1 + l2 * l2));
    const double y_ep_closet = -(l1 / l2) * x_ep_closet - (l3 / l2);

    const double x_0sp = 0;
    const double y_0sp = sp.y - (l2 / l1) * sp.x;
    const double x_0ep = 0;
    const double y_0ep = ep.y - (l2 / l1) * ep.x;

    Mat34_t rotation_translation_combined = Mat34_t::Zero();
    rotation_translation_combined.block<3, 3>(0, 0) = rot_cw;
    rotation_translation_combined.block<3, 1>(0, 3) = trans_cw;
    const Mat34_t P = camera->eigen_cam_matrix_ * rotation_translation_combined;

    const Vec3_t point2d_sp_closet{x_sp_closet, y_sp_closet, 1.0};
    const Vec3_t point2d_0sp{x_0sp, y_0sp, 1.0};
    const Vec4_t plane3d_temp_sp = P.transpose() * point2d_sp_closet.cross(point2d_0sp);

    const Vec3_t point2d_ep_closet{x_ep_closet, y_ep_closet, 1.0};
    const Vec3_t point2d_0ep{x_0ep, y_0ep, 1.0};
    const Vec4_t plane3d_temp_ep = P.transpose() * point2d_ep_closet.cross(point2d_0ep);

    Mat44_t line3d_pluecker_matrix = Mat44_t::Zero();
    const Vec3_t m = pluecker_coord.head<3>();
    const Vec3_t d = pluecker_coord.tail<3>();
    line3d_pluecker_matrix.block<3, 3>(0, 0) = line_skew(m);
    line3d_pluecker_matrix.block<3, 1>(0, 3) = d;
    line3d_pluecker_matrix.block<1, 3>(3, 0) = -d.transpose();

    const Vec4_t intersect_endpoint_sp = line3d_pluecker_matrix * plane3d_temp_sp;
    const Vec4_t intersect_endpoint_ep = line3d_pluecker_matrix * plane3d_temp_ep;

    const Vec4_t pos_w_sp(intersect_endpoint_sp(0) / intersect_endpoint_sp(3),
                          intersect_endpoint_sp(1) / intersect_endpoint_sp(3),
                          intersect_endpoint_sp(2) / intersect_endpoint_sp(3), 1.0);
    const Vec4_t pos_w_ep(intersect_endpoint_ep(0) / intersect_endpoint_ep(3),
                          intersect_endpoint_ep(1) / intersect_endpoint_ep(3),
                          intersect_endpoint_ep(2) / intersect_endpoint_ep(3), 1.0);

    const Vec3_t pos_c_sp = rotation_translation_combined * pos_w_sp;
    const Vec3_t pos_c_ep = rotation_translation_combined * pos_w_ep;
    if (pos_c_sp(2) <= 0 || pos_c_ep(2) <= 0) {
        return false;
    }

    updated_pose_w << pos_w_sp(0), pos_w_sp(1), pos_w_sp(2), pos_w_ep(0), pos_w_ep(1), pos_w_ep(2);
    return true;
}

}  // namespace autonomy::localization::atlas::optimize
