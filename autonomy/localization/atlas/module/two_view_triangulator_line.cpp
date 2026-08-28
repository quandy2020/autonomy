/**
 * This file is part of Structure PLP-SLAM.
 *
 * Copyright 2022 DFKI (German Research Center for Artificial Intelligence)
 * Developed by Fangwen Shu <Fangwen.Shu@dfki.de>
 *
 * If you use this code, please cite the respective publications as
 * listed on the github repository.
 *
 * Structure PLP-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Structure PLP-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Structure PLP-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/module/two_view_triangulator_line.hpp"

namespace autonomy::localization::atlas
{
    namespace module
    {
        namespace
        {
            float median_line_depth(const data::keyframe& keyfrm, const unsigned int idx) {
                if (idx >= keyfrm.line_obs_.depths_start.size() ||
                    idx >= keyfrm.line_obs_.depths_end.size()) {
                    return -1.f;
                }
                const float depth_sp = keyfrm.line_obs_.depths_start.at(idx);
                const float depth_ep = keyfrm.line_obs_.depths_end.at(idx);
                if (depth_sp <= 0.f || depth_ep <= 0.f) {
                    return -1.f;
                }
                return 0.5f * (depth_sp + depth_ep);
            }

            bool try_rgbd_line_triangulation(const std::shared_ptr<data::keyframe>& keyfrm,
                                             const unsigned int idx,
                                             Vec4_t& sp_3D,
                                             Vec4_t& ep_3D) {
                if (median_line_depth(*keyfrm, idx) <= 0.f) {
                    return false;
                }
                const Vec6_t pos_w_line = keyfrm->triangulate_stereo_for_line(idx);
                if (pos_w_line.isZero()) {
                    return false;
                }
                sp_3D.block<3, 1>(0, 0) = pos_w_line.head<3>();
                sp_3D(3) = 1.0;
                ep_3D.block<3, 1>(0, 0) = pos_w_line.tail<3>();
                ep_3D(3) = 1.0;
                return true;
            }
        }  // namespace

        two_view_triangulator_line::two_view_triangulator_line(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                                                               const float rays_parallax_deg_thr)
            : keyfrm_1_(keyfrm_1), keyfrm_2_(keyfrm_2),
              rot_1w_(keyfrm_1->get_rot_cw()), rot_w1_(rot_1w_.transpose()), trans_1w_(keyfrm_1->get_trans_cw()),
              cam_pose_1w_(keyfrm_1->get_pose_cw()), cam_center_1_(keyfrm_1->get_trans_wc()), camera_1_(keyfrm_1->camera_),
              rot_2w_(keyfrm_2->get_rot_cw()), rot_w2_(rot_2w_.transpose()), trans_2w_(keyfrm_2->get_trans_cw()),
              cam_pose_2w_(keyfrm_2->get_pose_cw()), cam_center_2_(keyfrm_2->get_trans_wc()), camera_2_(keyfrm_2->camera_),
              ratio_factor_(2.0f * std::max(keyfrm_1->orb_params_->scale_factor_, keyfrm_2->orb_params_->scale_factor_)),
              cos_rays_parallax_thr_(std::cos(rays_parallax_deg_thr * M_PI / 180.0))
        {
            camera_persp_ = static_cast<camera::perspective *>(keyfrm_1->camera_);

            K_ << camera_persp_->fy_, 0.0, 0.0,
                0.0, camera_persp_->fx_, 0.0,
                -camera_persp_->fy_ * camera_persp_->cx_, -camera_persp_->fx_ * camera_persp_->cy_, camera_persp_->fx_ * camera_persp_->fy_;

            camera_type_ = keyfrm_1->camera_->setup_type_;
        }

        bool two_view_triangulator_line::triangulate(const unsigned idx_1, const unsigned int idx_2, Vec6_t &pos_w_line) const
        {
            // get 2D line segments and function (parameters)
            cv::line_descriptor::KeyLine keyline1 = keyfrm_1_->line_obs_.keylines[idx_1];
            cv::line_descriptor::KeyLine keyline2 = keyfrm_2_->line_obs_.keylines[idx_2];
            Vec3_t keyline1_function = keyfrm_1_->line_obs_.line_functions[idx_1];
            Vec3_t keyline2_function = keyfrm_2_->line_obs_.line_functions[idx_2];

            const float keyline_1_right = keyfrm_1_->line_obs_.stereo_x_right[idx_1];
            const bool is_stereo_1 =
                (0 <= keyline_1_right) ||
                (idx_1 < keyfrm_1_->line_obs_.depths_start.size() &&
                 keyfrm_1_->line_obs_.depths_start[idx_1] > 0.0f &&
                 keyfrm_1_->line_obs_.depths_end[idx_1] > 0.0f);

            const float keyline_2_right = keyfrm_2_->line_obs_.stereo_x_right[idx_2];
            const bool is_stereo_2 =
                (0 <= keyline_2_right) ||
                (idx_2 < keyfrm_2_->line_obs_.depths_start.size() &&
                 keyfrm_2_->line_obs_.depths_start[idx_2] > 0.0f &&
                 keyfrm_2_->line_obs_.depths_end[idx_2] > 0.0f);

            // convert keyline's middle point to bearings and compute the parallax between keyframe_1 and keyframe_2
            // bearing in keyframe_1
            const double x_normalized_1 = (keyline1.pt.x - camera_persp_->cx_) / camera_persp_->fx_;
            const double y_normalized_1 = (keyline1.pt.y - camera_persp_->cy_) / camera_persp_->fy_;
            const auto l2_norm_1 = std::sqrt(x_normalized_1 * x_normalized_1 + y_normalized_1 * y_normalized_1 + 1.0);
            Vec3_t ray_c_1 = Vec3_t{x_normalized_1 / l2_norm_1,
                                    y_normalized_1 / l2_norm_1,
                                    1.0 / l2_norm_1};

            // bearing in keyframe_2
            const double x_normalized_2 = (keyline2.pt.x - camera_persp_->cx_) / camera_persp_->fx_;
            const double y_normalized_2 = (keyline2.pt.y - camera_persp_->cy_) / camera_persp_->fy_;
            const auto l2_norm_2 = std::sqrt(x_normalized_2 * x_normalized_2 + y_normalized_2 * y_normalized_2 + 1.0);
            Vec3_t ray_c_2 = Vec3_t{x_normalized_2 / l2_norm_2,
                                    y_normalized_2 / l2_norm_2,
                                    1.0 / l2_norm_2};

            // rays with the world reference
            const Vec3_t ray_w_1 = rot_w1_ * ray_c_1;
            const Vec3_t ray_w_2 = rot_w2_ * ray_c_2;
            const auto cos_rays_parallax = ray_w_1.dot(ray_w_2);

            // compute the stereo parallax if the keypoint is observed as stereo
            const auto line_depth_1 = median_line_depth(*keyfrm_1_, idx_1);
            const auto line_depth_2 = median_line_depth(*keyfrm_2_, idx_2);
            const auto cos_stereo_parallax_1 = is_stereo_1
                                                   ? std::cos(2.0 * atan2(camera_1_->true_baseline_ / 2.0,
                                                                          camera_type_ == camera::setup_type_t::RGBD &&
                                                                                  line_depth_1 > 0.f
                                                                              ? line_depth_1
                                                                              : keyfrm_1_->frm_obs_.depths_.at(idx_1)))
                                                   : 2.0;
            const auto cos_stereo_parallax_2 = is_stereo_2
                                                   ? std::cos(2.0 * atan2(camera_2_->true_baseline_ / 2.0,
                                                                          camera_type_ == camera::setup_type_t::RGBD &&
                                                                                  line_depth_2 > 0.f
                                                                              ? line_depth_2
                                                                              : keyfrm_2_->frm_obs_.depths_.at(idx_2)))
                                                   : 2.0;
            const auto cos_stereo_parallax = std::min(cos_stereo_parallax_1, cos_stereo_parallax_2);

            // select to use "linear triangulation" or "stereo triangulation/depth triangulation"
            // threshold of minimum angle of the two rays
            const bool triangulate_with_twocams =
                // check if the sufficient parallax is provided
                ((!is_stereo_1 && !is_stereo_2) && 0.0 < cos_rays_parallax && cos_rays_parallax < cos_rays_parallax_thr_)
                // check if the parallax between the two cameras is larger than the stereo parallax
                || ((is_stereo_1 || is_stereo_2) && 0.0 < cos_rays_parallax && cos_rays_parallax < cos_stereo_parallax);

            // recover 3D position of starting/ending point
            Vec4_t sp_3D, ep_3D;
            bool got_endpoints = false;
            if (camera_type_ == camera::setup_type_t::RGBD) {
                got_endpoints = try_rgbd_line_triangulation(keyfrm_1_, idx_1, sp_3D, ep_3D) ||
                                try_rgbd_line_triangulation(keyfrm_2_, idx_2, sp_3D, ep_3D);
            }

            if (!got_endpoints && triangulate_with_twocams)
            { // FW: Triangulation Method 2: find an infinite 3D line via intersection of two 3D planes, while endpoints estimated by endpoints trimming
                // references of endpoints trimming:
                // "Elaborate Monocular Point and Line SLAM with Robust Initialization", ICCV'19
                // "Building a 3-D line-based map using stereo SLAM", IEEE Transactions on Robotics'15
                //! Notice: endpoints trimming is also used in BA for re-estimating the endpoints

                // the projection matrix of two keyframes
                Mat34_t Tcw1 = keyfrm_1_->get_pose_cw().block<3, 4>(0, 0);
                Mat34_t Tcw2 = keyfrm_2_->get_pose_cw().block<3, 4>(0, 0);
                Mat34_t P1 = camera_persp_->eigen_cam_matrix_ * Tcw1;
                Mat34_t P2 = camera_persp_->eigen_cam_matrix_ * Tcw2;

                // construct two planes
                Vec3_t xs_1{keyline1.getStartPoint().x, keyline1.getStartPoint().y, 1.0};
                Vec3_t xe_1{keyline1.getEndPoint().x, keyline1.getEndPoint().y, 1.0};
                Vec3_t line_1 = xs_1.cross(xe_1);
                Vec4_t plane_1 = line_1.transpose() * P1;

                Vec3_t xs_2{keyline2.getStartPoint().x, keyline2.getStartPoint().y, 1.0};
                Vec3_t xe_2{keyline2.getEndPoint().x, keyline2.getEndPoint().y, 1.0};
                Vec3_t line_2 = xs_2.cross(xe_2);
                Vec4_t plane_2 = line_2.transpose() * P2;

                // calculate dual Pluecker matrix via two plane intersection
                Mat44_t L_star = plane_1 * plane_2.transpose() - plane_2 * plane_1.transpose();

                // extract Pluecker coordinates of the 3D line (infinite line representation)
                Mat33_t d_skew = L_star.block<3, 3>(0, 0);
                Vec3_t d;
                d << d_skew(2, 1), d_skew(0, 2), d_skew(1, 0); // the direction vector of the line
                Vec3_t m = L_star.block<3, 1>(0, 3);           // the moment vector of the line

                Vec6_t plucker_coord;
                plucker_coord << m(0), m(1), m(2), d(0), d(1), d(2);

                // endpoints trimming (using keyframe 1)
                Mat66_t transformation_line_cw = Eigen::Matrix<double, 6, 6>::Zero();
                transformation_line_cw.block<3, 3>(0, 0) = rot_1w_;
                transformation_line_cw.block<3, 3>(3, 3) = rot_1w_;
                transformation_line_cw.block<3, 3>(0, 3) = skew(trans_1w_) * rot_1w_;

                Vec3_t reproj_line_function;
                reproj_line_function = K_ * (transformation_line_cw * plucker_coord).block<3, 1>(0, 0);

                double l1 = reproj_line_function(0);
                double l2 = reproj_line_function(1);
                double l3 = reproj_line_function(2);

                // calculate closet point on the re-projected line
                auto sp = keyline1.getStartPoint();
                auto ep = keyline1.getEndPoint();
                double x_sp_closet = -(sp.y - (l2 / l1) * sp.x + (l3 / l2)) * ((l1 * l2) / (l1 * l1 + l2 * l2));
                double y_sp_closet = -(l1 / l2) * x_sp_closet - (l3 / l2);

                double x_ep_closet = -(ep.y - (l2 / l1) * ep.x + (l3 / l2)) * ((l1 * l2) / (l1 * l1 + l2 * l2));
                double y_ep_closet = -(l1 / l2) * x_ep_closet - (l3 / l2);

                // calculate another point
                double x_0sp = 0;
                double y_0sp = sp.y - (l2 / l1) * sp.x;

                double x_0ep = 0;
                double y_0ep = ep.y - (l2 / l1) * ep.x;

                // calculate 3D plane (using keyframe 1)
                Vec3_t point2d_sp_closet{x_sp_closet, y_sp_closet, 1.0};
                Vec3_t point2d_0sp{x_0sp, y_0sp, 1.0};
                Vec3_t line_temp_sp = point2d_sp_closet.cross(point2d_0sp);
                Vec4_t plane3d_temp_sp = P1.transpose() * line_temp_sp;

                Vec3_t point2d_ep_closet{x_ep_closet, y_ep_closet, 1.0};
                Vec3_t point2d_0ep{x_0ep, y_0ep, 1.0};
                Vec3_t line_temp_ep = point2d_ep_closet.cross(point2d_0ep);
                Vec4_t plane3d_temp_ep = P1.transpose() * line_temp_ep;

                // calculate intersection of the 3D plane and 3d line
                Mat44_t line3d_pluecker_matrix = Eigen::Matrix<double, 4, 4>::Zero();
                line3d_pluecker_matrix.block<3, 3>(0, 0) = skew(m);
                line3d_pluecker_matrix.block<3, 1>(0, 3) = d;
                line3d_pluecker_matrix.block<1, 3>(3, 0) = -d.transpose();

                Vec4_t intersect_endpoint_sp, intersect_endpoint_ep;
                intersect_endpoint_sp = line3d_pluecker_matrix * plane3d_temp_sp;
                intersect_endpoint_ep = line3d_pluecker_matrix * plane3d_temp_ep;

                sp_3D << intersect_endpoint_sp(0) / intersect_endpoint_sp(3),
                    intersect_endpoint_sp(1) / intersect_endpoint_sp(3),
                    intersect_endpoint_sp(2) / intersect_endpoint_sp(3),
                    1.0;
                ep_3D << intersect_endpoint_ep(0) / intersect_endpoint_ep(3),
                    intersect_endpoint_ep(1) / intersect_endpoint_ep(3),
                    intersect_endpoint_ep(2) / intersect_endpoint_ep(3),
                    1.0;
                got_endpoints = true;
            }
            else if (!got_endpoints && is_stereo_1 && cos_stereo_parallax_1 < cos_stereo_parallax_2)
            {
                if (camera_type_ == camera::setup_type_t::RGBD)
                {
                    const Vec6_t pos_w_line = keyfrm_1_->triangulate_stereo_for_line(idx_1);
                    if (pos_w_line.isZero()) {
                        return false;
                    }
                    sp_3D.block<3, 1>(0, 0) = pos_w_line.head<3>();
                    sp_3D(3) = 1.0;
                    ep_3D.block<3, 1>(0, 0) = pos_w_line.tail<3>();
                    ep_3D(3) = 1.0;
                    got_endpoints = true;
                }
                else if (camera_type_ == camera::setup_type_t::Stereo)
                {
                    const Vec3_t pt_w = keyfrm_1_->triangulate_stereo(idx_1);
                    sp_3D.block<3, 1>(0, 0) = pt_w;
                    sp_3D(3) = 1.0;
                    ep_3D.block<3, 1>(0, 0) = pt_w;
                    ep_3D(3) = 1.0;
                    got_endpoints = true;
                }
            }
            else if (!got_endpoints && is_stereo_2 && cos_stereo_parallax_2 < cos_stereo_parallax_1)
            {
                if (camera_type_ == camera::setup_type_t::RGBD)
                {
                    const Vec6_t pos_w_line = keyfrm_2_->triangulate_stereo_for_line(idx_2);
                    if (pos_w_line.isZero()) {
                        return false;
                    }
                    sp_3D.block<3, 1>(0, 0) = pos_w_line.head<3>();
                    sp_3D(3) = 1.0;
                    ep_3D.block<3, 1>(0, 0) = pos_w_line.tail<3>();
                    ep_3D(3) = 1.0;
                    got_endpoints = true;
                }
                else if (camera_type_ == camera::setup_type_t::Stereo)
                {
                    const Vec3_t pt_w = keyfrm_2_->triangulate_stereo(idx_2);
                    sp_3D.block<3, 1>(0, 0) = pt_w;
                    sp_3D(3) = 1.0;
                    ep_3D.block<3, 1>(0, 0) = pt_w;
                    ep_3D(3) = 1.0;
                    got_endpoints = true;
                }
            }
            else if (!got_endpoints)
            {
                return false;
            }

            // check if the ending points are too close to the camera_center
            if ((sp_3D.block<3, 1>(0, 0) - cam_center_1_).norm() / keyfrm_2_->compute_median_depth(true) < 0.3 ||
                (ep_3D.block<3, 1>(0, 0) - cam_center_2_).norm() / keyfrm_2_->compute_median_depth(true) < 0.3)
            {
                return false;
            }

            // check if the 3D line is too lang
            if ((ep_3D.block<3, 1>(0, 0) - sp_3D.block<3, 1>(0, 0)).norm() / keyfrm_2_->compute_median_depth(true) > 0.9)
            {
                return false;
            }

            // check if positive depth in both keyframes
            if (!check_depth_is_positive(sp_3D.block<3, 1>(0, 0), rot_1w_, trans_1w_) ||
                !check_depth_is_positive(sp_3D.block<3, 1>(0, 0), rot_2w_, trans_2w_) ||
                !check_depth_is_positive(ep_3D.block<3, 1>(0, 0), rot_1w_, trans_1w_) ||
                !check_depth_is_positive(ep_3D.block<3, 1>(0, 0), rot_2w_, trans_2w_))
            {
                return false;
            }

            // check reprojection error (midpoint, starting point, and ending point) in both keyframes
            Vec3_t midpoint = 0.5 * (sp_3D.block<3, 1>(0, 0) + ep_3D.block<3, 1>(0, 0));
            if (!check_reprojection_error(midpoint, rot_1w_, trans_1w_, keyline1, keyfrm_1_->inv_level_sigma_sq_lsd_.at(keyline1.octave)) ||
                !check_reprojection_error(midpoint, rot_2w_, trans_2w_, keyline2, keyfrm_2_->inv_level_sigma_sq_lsd_.at(keyline2.octave)) ||
                !check_reprojection_error(sp_3D.block<3, 1>(0, 0), rot_1w_, trans_1w_, keyline1_function, keyline_1_right,
                                          keyfrm_1_->inv_level_sigma_sq_lsd_.at(keyline1.octave), is_stereo_1) ||
                !check_reprojection_error(ep_3D.block<3, 1>(0, 0), rot_1w_, trans_1w_, keyline1_function, keyline_1_right,
                                          keyfrm_1_->inv_level_sigma_sq_lsd_.at(keyline1.octave), is_stereo_1) ||
                !check_reprojection_error(sp_3D.block<3, 1>(0, 0), rot_2w_, trans_2w_, keyline2_function, keyline_2_right,
                                          keyfrm_2_->inv_level_sigma_sq_lsd_.at(keyline2.octave), is_stereo_2) ||
                !check_reprojection_error(ep_3D.block<3, 1>(0, 0), rot_2w_, trans_2w_, keyline2_function, keyline_2_right,
                                          keyfrm_2_->inv_level_sigma_sq_lsd_.at(keyline2.octave), is_stereo_2))
            {
                return false;
            }

            // reject the line if the real scale factor and the predicted one are much different
            if (!check_scale_factors(sp_3D.block<3, 1>(0, 0),
                                     keyfrm_1_->scale_factors_lsd_.at(keyline1.octave),
                                     keyfrm_2_->scale_factors_lsd_.at(keyline2.octave)) ||
                !check_scale_factors(ep_3D.block<3, 1>(0, 0),
                                     keyfrm_1_->scale_factors_lsd_.at(keyline1.octave),
                                     keyfrm_2_->scale_factors_lsd_.at(keyline2.octave)))
            {
                return false;
            }

            pos_w_line << sp_3D(0), sp_3D(1), sp_3D(2), ep_3D(0), ep_3D(1), ep_3D(2);
            return true;
        }

        // used to check the ending point of the line
        bool two_view_triangulator_line::check_reprojection_error(const Vec3_t &pos_w, const Mat33_t &rot_cw, const Vec3_t &trans_cw,
                                                                  const Vec3_t &keyline_function, const float x_right,
                                                                  const float sigma_sq, const bool is_stereo) const
        {
            // chi-squared values for p=5%
            // (n=2)
            constexpr float chi_sq_2D = 5.99146;
            // (n=3)
            // constexpr float chi_sq_3D = 7.81473;

            Vec2_t reproj_in_cur;
            float x_right_in_cur;
            camera_persp_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj_in_cur, x_right_in_cur);

            const float reproj_err = (keyline_function(0) * reproj_in_cur(0) + keyline_function(1) * reproj_in_cur(1) + keyline_function(2)) /
                                     sqrt((keyline_function(0) * keyline_function(0) + keyline_function(1) * keyline_function(1)));

            if (chi_sq_2D * sigma_sq < abs(reproj_err))
            {
                return false;
            }

            return true;
        }

        // used to check the middle point of the line
        bool two_view_triangulator_line::check_reprojection_error(const Vec3_t &pos_w_middlepoint, const Mat33_t &rot_cw, const Vec3_t &trans_cw,
                                                                  const cv::line_descriptor::KeyLine &keyline, const float sigma_sq) const
        {
            // chi-squared values for p=5%
            // (n=2)
            constexpr float chi_sq_2D = 5.99146;
            // (n=3)
            // constexpr float chi_sq_3D = 7.81473;

            Vec2_t reproj_in_cur;
            float x_right_in_cur;
            camera_persp_->reproject_to_image(rot_cw, trans_cw, pos_w_middlepoint, reproj_in_cur, x_right_in_cur);

            const Vec2_t reproj_err_curr = reproj_in_cur - keyline.pt;
            if (chi_sq_2D * sigma_sq < reproj_err_curr.squaredNorm())
            {
                return false;
            }

            return true;
        }

    } // namespace module
} // namespace autonomy::localization::atlas
