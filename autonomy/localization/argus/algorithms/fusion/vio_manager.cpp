/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "argus/odometry/vio_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "argus/common/math/so3.hpp"
#include "argus/common/vision/vision_utils.hpp"
#include "argus/frontend/types.hpp"

namespace argus {
namespace odometry {

using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::VectorXd;
using frontend::pointWithVar;
using map::Feature;
using map::VisualPoint;
using map::VoxelLocation;
using map::VoxelPoints;
using map::VoxelOctoTree;
using map::VoxelPlane;
using math::SE3;

namespace {
inline double NowSec() {
#ifdef _OPENMP
    return omp_get_wtime();
#else
    return 0.0;
#endif
}
}  // namespace

#ifndef VEC_FROM_ARRAY
#define VEC_FROM_ARRAY(v) (v)[0], (v)[1], (v)[2]
#endif
#ifndef MAT_FROM_ARRAY
#define MAT_FROM_ARRAY(v) \
    (v)[0], (v)[1], (v)[2], (v)[3], (v)[4], (v)[5], (v)[6], (v)[7], (v)[8]
#endif

VIOManager::VIOManager() : feat_map(visual_map.feat_map()) {}

VIOManager::~VIOManager() {
    delete visual_submap;
    for (auto& pair : warp_map)
        delete pair.second;
    warp_map.clear();
    visual_map.Clear();
}

void VIOManager::SetImuToLidarExtrinsic(const Vector3d& transl,
                                        const Matrix3d& rot) {
    Pli = -rot.transpose() * transl;
    Rli = rot.transpose();
}

void VIOManager::SetLidarToCameraExtrinsic(std::vector<double>& R,
                                           std::vector<double>& P) {
    Rcl << MAT_FROM_ARRAY(R);
    Pcl << VEC_FROM_ARRAY(P);
}

void VIOManager::InitIntrinsicsAndPoseJacobians() {
    fx = cam->fx();
    fy = cam->fy();
    cx = cam->cx();
    cy = cam->cy();
    image_resize_factor = cam->scale();

    if (enable_plot) {
        printf("intrinsic: %.6lf, %.6lf, %.6lf, %.6lf\n", fx, fy, cx, cy);
    }

    width = cam->width();
    height = cam->height();

    if (enable_plot) {
        printf("width: %d, height: %d, scale: %f\n", width, height,
               image_resize_factor);
    }
    Rci = Rcl * Rli;
    Pci = Rcl * Pli + Pcl;

    Vector3d Pic;
    Matrix3d tmp;
    Jdphi_dR = Rci;
    Pic = -Rci.transpose() * Pci;
    tmp << SKEW_SYM_MATRX(Pic);
    Jdp_dR = -Rci * tmp;
}

void VIOManager::InitImageGrid() {
    // Must divide as double then ceil — `(width / grid_size)` is int truncation
    // and under-covers the image (FAST-LIVO2 upstream bug).
    if (grid_size > 10) {
        grid_n_width = static_cast<int>(
            std::ceil(static_cast<double>(width) / grid_size));
        grid_n_height = static_cast<int>(
            std::ceil(static_cast<double>(height) / grid_size));
    } else {
        if (grid_n_height <= 0) {
            grid_n_height = 1;
        }
        grid_size = std::max(1, height / grid_n_height);
        grid_n_height = static_cast<int>(
            std::ceil(static_cast<double>(height) / grid_size));
        grid_n_width = static_cast<int>(
            std::ceil(static_cast<double>(width) / grid_size));
    }
    grid_n_width = std::max(1, grid_n_width);
    grid_n_height = std::max(1, grid_n_height);
    length = grid_n_width * grid_n_height;
}

bool VIOManager::GridIndexFromPixel(const Vector2d& pc, int* index) const {
    if (index == nullptr || grid_size <= 0 || length <= 0) {
        return false;
    }
    const int gx = static_cast<int>(pc[0] / grid_size);
    const int gy = static_cast<int>(pc[1] / grid_size);
    if (gx < 0 || gy < 0 || gx >= grid_n_width || gy >= grid_n_height) {
        return false;
    }
    *index = gy * grid_n_width + gx;
    return *index >= 0 && *index < length;
}

void VIOManager::InitRaycastSamples() {
    if (!enable_raycast)
        return;

    // cv::Mat img_test = cv::Mat::zeros(height, width, CV_8UC1);
    // uchar* it = (uchar*)img_test.data;

    border_flag.resize(length, 0);

    rays_with_sample_points.clear();
    rays_with_sample_points.reserve(static_cast<std::size_t>(length));
    if (enable_plot) {
        printf("grid_size: %d, grid_n_height: %d, grid_n_width: %d, "
               "length: %d\n",
               grid_size, grid_n_height, grid_n_width, length);
    }

    float d_min = 0.1;
    float d_max = 3.0;
    float step = 0.2;
    for (int grid_row = 1; grid_row <= grid_n_height; grid_row++) {
        for (int grid_col = 1; grid_col <= grid_n_width; grid_col++) {
            std::vector<Vector3d> SamplePointsEachGrid;
            int index = (grid_row - 1) * grid_n_width + grid_col - 1;

            if (grid_row == 1 || grid_col == 1 || grid_row == grid_n_height ||
                grid_col == grid_n_width)
                border_flag[index] = 1;

            int u = grid_size / 2 + (grid_col - 1) * grid_size;
            int v = grid_size / 2 + (grid_row - 1) * grid_size;
            // it[ u + v * width ] = 255;
            for (float d_temp = d_min; d_temp <= d_max; d_temp += step) {
                Vector3d xyz;
                xyz = cam->cam2world(u, v);
                xyz *= d_temp / xyz[2];
                // xyz[0] = (u - cx) / fx * d_temp;
                // xyz[1] = (v - cy) / fy * d_temp;
                // xyz[2] = d_temp;
                SamplePointsEachGrid.push_back(xyz);
            }
            rays_with_sample_points.push_back(SamplePointsEachGrid);
        }
    }
    // printf("rays_with_sample_points: %d, RaysWithSamplePointsCapacity:
    // %d, rays_with_sample_points[0].capacity(): %d,
    // rays_with_sample_points[0]: %d\n", rays_with_sample_points.size(),
    // rays_with_sample_points.capacity(),
    // rays_with_sample_points[0].capacity(),
    // rays_with_sample_points[0].size()); for (const auto & it :
    // rays_with_sample_points[0]) std::cout << it.transpose() << std::endl;
    // cv::imshow("img_test", img_test);
    // cv::waitKey(1);
}

void VIOManager::InitColmapLogs() {
    if (!enable_colmap_output)
        return;

    pinhole_cam = dynamic_cast<argus::camera::PinholeCamera*>(cam);
    colmap_log_.open(
        (log_dir.empty() ? std::string()
                         : (log_dir + "/Colmap/sparse/0/images.txt")),
        std::ios::out);
    colmap_log_ << "# Image list with two lines of data per image:\n";
    colmap_log_
        << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    colmap_log_ << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    fout_camera.open(
        (log_dir.empty() ? std::string()
                         : (log_dir + "/Colmap/sparse/0/cameras.txt")),
        std::ios::out);
    fout_camera << "# Camera list with one line of data per camera:\n";
    fout_camera << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    fout_camera << "1 PINHOLE " << width << " " << height << " " << std::fixed
                << std::setprecision(6)  // 控制浮点数精度为10位
                << fx << " " << fy << " " << cx << " " << cy << std::endl;
    fout_camera.close();
}

void VIOManager::InitPatchBuffers() {
    grid_num.resize(static_cast<std::size_t>(length));
    map_index.resize(static_cast<std::size_t>(length));
    map_dist.resize(static_cast<std::size_t>(length));
    update_flag.resize(static_cast<std::size_t>(length));
    scan_value.resize(static_cast<std::size_t>(length));

    patch_size_total = patch_size * patch_size;
    patch_size_half = static_cast<int>(patch_size / 2);
    patch_buffer.resize(static_cast<std::size_t>(patch_size_total));
    warp_len = patch_size_total * patch_pyramid_level;
    border = (patch_size_half + 1) * (1 << patch_pyramid_level);

    // Size (not only reserve) so Score* can write before first ResetGrid.
    retrieve_voxel_points.assign(static_cast<std::size_t>(length), nullptr);
    append_voxel_points.resize(static_cast<std::size_t>(length));

    sub_feat_map.clear();
}

void VIOManager::InitializeVIO() {
    visual_submap = new SubSparseMap;
    InitIntrinsicsAndPoseJacobians();
    InitImageGrid();
    InitRaycastSamples();
    InitColmapLogs();
    InitPatchBuffers();
}

void VIOManager::ResetGrid() {
    fill(grid_num.begin(), grid_num.end(), TYPE_UNKNOWN);
    fill(map_index.begin(), map_index.end(), 0);
    fill(map_dist.begin(), map_dist.end(), 10000.0f);
    fill(update_flag.begin(), update_flag.end(), 0);
    fill(scan_value.begin(), scan_value.end(), 0.0f);

    retrieve_voxel_points.clear();
    retrieve_voxel_points.resize(length);

    append_voxel_points.clear();
    append_voxel_points.resize(length);

    total_points = 0;
}

// void VIOManager::resetRvizDisplay()
// {
// sub_map_ray.clear();
// sub_map_ray_fov.clear();
// visual_sub_map_cur.clear();
// visual_converged_point.clear();
// map_cur_frame.clear();
// sample_points.clear();
// }

void VIOManager::ComputeProjectionJacobian(Vector3d p,
                                           Eigen::Matrix<double, 2, 3>& J) {
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1. / p[2];
    const double z_inv_2 = z_inv * z_inv;
    J(0, 0) = fx * z_inv;
    J(0, 1) = 0.0;
    J(0, 2) = -fx * x * z_inv_2;
    J(1, 0) = 0.0;
    J(1, 1) = fy * z_inv;
    J(1, 2) = -fy * y * z_inv_2;
}

void VIOManager::GetImagePatch(cv::Mat img, Vector2d pc, float* patch_tmp,
                               int level) {
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int scale = (1 << level);
    const int u_ref_i = floorf(pc[0] / scale) * scale;
    const int v_ref_i = floorf(pc[1] / scale) * scale;
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    for (int x = 0; x < patch_size; x++) {
        uint8_t* img_ptr =
            (uint8_t*)img.data +
            (v_ref_i - patch_size_half * scale + x * scale) * width +
            (u_ref_i - patch_size_half * scale);
        for (int y = 0; y < patch_size; y++, img_ptr += scale) {
            patch_tmp[patch_size_total * level + x * patch_size + y] =
                w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
                w_ref_bl * img_ptr[scale * width] +
                w_ref_br * img_ptr[scale * width + scale];
        }
    }
}

void VIOManager::InsertPointIntoVoxelMap(VisualPoint* pt_new) {
    visual_map.InsertPoint(pt_new);
}

void VIOManager::GetWarpMatrixAffineHomography(
    const argus::camera::AbstractCamera& cam, const Vector2d& px_ref,
    const Vector3d& xyz_ref, const Vector3d& normal_ref, const SE3& T_cur_ref,
    const int level_ref, Matrix2d& A_cur_ref) {
    // create homography matrix
    const Vector3d t = T_cur_ref.inverse().translation();
    const Eigen::Matrix3d H_cur_ref =
        T_cur_ref.rotation_matrix() *
        (normal_ref.dot(xyz_ref) * Eigen::Matrix3d::Identity() -
         t * normal_ref.transpose());
    // Compute affine warp matrix A_ref_cur using homography projection
    const int kHalfPatchSize = 4;
    Vector3d f_du_ref(cam.cam2world(
        px_ref + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref)));
    Vector3d f_dv_ref(cam.cam2world(
        px_ref + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref)));
    //   f_du_ref = f_du_ref/f_du_ref[2];
    //   f_dv_ref = f_dv_ref/f_dv_ref[2];
    const Vector3d f_cur(H_cur_ref * xyz_ref);
    const Vector3d f_du_cur = H_cur_ref * f_du_ref;
    const Vector3d f_dv_cur = H_cur_ref * f_dv_ref;
    Vector2d px_cur(cam.world2cam(f_cur));
    Vector2d px_du_cur(cam.world2cam(f_du_cur));
    Vector2d px_dv_cur(cam.world2cam(f_dv_cur));
    A_cur_ref.col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
    A_cur_ref.col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
}

void VIOManager::GetWarpMatrixAffine(
    const argus::camera::AbstractCamera& cam, const Vector2d& px_ref,
    const Vector3d& f_ref, const double depth_ref, const SE3& T_cur_ref,
    const int level_ref, const int pyramid_level, const int halfpatch_size,
    Matrix2d& A_cur_ref) {
    // Compute affine warp matrix A_ref_cur
    const Vector3d xyz_ref(f_ref * depth_ref);
    Vector3d xyz_du_ref(
        cam.cam2world(px_ref + Vector2d(halfpatch_size, 0) * (1 << level_ref) *
                                   (1 << pyramid_level)));
    Vector3d xyz_dv_ref(
        cam.cam2world(px_ref + Vector2d(0, halfpatch_size) * (1 << level_ref) *
                                   (1 << pyramid_level)));
    xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
    xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
    const Vector2d px_cur(cam.world2cam(T_cur_ref * (xyz_ref)));
    const Vector2d px_du(cam.world2cam(T_cur_ref * (xyz_du_ref)));
    const Vector2d px_dv(cam.world2cam(T_cur_ref * (xyz_dv_ref)));
    A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
    A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
}

void VIOManager::WarpAffine(const Matrix2d& A_cur_ref, const cv::Mat& img_ref,
                            const Vector2d& px_ref, const int level_ref,
                            const int search_level, const int pyramid_level,
                            const int halfpatch_size, float* patch) {
    const int patch_size = halfpatch_size * 2;
    const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
    if (std::isnan(A_ref_cur(0, 0))) {
        return;
    }

    float* patch_ptr = patch;
    for (int y = 0; y < patch_size; ++y) {
        for (int x = 0; x < patch_size; ++x)  //, ++patch_ptr)
        {
            Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
            px_patch *= (1 << search_level);
            px_patch *= (1 << pyramid_level);
            const Vector2f px(A_ref_cur * px_patch + px_ref.cast<float>());
            if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 ||
                px[1] >= img_ref.rows - 1)
                patch_ptr[patch_size_total * pyramid_level + y * patch_size +
                          x] = 0;
            else
                patch_ptr[patch_size_total * pyramid_level + y * patch_size +
                          x] =
                    (float)vision::InterpolateMat8u(img_ref, px[0], px[1]);
        }
    }
}

int VIOManager::GetBestSearchLevel(const Matrix2d& A_cur_ref,
                                   const int max_level) {
    // Compute patch level in other image
    int search_level = 0;
    double D = A_cur_ref.determinant();
    while (D > 3.0 && search_level < max_level) {
        search_level += 1;
        D *= 0.25;
    }
    return search_level;
}

double VIOManager::CalculateNCC(float* ref_patch, float* cur_patch,
                                int patch_size) {
    double sum_ref = std::accumulate(ref_patch, ref_patch + patch_size, 0.0);
    double mean_ref = sum_ref / patch_size;

    double sum_cur = std::accumulate(cur_patch, cur_patch + patch_size, 0.0);
    double mean_curr = sum_cur / patch_size;

    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < patch_size; i++) {
        double n = (ref_patch[i] - mean_ref) * (cur_patch[i] - mean_curr);
        numerator += n;
        demoniator1 += (ref_patch[i] - mean_ref) * (ref_patch[i] - mean_ref);
        demoniator2 += (cur_patch[i] - mean_curr) * (cur_patch[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

void VIOManager::BuildDepthAndSubmap(
    const std::vector<pointWithVar>& pg, cv::Mat& depth_img,
    float voxel_size) {
    float* it = (float*)depth_img.data;
    int loc_xyz[3];

    // printf("pg size: %zu \n", pg.size());

    for (int i = 0; i < pg.size(); i++) {
        // double t0 = NowSec();

        Vector3d pt_w = pg[i].point_w;

        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = floor(pt_w[j] / voxel_size);
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

        // t_position += NowSec()-t0;
        // double t1 = NowSec();

        auto iter = sub_feat_map.find(position);
        if (iter == sub_feat_map.end()) {
            sub_feat_map[position] = 0;
        } else {
            iter->second = 0;
        }

        // t_insert += NowSec()-t1;
        // double t2 = NowSec();

        Vector3d pt_c(new_frame_->w2f(pt_w));

        if (pt_c[2] > 0) {
            Vector2d px;
            // px[0] = fx * pt_c[0]/pt_c[2] + cx;
            // px[1] = fy * pt_c[1]/pt_c[2]+ cy;
            px = new_frame_->cam_->world2cam(pt_c);

            if (new_frame_->cam_->isInFrame(px.cast<int>(), border)) {
                // cv::circle(img_cp, cv::Point2f(px[0], px[1]), 3,
                // cv::Scalar(0, 0, 255), -1, 8);
                float depth = pt_c[2];
                int col = int(px[0]);
                int row = int(px[1]);
                it[width * row + col] = depth;
            }
        }
        // t_depth += NowSec()-t2;
    }
}

void VIOManager::CollectVisibleSubmapPoints(
    std::vector<VoxelLocation>& delete_key_list) {
    for (auto& iter : sub_feat_map) {
        VoxelLocation position = iter.first;

        // double t4 = NowSec();
        auto corre_voxel = feat_map.find(position);
        // double t5 = NowSec();

        if (corre_voxel != feat_map.end()) {
            bool voxel_in_fov = false;
            std::vector<VisualPoint*>& voxel_points =
                corre_voxel->second->voxel_points;
            int voxel_num = voxel_points.size();

            for (int i = 0; i < voxel_num; i++) {
                VisualPoint* pt = voxel_points[i];
                if (pt == nullptr)
                    continue;
                if (pt->obs_.size() == 0)
                    continue;

                Vector3d norm_vec(new_frame_->T_f_w_.rotation_matrix() *
                                  pt->normal_);
                Vector3d dir(new_frame_->T_f_w_ * pt->pos_);
                if (dir[2] < 0)
                    continue;
                // dir.normalize();
                // if (dir.dot(norm_vec) <= 0.17) continue; // 0.34 70 degree
                // 0.17 80 degree 0.08 85 degree

                Vector2d pc(new_frame_->w2c(pt->pos_));
                int index = 0;
                if (new_frame_->cam_->isInFrame(pc.cast<int>(), border) &&
                    GridIndexFromPixel(pc, &index)) {
                    // cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 3,
                    // cv::Scalar(0, 255, 255), -1, 8);
                    voxel_in_fov = true;
                    grid_num[index] = TYPE_MAP;
                    Vector3d obs_vec(new_frame_->pos() - pt->pos_);
                    float cur_dist = obs_vec.norm();
                    if (cur_dist <= map_dist[index]) {
                        map_dist[index] = cur_dist;
                        retrieve_voxel_points[index] = pt;
                    }
                }
            }
            if (!voxel_in_fov) {
                delete_key_list.push_back(position);
            }
        }
    }
}

bool VIOManager::TryRaycastHit(
    const Eigen::Vector3d& sample_point_w, float voxel_size,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map) {
    int loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = floor(sample_point_w[j] / voxel_size);
        if (loc_xyz[j] < 0) {
            loc_xyz[j] -= 1.0;
        }
    }
    VoxelLocation sample_pos(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    if (sub_feat_map.find(sample_pos) != sub_feat_map.end()) {
        return true;  // stop ray
    }
    auto corre_feat_map = feat_map.find(sample_pos);
    if (corre_feat_map != feat_map.end()) {
        bool voxel_in_fov = false;
        std::vector<VisualPoint*>& voxel_points =
            corre_feat_map->second->voxel_points;
        for (VisualPoint* pt : voxel_points) {
            if (pt == nullptr || pt->obs_.empty()) {
                continue;
            }
            Vector3d dir(new_frame_->T_f_w_ * pt->pos_);
            if (dir[2] < 0) {
                continue;
            }
            Vector2d pc(new_frame_->w2c(pt->pos_));
            int index = 0;
            if (!new_frame_->cam_->isInFrame(pc.cast<int>(), border) ||
                !GridIndexFromPixel(pc, &index)) {
                continue;
            }
            voxel_in_fov = true;
            grid_num[index] = TYPE_MAP;
            float cur_dist = (new_frame_->pos() - pt->pos_).norm();
            if (cur_dist <= map_dist[index]) {
                map_dist[index] = cur_dist;
                retrieve_voxel_points[index] = pt;
            }
        }
        if (voxel_in_fov) {
            sub_feat_map[sample_pos] = 0;
        }
        return true;
    }
    auto iter = plane_map.find(sample_pos);
    if (iter != plane_map.end()) {
        VoxelOctoTree* current_octo =
            iter->second->find_correspond(sample_point_w);
        if (current_octo->plane_ptr_->is_plane_) {
            pointWithVar plane_center;
            VoxelPlane& plane = *current_octo->plane_ptr_;
            plane_center.point_w = plane.center_;
            plane_center.normal = plane.normal_;
            visual_submap->add_from_voxel_map.push_back(plane_center);
            return true;
        }
    }
    return false;
}

void VIOManager::RaycastEmptyCells(
    float voxel_size,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map) {
    if (!enable_raycast) {
        return;
    }
    for (int i = 0; i < length; i++) {
        if (grid_num[i] == TYPE_MAP || border_flag[i] == 1) {
            continue;
        }
        for (const auto& it : rays_with_sample_points[i]) {
            if (TryRaycastHit(new_frame_->f2w(it), voxel_size, plane_map)) {
                break;
            }
        }
    }
}


bool VIOManager::HasDepthJump(const float* depth_it,
                                      const Vector2d& pc,
                                      const Vector3d& pt_cam) const {
    for (int u = -patch_size_half; u <= patch_size_half; u++) {
        for (int v = -patch_size_half; v <= patch_size_half; v++) {
            if (u == 0 && v == 0)
                continue;

            float depth =
                depth_it[width * (v + int(pc[1])) + u + int(pc[0])];

            if (depth == 0.)
                continue;

            double delta_dist = abs(pt_cam[2] - depth);

            if (delta_dist > 0.5) {
                return true;
            }
        }
    }
    return false;
}

bool VIOManager::SelectRefFeature(VisualPoint* pt, const Vector2d& pc,
                                        Feature*& ref_ftr) {
    if (!pt->is_normal_initialized_)
        return false;

    if (enable_normal) {
        float phtometric_errors_min = std::numeric_limits<float>::max();

        if (pt->obs_.size() == 1) {
            ref_ftr = *pt->obs_.begin();
            pt->ref_patch = ref_ftr;
            pt->has_ref_patch_ = true;
        } else if (!pt->has_ref_patch_) {
            for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite;
                 ++it) {
                Feature* ref_patch_temp = *it;
                float* patch_temp = ref_patch_temp->patch_;
                float phtometric_errors = 0.0;
                int count = 0;
                for (auto itm = pt->obs_.begin(), itme = pt->obs_.end();
                     itm != itme; ++itm) {
                    if ((*itm)->id_ == ref_patch_temp->id_)
                        continue;
                    float* patch_cache = (*itm)->patch_;

                    for (int ind = 0; ind < patch_size_total; ind++) {
                        phtometric_errors +=
                            (patch_temp[ind] - patch_cache[ind]) *
                            (patch_temp[ind] - patch_cache[ind]);
                    }
                    count++;
                }
                phtometric_errors = phtometric_errors / count;
                if (phtometric_errors < phtometric_errors_min) {
                    phtometric_errors_min = phtometric_errors;
                    ref_ftr = ref_patch_temp;
                }
            }
            pt->ref_patch = ref_ftr;
            pt->has_ref_patch_ = true;
        } else {
            ref_ftr = pt->ref_patch;
        }
        return true;
    }

    return pt->GetCloseViewObs(new_frame_->pos(), ref_ftr, pc);
}

void VIOManager::ComputePointWarp(VisualPoint* pt, Feature* ref_ftr,
                                     int& search_level,
                                     Matrix2d& A_cur_ref_zero) {
    if (enable_normal) {
        Vector3d norm_vec =
            (ref_ftr->T_f_w_.rotation_matrix() * pt->normal_).normalized();

        Vector3d pf(ref_ftr->T_f_w_ * pt->pos_);
        // Vector3d pf_norm = pf.normalized();

        // double cos_theta = norm_vec.dot(pf_norm);
        // if(cos_theta < 0) norm_vec = -norm_vec;
        // if (abs(cos_theta) < 0.08) continue; // 0.5 60 degree 0.34 70
        // degree 0.17 80 degree 0.08 85 degree

        SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();

        GetWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec,
                                      T_cur_ref, 0, A_cur_ref_zero);

        search_level = GetBestSearchLevel(A_cur_ref_zero, 2);
    } else {
        auto iter_warp = warp_map.find(ref_ftr->id_);
        if (iter_warp != warp_map.end()) {
            search_level = iter_warp->second->search_level;
            A_cur_ref_zero = iter_warp->second->A_cur_ref;
        } else {
            GetWarpMatrixAffine(
                *cam, ref_ftr->px_, ref_ftr->f_,
                (ref_ftr->pos() - pt->pos_).norm(),
                new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), ref_ftr->level_,
                0, patch_size_half, A_cur_ref_zero);

            search_level = GetBestSearchLevel(A_cur_ref_zero, 2);

            Warp* ot = new Warp(search_level, A_cur_ref_zero);
            warp_map[ref_ftr->id_] = ot;
        }
    }
}

void VIOManager::TryAddRetrievedPoint(cv::Mat img, int grid_index,
                                             const float* depth_it) {
    VisualPoint* pt = retrieve_voxel_points[grid_index];
    // visual_sub_map_cur.push_back(pt); // before

    Vector2d pc(new_frame_->w2c(pt->pos_));

    // cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 3, cv::Scalar(0, 0,
    // 255), -1, 8); // Green Sparse Align tracked

    Vector3d pt_cam(new_frame_->w2f(pt->pos_));
    if (HasDepthJump(depth_it, pc, pt_cam))
        return;

    Feature* ref_ftr = nullptr;
    std::vector<float> patch_wrap(warp_len);

    int search_level;
    Matrix2d A_cur_ref_zero;

    if (!SelectRefFeature(pt, pc, ref_ftr))
        return;

    ComputePointWarp(pt, ref_ftr, search_level, A_cur_ref_zero);

    for (int pyramid_level = 0; pyramid_level <= patch_pyramid_level - 1;
         pyramid_level++) {
        WarpAffine(A_cur_ref_zero, ref_ftr->image_, ref_ftr->px_, ref_ftr->level_,
                   search_level, pyramid_level, patch_size_half,
                   patch_wrap.data());
    }

    GetImagePatch(img, pc, patch_buffer.data(), 0);

    float error = 0.0;
    for (int ind = 0; ind < patch_size_total; ind++) {
        error += (ref_ftr->inv_expo_time_ * patch_wrap[ind] -
                  state->inv_expo_time * patch_buffer[ind]) *
                 (ref_ftr->inv_expo_time_ * patch_wrap[ind] -
                  state->inv_expo_time * patch_buffer[ind]);
    }

    if (enable_ncc) {
        double ncc = CalculateNCC(patch_wrap.data(), patch_buffer.data(),
                                  patch_size_total);
        if (ncc < ncc_threshold) {
            // grid_num[i] = TYPE_UNKNOWN;
            return;
        }
    }

    if (error > outlier_threshold * patch_size_total)
        return;

    visual_submap->voxel_points.push_back(pt);
    visual_submap->propa_errors.push_back(error);
    visual_submap->search_levels.push_back(search_level);
    visual_submap->errors.push_back(error);
    visual_submap->warp_patch.push_back(patch_wrap);
    visual_submap->inv_expo_list.push_back(ref_ftr->inv_expo_time_);
}

void VIOManager::AppendRetrievedToSubmap(cv::Mat img,
                                               const float* depth_it) {
    for (int i = 0; i < length; i++) {
        if (grid_num[i] == TYPE_MAP) {
            TryAddRetrievedPoint(img, i, depth_it);
        }
    }
}

void VIOManager::RetrieveFromVisualSparseMap(
    cv::Mat img, std::vector<pointWithVar>& pg,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map) {
    if (feat_map.size() <= 0)
        return;
    double ts0 = NowSec();
    (void)ts0;

    // pg_down->reserve(feat_map.size());
    // downSizeFilter.setInputCloud(pg);
    // downSizeFilter.filter(*pg_down);

    // resetRvizDisplay();
    visual_submap->Reset();

    // Controls whether to include the visual submap from the previous frame.
    sub_feat_map.clear();

    float voxel_size = 0.5;

    if (!enable_normal)
        warp_map.clear();

    cv::Mat depth_img = cv::Mat::zeros(height, width, CV_32FC1);
    float* it = (float*)depth_img.data;

    // float it[height * width] = {0.0};

    // double t_insert, t_depth, t_position;
    // t_insert=t_depth=t_position=0;

    // printf("A0. initial depthmap: %.6lf \n", NowSec() - ts0);
    // double ts1 = NowSec();

    BuildDepthAndSubmap(pg, depth_img, voxel_size);

    // imshow("depth_img", depth_img);
    // printf("A1: %.6lf \n", NowSec() - ts1);
    // printf("A11. calculate pt position: %.6lf \n", t_position);
    // printf("A12. sub_postion.insert(position): %.6lf \n", t_insert);
    // printf("A13. generate depth map: %.6lf \n", t_depth);
    // printf("A. projection: %.6lf \n", NowSec() - ts0);

    // double t1 = NowSec();
    std::vector<VoxelLocation> DeleteKeyList;

    CollectVisibleSubmapPoints(DeleteKeyList);

    // RayCasting Module
    RaycastEmptyCells(voxel_size, plane_map);

    for (auto& key : DeleteKeyList) {
        sub_feat_map.erase(key);
    }

    // double t2 = NowSec();

    // std::cout<<"B. feat_map.find: "<<t2-t1<<std::endl;

    // double t_2, t_3, t_4, t_5;
    // t_2=t_3=t_4=t_5=0;

    AppendRetrievedToSubmap(img, it);
    total_points = visual_submap->voxel_points.size();

    // double t3 = NowSec();
    // std::cout<<"C. addSubSparseMap: "<<t3-t2<<std::endl;
    // std::cout<<"depthcontinuous: C1 "<<t_2<<" C2 "<<t_3<<" C3 "<<t_4<<" C4
    // "<<t_5<<std::endl;
    if (enable_plot) {
        printf("[ VIO ] Retrieve %d points from visual sparse map\n",
               total_points);
    }
}

void VIOManager::ComputeJacobianAndUpdateEKF(cv::Mat img) {
    if (total_points == 0)
        return;

    compute_jacobian_time = update_ekf_time = 0.0;

    for (int level = patch_pyramid_level - 1; level >= 0; level--) {
        if (enable_inverse_composition) {
            has_ref_patch_cache = false;
            UpdateStateInverse(img, level);
        } else
            UpdateState(img, level);
    }
    state->cov -= G * state->cov;
    UpdateFrameState(*state);
}

void VIOManager::ScoreCloudCandidates(cv::Mat img,
                                           std::vector<pointWithVar>& pg) {
    for (int i = 0; i < pg.size(); i++) {
        if (pg[i].normal == Vector3d(0, 0, 0))
            continue;

        Vector3d pt = pg[i].point_w;
        Vector2d pc(new_frame_->w2c(pt));

        int index = 0;
        if (new_frame_->cam_->isInFrame(pc.cast<int>(), border) &&
            GridIndexFromPixel(pc, &index)) {
            if (grid_num[index] != TYPE_MAP) {
                float cur_value = vision::ShiTomasiScore(img, pc[0], pc[1]);
                // if (cur_value < 5) continue;
                if (cur_value > scan_value[index]) {
                    scan_value[index] = cur_value;
                    append_voxel_points[index] = pg[i];
                    grid_num[index] = TYPE_POINTCLOUD;
                }
            }
        }
    }
}

void VIOManager::ScoreVoxelCandidates(cv::Mat img) {
    for (int j = 0; j < visual_submap->add_from_voxel_map.size(); j++) {
        Vector3d pt = visual_submap->add_from_voxel_map[j].point_w;
        Vector2d pc(new_frame_->w2c(pt));

        int index = 0;
        if (new_frame_->cam_->isInFrame(pc.cast<int>(), border) &&
            GridIndexFromPixel(pc, &index)) {
            if (grid_num[index] != TYPE_MAP) {
                float cur_value = vision::ShiTomasiScore(img, pc[0], pc[1]);
                if (cur_value > scan_value[index]) {
                    scan_value[index] = cur_value;
                    append_voxel_points[index] =
                        visual_submap->add_from_voxel_map[j];
                    grid_num[index] = TYPE_POINTCLOUD;
                }
            }
        }
    }
}

int VIOManager::InsertCandidatePoints(cv::Mat img) {
    int add = 0;
    for (int i = 0; i < length; i++) {
        if (grid_num[i] == TYPE_POINTCLOUD)  // && (scan_value[i]>=50))
        {
            pointWithVar pt_var = append_voxel_points[i];
            Vector3d pt = pt_var.point_w;

            Vector3d norm_vec(new_frame_->T_f_w_.rotation_matrix() *
                              pt_var.normal);
            Vector3d dir(new_frame_->T_f_w_ * pt);
            dir.normalize();
            double cos_theta = dir.dot(norm_vec);
            // if(std::fabs(cos_theta)<0.34) continue; // 70 degree
            Vector2d pc(new_frame_->w2c(pt));

            float* patch = new float[patch_size_total];
            GetImagePatch(img, pc, patch, 0);

            VisualPoint* pt_new = new VisualPoint(pt);

            Vector3d f = cam->cam2world(pc);
            Feature* ftr_new =
                new Feature(pt_new, patch, pc, f, new_frame_->T_f_w_, 0);
            ftr_new->image_ = img;
            ftr_new->id_ = new_frame_->id_;
            ftr_new->inv_expo_time_ = state->inv_expo_time;

            pt_new->AddFrameRef(ftr_new);
            pt_new->covariance_ = pt_var.var;
            pt_new->is_normal_initialized_ = true;

            if (cos_theta < 0) {
                pt_new->normal_ = -pt_var.normal;
            } else {
                pt_new->normal_ = pt_var.normal;
            }

            pt_new->previous_normal_ = pt_new->normal_;

            InsertPointIntoVoxelMap(pt_new);
            add += 1;
            // map_cur_frame.push_back(pt_new);
        }
    }
    return add;
}

void VIOManager::GenerateVisualMapPoints(cv::Mat img,
                                         std::vector<pointWithVar>& pg) {
    if (pg.size() <= 10)
        return;

    // double t0 = NowSec();
    ScoreCloudCandidates(img, pg);
    ScoreVoxelCandidates(img);

    // double t_b1 = NowSec() - t0;
    // t0 = NowSec();

    int add = InsertCandidatePoints(img);

    // double t_b2 = NowSec() - t0;

    if (enable_plot) {
        printf("[ VIO ] Append %d new visual map points\n", add);
    }
}

void VIOManager::UpdateVisualMapPoints(cv::Mat img) {
    if (total_points == 0)
        return;

    int update_num = 0;
    SE3 pose_cur = new_frame_->T_f_w_;
    for (int i = 0; i < total_points; i++) {
        VisualPoint* pt = visual_submap->voxel_points[i];
        if (pt == nullptr)
            continue;
        if (pt->is_converged_) {
            pt->DeleteNonRefPatchFeatures();
            continue;
        }

        Vector2d pc(new_frame_->w2c(pt->pos_));
        bool add_flag = false;

        float* patch_temp = new float[patch_size_total];
        GetImagePatch(img, pc, patch_temp, 0);
        // TODO: condition: distance and view_angle
        // Step 1: time
        Feature* last_feature = pt->obs_.back();
        // if(new_frame_->id_ >= last_feature->id_ + 10) add_flag = true; // 10

        // Step 2: delta_pose
        SE3 pose_ref = last_feature->T_f_w_;
        SE3 delta_pose = pose_ref * pose_cur.inverse();
        double delta_p = delta_pose.translation().norm();
        double delta_theta =
            (delta_pose.rotation_matrix().trace() > 3.0 - 1e-6)
                ? 0.0
                : std::acos(0.5 * (delta_pose.rotation_matrix().trace() - 1));
        if (delta_p > 0.5 || delta_theta > 0.3)
            add_flag = true;  // 0.5 || 0.3

        // Step 3: pixel distance
        Vector2d last_px = last_feature->px_;
        double pixel_dist = (pc - last_px).norm();
        if (pixel_dist > 40)
            add_flag = true;

        // Maintain the size of 3D point observation features.
        if (pt->obs_.size() >= 30) {
            Feature* ref_ftr;
            pt->FindMinScoreFeature(new_frame_->pos(), ref_ftr);
            pt->DeleteFeatureRef(ref_ftr);
            // std::cout<<"pt->obs_.size() exceed 20 !!!!!!"<<std::endl;
        }
        if (add_flag) {
            update_num += 1;
            update_flag[i] = 1;
            Vector3d f = cam->cam2world(pc);
            Feature* ftr_new =
                new Feature(pt, patch_temp, pc, f, new_frame_->T_f_w_,
                            visual_submap->search_levels[i]);
            ftr_new->image_ = img;
            ftr_new->id_ = new_frame_->id_;
            ftr_new->inv_expo_time_ = state->inv_expo_time;
            pt->AddFrameRef(ftr_new);
        }
    }
    if (enable_plot) {
        printf("[ VIO ] Update %d points in visual submap\n", update_num);
    }
}

void VIOManager::UpdateNormalFromPlane(
    VisualPoint* pt,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map) {
    const Vector3d& p_w = pt->pos_;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = p_w[j] / 0.5;
        if (loc_xyz[j] < 0) {
            loc_xyz[j] -= 1.0;
        }
    }
    VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                            (int64_t)loc_xyz[2]);
    auto iter = plane_map.find(position);
    if (iter != plane_map.end()) {
        VoxelOctoTree* current_octo;
        current_octo = iter->second->find_correspond(p_w);
        if (current_octo->plane_ptr_->is_plane_) {
            VoxelPlane& plane = *current_octo->plane_ptr_;
            float dis_to_plane = plane.normal_(0) * p_w(0) +
                                 plane.normal_(1) * p_w(1) +
                                 plane.normal_(2) * p_w(2) + plane.d_;
            float dis_to_plane_abs = fabs(dis_to_plane);
            float dis_to_center =
                (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
            float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
            if (range_dis <= 3 * plane.radius_) {
                Eigen::Matrix<double, 1, 6> J_nq;
                J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
                J_nq.block<1, 3>(0, 3) = -plane.normal_;
                double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
                sigma_l +=
                    plane.normal_.transpose() * pt->covariance_ * plane.normal_;

                if (dis_to_plane_abs < 3 * sqrt(sigma_l)) {
                    // Vector3d
                    // norm_vec(new_frame_->T_f_w_.rotation_matrix() *
                    // plane.normal_); Vector3d pf(new_frame_->T_f_w_ *
                    // pt->pos_); Vector3d pf_ref(pt->ref_patch->T_f_w_ *
                    // pt->pos_); Vector3d
                    // norm_vec_ref(pt->ref_patch->T_f_w_.rotation_matrix()
                    // * plane.normal); double cos_ref =
                    // pf_ref.dot(norm_vec_ref);

                    if (pt->previous_normal_.dot(plane.normal_) < 0) {
                        pt->normal_ = -plane.normal_;
                    } else {
                        pt->normal_ = plane.normal_;
                    }

                    double normal_update =
                        (pt->normal_ - pt->previous_normal_).norm();

                    pt->previous_normal_ = pt->normal_;

                    if (normal_update < 0.0001 && pt->obs_.size() > 10) {
                        pt->is_converged_ = true;
                        // visual_converged_point.push_back(pt);
                    }
                }
            }
        }
    }
}

void VIOManager::SelectBestRefPatch(VisualPoint* pt) {
    float score_max = -1000.;
    for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it) {
        Feature* ref_patch_temp = *it;
        float* patch_temp = ref_patch_temp->patch_;
        float NCC_up = 0.0;
        float NCC_down1 = 0.0;
        float NCC_down2 = 0.0;
        float NCC = 0.0;
        float score = 0.0;
        int count = 0;

        Vector3d pf = ref_patch_temp->T_f_w_ * pt->pos_;
        Vector3d norm_vec =
            ref_patch_temp->T_f_w_.rotation_matrix() * pt->normal_;
        pf.normalize();
        double cos_angle = pf.dot(norm_vec);
        // if(fabs(cos_angle) < 0.86) continue; // 20 degree

        float ref_mean = static_cast<float>(ref_patch_temp->mean_);
        if (std::abs(ref_patch_temp->mean_) < 1e-6) {
            float ref_sum =
                std::accumulate(patch_temp, patch_temp + patch_size_total, 0.0f);
            ref_mean = ref_sum / patch_size_total;
            ref_patch_temp->mean_ = ref_mean;
        }

        for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme;
             ++itm) {
            if ((*itm)->id_ == ref_patch_temp->id_)
                continue;
            float* patch_cache = (*itm)->patch_;

            float other_mean = static_cast<float>((*itm)->mean_);
            if (std::abs((*itm)->mean_) < 1e-6) {
                float other_sum = std::accumulate(
                    patch_cache, patch_cache + patch_size_total, 0.0f);
                other_mean = other_sum / patch_size_total;
                (*itm)->mean_ = other_mean;
            }

            for (int ind = 0; ind < patch_size_total; ind++) {
                NCC_up += (patch_temp[ind] - ref_mean) *
                          (patch_cache[ind] - other_mean);
                NCC_down1 += (patch_temp[ind] - ref_mean) *
                             (patch_temp[ind] - ref_mean);
                NCC_down2 += (patch_cache[ind] - other_mean) *
                             (patch_cache[ind] - other_mean);
            }
            NCC += fabs(NCC_up / sqrt(NCC_down1 * NCC_down2));
            count++;
        }

        NCC = NCC / count;

        score = NCC + cos_angle;

        ref_patch_temp->score_ = score;

        if (score > score_max) {
            score_max = score;
            pt->ref_patch = ref_patch_temp;
            pt->has_ref_patch_ = true;
        }
    }
}

void VIOManager::UpdateReferencePatch(
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map) {
    if (total_points == 0)
        return;

    for (int i = 0; i < visual_submap->voxel_points.size(); i++) {
        VisualPoint* pt = visual_submap->voxel_points[i];

        if (!pt->is_normal_initialized_)
            continue;
        if (pt->is_converged_)
            continue;
        if (pt->obs_.size() <= 5)
            continue;
        if (update_flag[i] == 0)
            continue;

        UpdateNormalFromPlane(pt, plane_map);
        SelectBestRefPatch(pt);
    }
}


void VIOManager::WarpPatchesByNormal(uchar* it_normal, int& num,
                                              const std::string& dir) {
    const int patch_size_dbg = 25;
    for (int i = 0; i < static_cast<int>(visual_submap->voxel_points.size());
         i++) {
        VisualPoint* pt = visual_submap->voxel_points[i];
        if (!pt->is_normal_initialized_) {
            continue;
        }
        Feature* ref_ftr = pt->ref_patch;
        Vector2d pc(new_frame_->w2c(pt->pos_));
        Vector2d pc_prior(new_frame_->w2c_prior(pt->pos_));

        Vector3d norm_vec(ref_ftr->T_f_w_.rotation_matrix() * pt->normal_);
        Vector3d pf(ref_ftr->T_f_w_ * pt->pos_);
        if (pf.dot(norm_vec) < 0) {
            norm_vec = -norm_vec;
        }

        SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();
        Matrix2d A_cur_ref;
        GetWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec,
                                      T_cur_ref, 0, A_cur_ref);
        int search_level = GetBestSearchLevel(A_cur_ref.inverse(), 2);
        if (A_cur_ref.determinant() > 3) {
            continue;
        }
        num++;
        DrawRefCurOverlay(i, ref_ftr, pc, pc_prior, num, dir);
        RasterizeWarpedPatch(ref_ftr, pc, A_cur_ref, search_level,
                                 it_normal);
        (void)patch_size_dbg;
    }
}

void VIOManager::RasterizeWarpedPatch(Feature* ref_ftr, const Vector2d& pc,
                                          const Eigen::Matrix2d& A_cur_ref,
                                          int search_level, uchar* it_normal) {
    const int patch_size_dbg = 25;
    cv::Mat img_ref = ref_ftr->image_;
    float x_min = 1000.f;
    float y_min = 1000.f;
    float x_max = 0.f;
    float y_max = 0.f;
    for (int y = 0; y < patch_size_dbg; ++y) {
        for (int x = 0; x < patch_size_dbg; ++x) {
            Vector2f px_patch(x - patch_size_dbg / 2, y - patch_size_dbg / 2);
            px_patch *= (1 << search_level);
            const Vector2f px(A_cur_ref.cast<float>() * px_patch +
                              pc.cast<float>());
            if (px[0] < 0 || px[1] < 0 || px[0] >= new_frame_->image_.cols - 1 ||
                px[1] >= new_frame_->image_.rows - 1) {
                continue;
            }
            x_min = std::min(x_min, px[0]);
            y_min = std::min(y_min, px[1]);
            x_max = std::max(x_max, px[0]);
            y_max = std::max(y_max, px[1]);
        }
    }
    Matrix2f A_inv = A_cur_ref.inverse().cast<float>();
    const float half =
        static_cast<float>(patch_size_dbg / 2 * (1 << search_level));
    for (int i = static_cast<int>(std::floor(x_min));
         i < static_cast<int>(std::ceil(x_max)); i++) {
        for (int j = static_cast<int>(std::floor(y_min));
             j < static_cast<int>(std::ceil(y_max)); j++) {
            Vector2f px_patch = A_inv * (Eigen::Vector2f(i, j) - pc.cast<float>());
            if (px_patch[0] <= -half || px_patch[0] >= half ||
                px_patch[1] <= -half || px_patch[1] >= half) {
                continue;
            }
            const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
            it_normal[width * j + i] = static_cast<uint8_t>(
                vision::InterpolateMat8u(img_ref, px_ref[0], px_ref[1]));
        }
    }
}

void VIOManager::WarpPatchesByAffine(uchar* it) {
    const int patch_size_dbg = 25;
    for (int i = 0; i < static_cast<int>(visual_submap->voxel_points.size());
         i++) {
        VisualPoint* pt = visual_submap->voxel_points[i];
        if (!pt->is_normal_initialized_) {
            continue;
        }
        Feature* ref_ftr = pt->ref_patch;
        Vector2d pc(new_frame_->w2c(pt->pos_));
        Matrix2d A_cur_ref;
        GetWarpMatrixAffine(*cam, ref_ftr->px_, ref_ftr->f_,
                            (ref_ftr->pos() - pt->pos_).norm(),
                            new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0,
                            patch_size_half, A_cur_ref);
        int search_level = GetBestSearchLevel(A_cur_ref.inverse(), 2);
        if (A_cur_ref.determinant() > 3) {
            continue;
        }
        cv::Mat img_ref = ref_ftr->image_;
        for (int y = 0; y < patch_size_dbg; ++y) {
            for (int x = 0; x < patch_size_dbg; ++x) {
                Vector2f px_patch(x - patch_size_dbg / 2, y - patch_size_dbg / 2);
                px_patch *= (1 << search_level);
                const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
                uint8_t pixel_value = static_cast<uint8_t>(
                    vision::InterpolateMat8u(img_ref, px_ref[0], px_ref[1]));
                const Vector2f px(A_cur_ref.cast<float>() * px_patch +
                                  pc.cast<float>());
                if (px[0] < 0 || px[1] < 0 || px[0] >= new_frame_->image_.cols - 1 ||
                    px[1] >= new_frame_->image_.rows - 1) {
                    continue;
                }
                it[width * static_cast<int>(px[1]) + static_cast<int>(px[0])] =
                    pixel_value;
            }
        }
    }
}

void VIOManager::SavePatchDebugMosaics(
    const cv::Mat& result, const cv::Mat& result_normal,
    const cv::Mat& img_photometric_error, const std::string& dir) {
    cv::Mat ref_cur_combine;
    cv::Mat ref_cur_combine_normal;
    cv::Mat ref_cur_combine_error;
    cv::Mat err = img_photometric_error.clone();
    cv::hconcat(result, new_frame_->image_, ref_cur_combine);
    cv::hconcat(result_normal, new_frame_->image_, ref_cur_combine_normal);
    cv::cvtColor(ref_cur_combine, ref_cur_combine, cv::COLOR_GRAY2BGR);
    cv::cvtColor(ref_cur_combine_normal, ref_cur_combine_normal,
                 cv::COLOR_GRAY2BGR);
    cv::absdiff(err, result_normal, err);
    cv::hconcat(err, new_frame_->image_, ref_cur_combine_error);
    cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_.png",
                ref_cur_combine);
    cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_photometric.png",
                ref_cur_combine_error);
    cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_normal.png",
                ref_cur_combine_normal);
}

void VIOManager::ProjectPatchFromRefToCur(
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& /*plane_map*/) {
    if (total_points == 0) {
        return;
    }
    const std::string dir = log_dir.empty() ? std::string("Log/ref_cur_combine/")
                                            : (log_dir + "/ref_cur_combine/");
    cv::Mat result = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat result_normal = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat img_photometric_error = new_frame_->image_.clone();
    int num = 0;
    WarpPatchesByNormal(result_normal.data, num, dir);
    WarpPatchesByAffine(result.data);
    SavePatchDebugMosaics(result, result_normal, img_photometric_error,
                                    dir);
}

void VIOManager::DrawRefCurOverlay(
    int submap_index, Feature* ref_ftr, const Vector2d& pc,
    const Vector2d& pc_prior, int num, const std::string& dir) {
    cv::Mat img_cur = new_frame_->image_;
    cv::Mat img_ref = ref_ftr->image_;
    cv::Mat ref_cur_combine_temp;
    int radius = 20;
    cv::hconcat(img_cur, img_ref, ref_cur_combine_temp);
    cv::cvtColor(ref_cur_combine_temp, ref_cur_combine_temp, cv::COLOR_GRAY2BGR);

    GetImagePatch(img_cur, pc, patch_buffer.data(), 0);

    float error_est = 0.0;
    float error_gt = 0.0;

    const int sub_idx = submap_index;
    if (sub_idx >= 0 &&
        sub_idx < static_cast<int>(visual_submap->warp_patch.size())) {
        for (int ind = 0; ind < patch_size_total; ind++) {
            error_est +=
                (ref_ftr->inv_expo_time_ *
                     visual_submap->warp_patch[sub_idx][ind] -
                 state->inv_expo_time * patch_buffer[ind]) *
                (ref_ftr->inv_expo_time_ *
                     visual_submap->warp_patch[sub_idx][ind] -
                 state->inv_expo_time * patch_buffer[ind]);
        }
    }
    std::string ref_est =
        "ref_est " + std::to_string(1.0 / ref_ftr->inv_expo_time_);
    std::string cur_est =
        "cur_est " + std::to_string(1.0 / state->inv_expo_time);
    std::string cur_propa = "cur_gt " + std::to_string(error_gt);
    std::string cur_optimize = "cur_est " + std::to_string(error_est);

    cv::putText(ref_cur_combine_temp, ref_est,
                cv::Point2f(ref_ftr->px_[0] + img_cur.cols - 40,
                            ref_ftr->px_[1] + 40),
                cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::putText(ref_cur_combine_temp, cur_est,
                cv::Point2f(pc[0] - 40, pc[1] + 40), cv::FONT_HERSHEY_COMPLEX,
                0.4, cv::Scalar(0, 255, 0), 1, 8, 0);
    cv::putText(ref_cur_combine_temp, cur_propa,
                cv::Point2f(pc[0] - 40, pc[1] + 60), cv::FONT_HERSHEY_COMPLEX,
                0.4, cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::putText(ref_cur_combine_temp, cur_optimize,
                cv::Point2f(pc[0] - 40, pc[1] + 80), cv::FONT_HERSHEY_COMPLEX,
                0.4, cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::rectangle(ref_cur_combine_temp,
                  cv::Point2f(ref_ftr->px_[0] + img_cur.cols - radius,
                              ref_ftr->px_[1] - radius),
                  cv::Point2f(ref_ftr->px_[0] + img_cur.cols + radius,
                              ref_ftr->px_[1] + radius),
                  cv::Scalar(0, 0, 255), 1);
    cv::rectangle(ref_cur_combine_temp,
                  cv::Point2f(pc[0] - radius, pc[1] - radius),
                  cv::Point2f(pc[0] + radius, pc[1] + radius),
                  cv::Scalar(0, 255, 0), 1);
    cv::rectangle(ref_cur_combine_temp,
                  cv::Point2f(pc_prior[0] - radius, pc_prior[1] - radius),
                  cv::Point2f(pc_prior[0] + radius, pc_prior[1] + radius),
                  cv::Scalar(255, 255, 255), 1);
    cv::circle(ref_cur_combine_temp,
               cv::Point2f(ref_ftr->px_[0] + img_cur.cols, ref_ftr->px_[1]), 1,
               cv::Scalar(0, 0, 255), -1, 8);
    cv::circle(ref_cur_combine_temp, cv::Point2f(pc[0], pc[1]), 1,
               cv::Scalar(0, 255, 0), -1, 8);
    cv::circle(ref_cur_combine_temp, cv::Point2f(pc_prior[0], pc_prior[1]), 1,
               cv::Scalar(255, 255, 255), -1, 8);
    cv::imwrite(dir + std::to_string(new_frame_->id_) + "_" +
                    std::to_string(ref_ftr->id_) + "_" + std::to_string(num) +
                    ".png",
                ref_cur_combine_temp);
}

void VIOManager::FillRefPatchJacobians(int level, int point_index) {
    VisualPoint* pt = visual_submap->voxel_points[point_index];
    if (pt == nullptr) {
        return;
    }
    const int scale = (1 << level);
    cv::Mat img = pt->ref_patch->image_;
    double depth((pt->pos_ - pt->ref_patch->pos()).norm());
    Vector3d pf = pt->ref_patch->f_ * depth;
    Vector2d pc = pt->ref_patch->px_;
    Matrix3d R_ref_w = pt->ref_patch->T_f_w_.rotation_matrix();
    Eigen::Matrix<double, 2, 3> Jdpi;
    MD(1, 2) Jimg;
    MD(1, 3) JdR, Jdt;
    Matrix3d p_w_hat;
    ComputeProjectionJacobian(pf, Jdpi);
    p_w_hat << SKEW_SYM_MATRX(pt->pos_);

    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0] / scale) * scale;
    const int v_ref_i = floorf(pc[1] / scale) * scale;
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;

    for (int x = 0; x < patch_size; x++) {
        uint8_t* img_ptr =
            (uint8_t*)img.data +
            (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i -
            patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale) {
            float du =
                0.5f *
                ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] +
                  w_ref_bl * img_ptr[scale * width + scale] +
                  w_ref_br * img_ptr[scale * width + scale * 2]) -
                 (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] +
                  w_ref_bl * img_ptr[scale * width - scale] +
                  w_ref_br * img_ptr[scale * width]));
            float dv =
                0.5f *
                ((w_ref_tl * img_ptr[scale * width] +
                  w_ref_tr * img_ptr[scale + scale * width] +
                  w_ref_bl * img_ptr[width * scale * 2] +
                  w_ref_br * img_ptr[width * scale * 2 + scale]) -
                 (w_ref_tl * img_ptr[-scale * width] +
                  w_ref_tr * img_ptr[-scale * width + scale] +
                  w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));
            Jimg << du, dv;
            Jimg = Jimg * (1.0 / scale);
            JdR = Jimg * Jdpi * R_ref_w * p_w_hat;
            Jdt = -Jimg * Jdpi * R_ref_w;
            H_sub_inv.block<1, 6>(point_index * patch_size_total +
                                      x * patch_size + y,
                                  0)
                << JdR,
                Jdt;
        }
    }
}

void VIOManager::PrecomputeReferencePatches(int level) {
    if (total_points == 0) {
        return;
    }
    const int H_DIM = total_points * patch_size_total;
    H_sub_inv.resize(H_DIM, 6);
    H_sub_inv.setZero();
    for (int i = 0; i < total_points; i++) {
        FillRefPatchJacobians(level, i);
    }
    has_ref_patch_cache = true;
}


float VIOManager::AccumulateInverseResiduals(
    cv::Mat img, int level, Eigen::VectorXd& z, Eigen::MatrixXd& H_sub,
    int& n_meas) {
    if (!has_ref_patch_cache) {
        PrecomputeReferencePatches(level);
    }
    float error = 0.0;
    n_meas = 0;
    Matrix3d Rwi(state->rot_end);
    Vector3d Pwi(state->pos_end);
    Matrix3d P_wi_hat;
    P_wi_hat << SKEW_SYM_MATRX(Pwi);
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
    MD(1, 3) JdR, Jdt;

    for (int i = 0; i < total_points; i++) {
        float patch_error = 0.0;
        const int scale = (1 << level);
        VisualPoint* pt = visual_submap->voxel_points[i];
        if (pt == nullptr) {
            continue;
        }
        Vector3d pf = Rcw * pt->pos_ + Pcw;
        Vector2d pc = cam->world2cam(pf);
        const float u_ref = pc[0];
        const float v_ref = pc[1];
        const int u_ref_i = floorf(pc[0] / scale) * scale;
        const int v_ref_i = floorf(pc[1] / scale) * scale;
        const float subpix_u_ref = (u_ref - u_ref_i) / scale;
        const float subpix_v_ref = (v_ref - v_ref_i) / scale;
        const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
        const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
        const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
        const float w_ref_br = subpix_u_ref * subpix_v_ref;
        std::vector<float> P = visual_submap->warp_patch[i];
        for (int x = 0; x < patch_size; x++) {
            uint8_t* img_ptr =
                (uint8_t*)img.data +
                (v_ref_i + x * scale - patch_size_half * scale) * width +
                u_ref_i - patch_size_half * scale;
            for (int y = 0; y < patch_size; ++y, img_ptr += scale) {
                double res =
                    w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
                    w_ref_bl * img_ptr[scale * width] +
                    w_ref_br * img_ptr[scale * width + scale] -
                    P[patch_size_total * level + x * patch_size + y];
                z(i * patch_size_total + x * patch_size + y) = res;
                patch_error += res * res;
                MD(1, 3)
                J_dR = H_sub_inv.block<1, 3>(
                    i * patch_size_total + x * patch_size + y, 0);
                MD(1, 3)
                J_dt = H_sub_inv.block<1, 3>(
                    i * patch_size_total + x * patch_size + y, 3);
                JdR = J_dR * Rwi + J_dt * P_wi_hat * Rwi;
                Jdt = J_dt * Rwi;
                H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0)
                    << JdR,
                    Jdt;
                n_meas++;
            }
        }
        visual_submap->errors[i] = patch_error;
        error += patch_error;
    }
    return (n_meas > 0) ? (error / n_meas) : error;
}

bool VIOManager::ApplyInverseUpdate(StatesGroup& old_state, float error,
                                       float& last_error,
                                       const Eigen::VectorXd& z,
                                       const Eigen::MatrixXd& H_sub) {
    if (error <= last_error) {
        old_state = (*state);
        last_error = error;
        auto&& H_sub_T = H_sub.transpose();
        H_T_H.setZero();
        G.setZero();
        H_T_H.block<6, 6>(0, 0) = H_sub_T * H_sub;
        MD(frontend::kDimState, frontend::kDimState) && K_1 =
            (H_T_H + (state->cov / image_point_cov).inverse()).inverse();
        auto&& HTz = H_sub_T * z;
        auto vec = (*state_propagated) - (*state);
        G.block<frontend::kDimState, 6>(0, 0) =
            K_1.block<frontend::kDimState, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
        auto solution =
            -K_1.block<frontend::kDimState, 6>(0, 0) * HTz + vec -
            G.block<frontend::kDimState, 6>(0, 0) * vec.block<6, 1>(0, 0);
        (*state) += solution;
        auto&& rot_add = solution.block<3, 1>(0, 0);
        auto&& t_add = solution.block<3, 1>(3, 0);
        if ((rot_add.norm() * 57.3f < 0.001f) &&
            (t_add.norm() * 100.0f < 0.001f)) {
            return true;
        }
        return false;
    }
    (*state) = old_state;
    return true;
}

void VIOManager::UpdateStateInverse(cv::Mat img, int level) {
    if (total_points == 0) {
        return;
    }
    StatesGroup old_state = (*state);
    VectorXd z;
    MatrixXd H_sub;
    float last_error = std::numeric_limits<float>::max();
    compute_jacobian_time = update_ekf_time = 0.0;
    const int H_DIM = total_points * patch_size_total;
    z.resize(H_DIM);
    z.setZero();
    H_sub.resize(H_DIM, 6);
    H_sub.setZero();

    for (int iteration = 0; iteration < max_iterations; iteration++) {
        double t1 = NowSec();
        int n_meas = 0;
        float error = AccumulateInverseResiduals(img, level, z, H_sub, n_meas);
        compute_jacobian_time += NowSec() - t1;
        double t3 = NowSec();
        bool EKF_end =
            ApplyInverseUpdate(old_state, error, last_error, z, H_sub);
        update_ekf_time += NowSec() - t3;
        if (iteration == max_iterations || EKF_end) {
            break;
        }
    }
}


float VIOManager::FillPointResidualsForward(cv::Mat img, int level,
                                            int point_index, Eigen::VectorXd& z,
                                            Eigen::MatrixXd& H_sub,
                                            int& n_meas) {
    MD(1, 2) Jimg;
    Eigen::Matrix<double, 2, 3> Jdpi;
    MD(1, 3) Jdphi, Jdp, JdR, Jdt;
    float patch_error = 0.0;
    int search_level = visual_submap->search_levels[point_index];
    int scale = (1 << (level + search_level));
    float inv_scale = 1.0f / scale;
    VisualPoint* pt = visual_submap->voxel_points[point_index];
    if (pt == nullptr) {
        return 0.f;
    }
    Vector3d pf = Rcw * pt->pos_ + Pcw;
    Vector2d pc = cam->world2cam(pf);
    ComputeProjectionJacobian(pf, Jdpi);
    Matrix3d p_hat;
    p_hat << SKEW_SYM_MATRX(pf);
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0] / scale) * scale;
    const int v_ref_i = floorf(pc[1] / scale) * scale;
    // Central-difference (±scale) + bilinear (+scale) need a 2*scale halo.
    const int u0 = u_ref_i - patch_size_half * scale;
    const int v0 = v_ref_i - patch_size_half * scale;
    const int u1 = u0 + (patch_size - 1) * scale + 2 * scale;
    const int v1 = v0 + (patch_size - 1) * scale + 2 * scale;
    const int img_w = img.cols > 0 ? img.cols : width;
    const int img_h = img.rows > 0 ? img.rows : height;
    if (img.empty() || img.data == nullptr || width <= 0 || height <= 0 ||
        u0 - scale < 0 || v0 - scale < 0 || u1 >= img_w || v1 >= img_h ||
        u1 >= width || v1 >= height) {
        return 0.f;
    }
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    if (point_index < 0 ||
        point_index >= static_cast<int>(visual_submap->warp_patch.size()) ||
        point_index >= static_cast<int>(visual_submap->inv_expo_list.size())) {
        return 0.f;
    }
    const std::vector<float>& P = visual_submap->warp_patch[point_index];
    const double inv_ref_expo = visual_submap->inv_expo_list[point_index];
    auto sample = [&](uint8_t* p) {
        return w_ref_tl * p[0] + w_ref_tr * p[scale] +
               w_ref_bl * p[scale * width] + w_ref_br * p[scale * width + scale];
    };
    for (int x = 0; x < patch_size; x++) {
        uint8_t* img_ptr =
            (uint8_t*)img.data +
            (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i -
            patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale) {
            const float du = 0.5f * (sample(img_ptr + scale) - sample(img_ptr - scale));
            const float dv =
                0.5f * (sample(img_ptr + scale * width) -
                        sample(img_ptr - scale * width));
            Jimg << du, dv;
            Jimg = Jimg * state->inv_expo_time * inv_scale;
            Jdphi = Jimg * Jdpi * p_hat;
            Jdp = -Jimg * Jdpi;
            JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
            Jdt = Jdp * Jdp_dt;
            const double cur_value = sample(img_ptr);
            const double res =
                state->inv_expo_time * cur_value -
                inv_ref_expo * P[patch_size_total * level + x * patch_size + y];
            const int row = point_index * patch_size_total + x * patch_size + y;
            z(row) = res;
            patch_error += static_cast<float>(res * res);
            n_meas += 1;
            if (enable_exposure_estimate) {
                H_sub.block<1, 7>(row, 0) << JdR, Jdt, cur_value;
            } else {
                H_sub.block<1, 6>(row, 0) << JdR, Jdt;
            }
        }
    }
    visual_submap->errors[point_index] = patch_error;
    return patch_error;
}

float VIOManager::AccumulateForwardResiduals(
    cv::Mat img, int level, Eigen::VectorXd& z, Eigen::MatrixXd& H_sub,
    int& n_meas) {
    Matrix3d Rwi(state->rot_end);
    Vector3d Pwi(state->pos_end);
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
    Jdp_dt = Rci * Rwi.transpose();
    float error = 0.0;
    n_meas = 0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for reduction(+ : error, n_meas)
#endif
    for (int i = 0; i < total_points; i++) {
        error += FillPointResidualsForward(img, level, i, z, H_sub, n_meas);
    }
    return (n_meas > 0) ? (error / n_meas) : error;
}

bool VIOManager::ApplyForwardUpdate(StatesGroup& old_state, float error,
                                       float& last_error,
                                       const Eigen::VectorXd& z,
                                       const Eigen::MatrixXd& H_sub) {
    if (error <= last_error) {
        old_state = (*state);
        last_error = error;
        auto&& H_sub_T = H_sub.transpose();
        H_T_H.setZero();
        G.setZero();
        H_T_H.block<7, 7>(0, 0) = H_sub_T * H_sub;
        MD(frontend::kDimState, frontend::kDimState) && K_1 =
            (H_T_H + (state->cov / image_point_cov).inverse()).inverse();
        auto&& HTz = H_sub_T * z;
        auto vec = (*state_propagated) - (*state);
        G.block<frontend::kDimState, 7>(0, 0) =
            K_1.block<frontend::kDimState, 7>(0, 0) * H_T_H.block<7, 7>(0, 0);
        MD(frontend::kDimState, 1)
        solution = -K_1.block<frontend::kDimState, 7>(0, 0) * HTz + vec -
                   G.block<frontend::kDimState, 7>(0, 0) * vec.block<7, 1>(0, 0);
        (*state) += solution;
        auto&& rot_add = solution.block<3, 1>(0, 0);
        auto&& t_add = solution.block<3, 1>(3, 0);
        if ((rot_add.norm() * 57.3f < 0.001f) &&
            (t_add.norm() * 100.0f < 0.001f)) {
            return true;
        }
        return false;
    }
    (*state) = old_state;
    return true;
}

void VIOManager::UpdateState(cv::Mat img, int level) {
    if (total_points == 0) {
        return;
    }
    StatesGroup old_state = (*state);
    VectorXd z;
    MatrixXd H_sub;
    float last_error = std::numeric_limits<float>::max();
    const int H_DIM = total_points * patch_size_total;
    z.resize(H_DIM);
    z.setZero();
    H_sub.resize(H_DIM, 7);
    H_sub.setZero();

    for (int iteration = 0; iteration < max_iterations; iteration++) {
        double t1 = NowSec();
        int n_meas = 0;
        float error = AccumulateForwardResiduals(img, level, z, H_sub, n_meas);
        compute_jacobian_time += NowSec() - t1;
        double t3 = NowSec();
        bool EKF_end =
            ApplyForwardUpdate(old_state, error, last_error, z, H_sub);
        update_ekf_time += NowSec() - t3;
        if (iteration == max_iterations || EKF_end) {
            break;
        }
    }
}


void VIOManager::UpdateFrameState(StatesGroup state) {
    Matrix3d Rwi(state.rot_end);
    Vector3d Pwi(state.pos_end);
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
    new_frame_->T_f_w_ = SE3(Rcw, Pcw);
}

void VIOManager::PlotTrackedPoints() {
    int total_points = visual_submap->voxel_points.size();
    if (total_points == 0)
        return;
    // int inlier_count = 0;
    // for (int i = 0; i < img_cp.rows / grid_size; i++)
    // {
    //   cv::line(img_cp, cv::Poaint2f(0, grid_size * i),
    //   cv::Point2f(img_cp.cols, grid_size * i), cv::Scalar(255, 255, 255), 1,
    //   CV_AA);
    // }
    // for (int i = 0; i < img_cp.cols / grid_size; i++)
    // {
    //   cv::line(img_cp, cv::Point2f(grid_size * i, 0), cv::Point2f(grid_size *
    //   i, img_cp.rows), cv::Scalar(255, 255, 255), 1, CV_AA);
    // }
    // for (int i = 0; i < img_cp.rows / grid_size; i++)
    // {
    //   cv::line(img_cp, cv::Point2f(0, grid_size * i),
    //   cv::Point2f(img_cp.cols, grid_size * i), cv::Scalar(255, 255, 255), 1,
    //   CV_AA);
    // }
    // for (int i = 0; i < img_cp.cols / grid_size; i++)
    // {
    //   cv::line(img_cp, cv::Point2f(grid_size * i, 0), cv::Point2f(grid_size *
    //   i, img_cp.rows), cv::Scalar(255, 255, 255), 1, CV_AA);
    // }
    for (int i = 0; i < total_points; i++) {
        VisualPoint* pt = visual_submap->voxel_points[i];
        Vector2d pc(new_frame_->w2c(pt->pos_));

        if (visual_submap->errors[i] <= visual_submap->propa_errors[i]) {
            // inlier_count++;
            cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7,
                       cv::Scalar(0, 255, 0), -1,
                       8);  // Green Sparse Align tracked
        } else {
            cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7,
                       cv::Scalar(255, 0, 0), -1,
                       8);  // Blue Sparse Align tracked
        }
    }
    // std::string text = std::to_string(inlier_count) + " " +
    // std::to_string(total_points); cv::Point2f origin; origin.x = img_cp.cols
    // - 110; origin.y = 20; cv::putText(img_cp, text, origin,
    // cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, 8, 0);
}

Vector3f VIOManager::GetInterpolatedPixel(cv::Mat img, Vector2d pc) {
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0]);
    const int v_ref_i = floorf(pc[1]);
    const float subpix_u_ref = (u_ref - u_ref_i);
    const float subpix_v_ref = (v_ref - v_ref_i);
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    uint8_t* img_ptr = (uint8_t*)img.data + ((v_ref_i)*width + (u_ref_i)) * 3;
    float B = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[0 + 3] +
              w_ref_bl * img_ptr[width * 3] +
              w_ref_br * img_ptr[width * 3 + 0 + 3];
    float G = w_ref_tl * img_ptr[1] + w_ref_tr * img_ptr[1 + 3] +
              w_ref_bl * img_ptr[1 + width * 3] +
              w_ref_br * img_ptr[width * 3 + 1 + 3];
    float R = w_ref_tl * img_ptr[2] + w_ref_tr * img_ptr[2 + 3] +
              w_ref_bl * img_ptr[2 + width * 3] +
              w_ref_br * img_ptr[width * 3 + 2 + 3];
    Vector3f pixel(B, G, R);
    return pixel;
}

void VIOManager::DumpDataForColmap() {
    static int cnt = 1;
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << cnt;
    std::string cnt_str = ss.str();
    std::string image_path =
        (log_dir.empty() ? std::string("Log/Colmap/images/")
                         : (log_dir + "/Colmap/images/")) +
        cnt_str + ".png";

    cv::Mat img_rgb_undistort;
    pinhole_cam->undistortImage(img_rgb, img_rgb_undistort);
    cv::imwrite(image_path, img_rgb_undistort);

    Eigen::Quaterniond q(new_frame_->T_f_w_.rotation_matrix());
    Eigen::Vector3d t = new_frame_->T_f_w_.translation();
    colmap_log_ << cnt << " " << std::fixed
                << std::setprecision(6)  // 保证浮点数精度为6位
                << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
                << t.x() << " " << t.y() << " " << t.z() << " " << 1
                << " "  // CAMERA_ID (假设相机ID为1)
                << cnt_str << ".png" << std::endl;
    colmap_log_ << "0.0 0.0 -1" << std::endl;
    cnt++;
}

void VIOManager::PrepareImage(cv::Mat& img) {
    if (img.empty()) {
        if (enable_plot) {
            printf("[ VIO ] Empty Image!\n");
        }
        return;
    }
    // Always target the camera model size (avoids Frame size mismatch abort).
    if (width > 0 && height > 0 &&
        (img.cols != width || img.rows != height)) {
        cv::resize(img, img, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    } else if (image_resize_factor > 0.0 &&
               std::abs(image_resize_factor - 1.0) > 1e-6) {
        cv::resize(img, img,
                   cv::Size(static_cast<int>(img.cols * image_resize_factor),
                            static_cast<int>(img.rows * image_resize_factor)),
                   0, 0, cv::INTER_LINEAR);
    }
    img.copyTo(img_rgb);
    img.copyTo(img_cp);
    if (img.channels() == 3) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (!new_frame_) {
        new_frame_ = std::make_unique<Frame>(cam, img);
    } else {
        new_frame_->Reset(cam, img);
    }
    UpdateFrameState(*state);
    ResetGrid();
}

void VIOManager::PrintTimingStats(
    double t1, double t2, double t3, double t4, double t5, double t6, double t7,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& feat_map) {
    if (!enable_plot) {
        return;
    }
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf(
        "\033[1;34m|                         VIO Time                            |\033[0m\n");
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27zu |\033[0m\n", "Sparse Map Size",
           feat_map.size());
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
           "Time (secs)");
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n",
           "RetrieveFromVisualSparseMap", t2 - t1);
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n",
           "ComputeJacobianAndUpdateEKF", t3 - t2);
    printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> computeJacobian",
           compute_jacobian_time);
    printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> updateEKF",
           update_ekf_time);
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "GenerateVisualMapPoints",
           t4 - t3);
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "UpdateVisualMapPoints",
           t6 - t5);
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "UpdateReferencePatch",
           t7 - t6);
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Current Total Time",
           t7 - t1 - (t5 - t4));
    printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Average Total Time",
           ave_total);
    printf(
        "\033[1;34m+-------------------------------------------------------------+\033[0m\n");
}

void VIOManager::ProcessFrame(
    cv::Mat& img, std::vector<pointWithVar>& pg,
    const std::unordered_map<VoxelLocation, VoxelOctoTree*>& feat_map,
    double /*img_time*/) {
    PrepareImage(img);
    double t1 = NowSec();
    RetrieveFromVisualSparseMap(img, pg, feat_map);
    double t2 = NowSec();
    ComputeJacobianAndUpdateEKF(img);
    double t3 = NowSec();
    GenerateVisualMapPoints(img, pg);
    double t4 = NowSec();
    PlotTrackedPoints();
    if (enable_plot) {
        ProjectPatchFromRefToCur(feat_map);
    }
    double t5 = NowSec();
    UpdateVisualMapPoints(img);
    double t6 = NowSec();
    UpdateReferencePatch(feat_map);
    double t7 = NowSec();
    if (enable_colmap_output) {
        DumpDataForColmap();
    }
    frame_count++;
    ave_total = ave_total * (frame_count - 1) / frame_count +
                (t7 - t1 - (t5 - t4)) / frame_count;
    PrintTimingStats(t1, t2, t3, t4, t5, t6, t7, feat_map);
}

}  // namespace odometry
}  // namespace argus
