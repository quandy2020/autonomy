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

#include "autonomy/localization/atlas/map/local_grid_maker.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/camera/fisheye.hpp"
#include "autonomy/localization/atlas/camera/perspective.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {
namespace {

bool GetPinhole(camera::base* cam, double* fx, double* fy, double* cx,
                double* cy) {
    if (cam == nullptr || fx == nullptr || fy == nullptr || cx == nullptr ||
        cy == nullptr) {
        return false;
    }
    if (auto* p = dynamic_cast<camera::perspective*>(cam)) {
        *fx = p->fx_;
        *fy = p->fy_;
        *cx = p->cx_;
        *cy = p->cy_;
        return *fx > 1e-6 && *fy > 1e-6;
    }
    if (auto* p = dynamic_cast<camera::fisheye*>(cam)) {
        *fx = p->fx_;
        *fy = p->fy_;
        *cx = p->cx_;
        *cy = p->cy_;
        return *fx > 1e-6 && *fy > 1e-6;
    }
    return false;
}

uint32_t PackRgb(const cv::Mat& rgb, int r, int c) {
    if (rgb.empty() || rgb.type() != CV_8UC3) {
        return 0;
    }
    if (r < 0 || c < 0 || r >= rgb.rows || c >= rgb.cols) {
        return 0;
    }
    const cv::Vec3b px = rgb.at<cv::Vec3b>(r, c);
    const uint32_t R = px[0];
    const uint32_t G = px[1];
    const uint32_t B = px[2];
    return (R << 16) | (G << 8) | B;
}

struct VoxelKey {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    bool operator==(const VoxelKey& o) const {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        return (static_cast<std::size_t>(k.ix) * 73856093u) ^
               (static_cast<std::size_t>(k.iy) * 19349663u) ^
               (static_cast<std::size_t>(k.iz) * 83492791u);
    }
};

using VoxelMap = std::unordered_map<VoxelKey, Cell3D, VoxelKeyHash>;

void InsertVoxel(VoxelMap* voxels, float x, float y, float z, uint32_t rgb,
                 float cell) {
    if (voxels == nullptr || cell <= 1e-6f) {
        return;
    }
    VoxelKey key;
    key.ix = static_cast<int>(std::floor(x / cell));
    key.iy = static_cast<int>(std::floor(y / cell));
    key.iz = static_cast<int>(std::floor(z / cell));
    auto& cell_ref = (*voxels)[key];
    if (cell_ref.rgb == 0 && rgb != 0) {
        cell_ref.rgb = rgb;
    }
    cell_ref.x = (static_cast<float>(key.ix) + 0.5f) * cell;
    cell_ref.y = (static_cast<float>(key.iy) + 0.5f) * cell;
    cell_ref.z = (static_cast<float>(key.iz) + 0.5f) * cell;
}

std::vector<Cell3D> FlattenVoxels(const VoxelMap& voxels, int max_cells) {
    std::vector<Cell3D> out;
    out.reserve(std::min(static_cast<int>(voxels.size()), max_cells));
    for (const auto& kv : voxels) {
        out.push_back(kv.second);
        if (static_cast<int>(out.size()) >= max_cells) {
            break;
        }
    }
    return out;
}

bool InFootprint(float x, float y, float z, float length, float width,
                 float height) {
    if (length <= 0.f || width <= 0.f) {
        return false;
    }
    const float half_l = 0.5f * length;
    const float half_w = 0.5f * width;
    if (std::fabs(x) > half_l || std::fabs(y) > half_w) {
        return false;
    }
    if (height > 0.f && (z < -height || z > 0.05f)) {
        return false;
    }
    return true;
}

VoxelMap FilterNoise(const VoxelMap& input, int radius, int min_neighbors) {
    if (radius <= 0 || min_neighbors <= 0 || input.empty()) {
        return input;
    }
    VoxelMap kept;
    kept.reserve(input.size());
    for (const auto& kv : input) {
        int neighbors = 0;
        for (int dz = -radius; dz <= radius; ++dz) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    VoxelKey n{kv.first.ix + dx, kv.first.iy + dy,
                               kv.first.iz + dz};
                    if (input.find(n) != input.end()) {
                        ++neighbors;
                    }
                }
            }
        }
        if (neighbors >= min_neighbors) {
            kept.emplace(kv);
        }
    }
    return kept;
}

void RayTraceEmpty(const std::vector<Cell3D>& obstacles,
                   const Eigen::Vector3f& view, float cell, int max_cells,
                   std::vector<Cell3D>* empty_out) {
    if (empty_out == nullptr || cell <= 1e-6f) {
        return;
    }
    VoxelMap empty_voxels;
    const int vx0 = static_cast<int>(std::floor(view.x() / cell));
    const int vy0 = static_cast<int>(std::floor(view.y() / cell));
    for (const auto& obs : obstacles) {
        int x1 = static_cast<int>(std::floor(obs.x / cell));
        int y1 = static_cast<int>(std::floor(obs.y / cell));
        int x0 = vx0;
        int y0 = vy0;
        const int dx = std::abs(x1 - x0);
        const int dy = std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (!(x0 == x1 && y0 == y1)) {
            const int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
            if (x0 == x1 && y0 == y1) {
                break;
            }
            InsertVoxel(&empty_voxels, (x0 + 0.5f) * cell, (y0 + 0.5f) * cell,
                        view.z(), 0, cell);
            if (static_cast<int>(empty_voxels.size()) >= max_cells) {
                break;
            }
        }
        if (static_cast<int>(empty_voxels.size()) >= max_cells) {
            break;
        }
    }
    *empty_out = FlattenVoxels(empty_voxels, max_cells);
}

struct RawPoint {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    uint32_t rgb = 0;
};

/** PCA normal from up to K nearest neighbors in a radius (RTAB-Map style). */
bool EstimateNormal(const std::vector<RawPoint>& pts,
                    const std::vector<int>& neighbor_idx, Eigen::Vector3f* n) {
    if (n == nullptr || neighbor_idx.size() < 3) {
        return false;
    }
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i : neighbor_idx) {
        mean += Eigen::Vector3f(pts[static_cast<std::size_t>(i)].x,
                                pts[static_cast<std::size_t>(i)].y,
                                pts[static_cast<std::size_t>(i)].z);
    }
    mean /= static_cast<float>(neighbor_idx.size());
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int i : neighbor_idx) {
        const Eigen::Vector3f d =
            Eigen::Vector3f(pts[static_cast<std::size_t>(i)].x,
                            pts[static_cast<std::size_t>(i)].y,
                            pts[static_cast<std::size_t>(i)].z) -
            mean;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    *n = solver.eigenvectors().col(0);
    if (n->z() < 0.f) {
        *n = -(*n);
    }
    return n->norm() > 1e-6f;
}

void SegmentByNormals(const std::vector<RawPoint>& pts,
                      const LocalGridMaker::Options& opt, VoxelMap* ground,
                      VoxelMap* obstacles) {
    if (ground == nullptr || obstacles == nullptr || pts.empty()) {
        return;
    }
    const float cell = std::max(opt.cell_size, 1e-3f);
    const float search_r = std::max(opt.cluster_radius, cell);
    const float search_r2 = search_r * search_r;
    const float cos_thr =
        std::cos(opt.max_ground_angle_deg * static_cast<float>(M_PI) / 180.f);

    using IndexMap = std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash>;
    IndexMap buckets;
    buckets.reserve(pts.size());
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        VoxelKey key;
        key.ix = static_cast<int>(std::floor(pts[static_cast<std::size_t>(i)].x /
                                             cell));
        key.iy = static_cast<int>(std::floor(pts[static_cast<std::size_t>(i)].y /
                                             cell));
        key.iz = static_cast<int>(std::floor(pts[static_cast<std::size_t>(i)].z /
                                             cell));
        buckets[key].push_back(i);
    }

    std::vector<char> is_flat(pts.size(), 0);
    const int k_need = std::max(3, opt.normal_k);
    const int nbr = std::max(1, static_cast<int>(std::ceil(search_r / cell)));

    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const auto& p = pts[static_cast<std::size_t>(i)];
        const int ix = static_cast<int>(std::floor(p.x / cell));
        const int iy = static_cast<int>(std::floor(p.y / cell));
        const int iz = static_cast<int>(std::floor(p.z / cell));
        std::vector<std::pair<float, int>> cand;
        cand.reserve(static_cast<std::size_t>(k_need) * 2);
        for (int dz = -nbr; dz <= nbr; ++dz) {
            for (int dy = -nbr; dy <= nbr; ++dy) {
                for (int dx = -nbr; dx <= nbr; ++dx) {
                    VoxelKey key{ix + dx, iy + dy, iz + dz};
                    auto it = buckets.find(key);
                    if (it == buckets.end()) {
                        continue;
                    }
                    for (int j : it->second) {
                        const auto& q = pts[static_cast<std::size_t>(j)];
                        const float d2 =
                            (p.x - q.x) * (p.x - q.x) +
                            (p.y - q.y) * (p.y - q.y) +
                            (p.z - q.z) * (p.z - q.z);
                        if (d2 <= search_r2) {
                            cand.emplace_back(d2, j);
                        }
                    }
                }
            }
        }
        if (static_cast<int>(cand.size()) < 3) {
            continue;
        }
        if (static_cast<int>(cand.size()) > k_need) {
            std::partial_sort(cand.begin(), cand.begin() + k_need, cand.end());
            cand.resize(static_cast<std::size_t>(k_need));
        }
        std::vector<int> nn;
        nn.reserve(cand.size());
        for (const auto& c : cand) {
            nn.push_back(c.second);
        }
        Eigen::Vector3f n;
        if (!EstimateNormal(pts, nn, &n)) {
            continue;
        }
        if (n.z() >= cos_thr) {
            is_flat[static_cast<std::size_t>(i)] = 1;
        }
    }

    // Euclidean clustering of flat points.
    std::vector<int> flat_idx;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        if (is_flat[static_cast<std::size_t>(i)]) {
            flat_idx.push_back(i);
        }
    }
    IndexMap flat_buckets;
    for (int i : flat_idx) {
        const auto& p = pts[static_cast<std::size_t>(i)];
        VoxelKey key;
        key.ix = static_cast<int>(std::floor(p.x / cell));
        key.iy = static_cast<int>(std::floor(p.y / cell));
        key.iz = static_cast<int>(std::floor(p.z / cell));
        flat_buckets[key].push_back(i);
    }

    std::vector<char> visited(pts.size(), 0);
    std::vector<std::vector<int>> clusters;
    const int c_nbr =
        std::max(1, static_cast<int>(std::ceil(opt.cluster_radius / cell)));
    const float c_r2 = opt.cluster_radius * opt.cluster_radius;

    for (int seed : flat_idx) {
        if (visited[static_cast<std::size_t>(seed)]) {
            continue;
        }
        std::vector<int> cluster;
        std::queue<int> q;
        q.push(seed);
        visited[static_cast<std::size_t>(seed)] = 1;
        while (!q.empty()) {
            const int cur = q.front();
            q.pop();
            cluster.push_back(cur);
            const auto& p = pts[static_cast<std::size_t>(cur)];
            const int ix = static_cast<int>(std::floor(p.x / cell));
            const int iy = static_cast<int>(std::floor(p.y / cell));
            const int iz = static_cast<int>(std::floor(p.z / cell));
            for (int dz = -c_nbr; dz <= c_nbr; ++dz) {
                for (int dy = -c_nbr; dy <= c_nbr; ++dy) {
                    for (int dx = -c_nbr; dx <= c_nbr; ++dx) {
                        auto it = flat_buckets.find(
                            VoxelKey{ix + dx, iy + dy, iz + dz});
                        if (it == flat_buckets.end()) {
                            continue;
                        }
                        for (int j : it->second) {
                            if (visited[static_cast<std::size_t>(j)]) {
                                continue;
                            }
                            const auto& qq = pts[static_cast<std::size_t>(j)];
                            const float d2 =
                                (p.x - qq.x) * (p.x - qq.x) +
                                (p.y - qq.y) * (p.y - qq.y) +
                                (p.z - qq.z) * (p.z - qq.z);
                            if (d2 <= c_r2) {
                                visited[static_cast<std::size_t>(j)] = 1;
                                q.push(j);
                            }
                        }
                    }
                }
            }
        }
        if (static_cast<int>(cluster.size()) >= opt.min_cluster_size) {
            clusters.push_back(std::move(cluster));
        }
    }

    if (clusters.empty()) {
        for (const auto& p : pts) {
            InsertVoxel(obstacles, p.x, p.y, p.z, p.rgb, cell);
        }
        return;
    }

    // Pick main ground: largest cluster with centroid under max_ground_height
    // (or just largest if height band unused).
    int best = -1;
    std::size_t best_size = 0;
    for (int ci = 0; ci < static_cast<int>(clusters.size()); ++ci) {
        float cz = 0.f;
        for (int i : clusters[static_cast<std::size_t>(ci)]) {
            cz += pts[static_cast<std::size_t>(i)].z;
        }
        cz /= static_cast<float>(clusters[static_cast<std::size_t>(ci)].size());
        if (cz > opt.max_ground_height) {
            continue;
        }
        if (clusters[static_cast<std::size_t>(ci)].size() > best_size) {
            best_size = clusters[static_cast<std::size_t>(ci)].size();
            best = ci;
        }
    }
    if (best < 0) {
        for (int ci = 0; ci < static_cast<int>(clusters.size()); ++ci) {
            if (clusters[static_cast<std::size_t>(ci)].size() > best_size) {
                best_size = clusters[static_cast<std::size_t>(ci)].size();
                best = ci;
            }
        }
    }
    if (best < 0) {
        for (const auto& p : pts) {
            InsertVoxel(obstacles, p.x, p.y, p.z, p.rgb, cell);
        }
        return;
    }

    float ground_z = 0.f;
    for (int i : clusters[static_cast<std::size_t>(best)]) {
        ground_z += pts[static_cast<std::size_t>(i)].z;
    }
    ground_z /=
        static_cast<float>(clusters[static_cast<std::size_t>(best)].size());

    std::vector<char> is_ground(pts.size(), 0);
    for (int i : clusters[static_cast<std::size_t>(best)]) {
        is_ground[static_cast<std::size_t>(i)] = 1;
    }
    // Merge other flat clusters near ground height.
    for (int ci = 0; ci < static_cast<int>(clusters.size()); ++ci) {
        if (ci == best) {
            continue;
        }
        float cz = 0.f;
        for (int i : clusters[static_cast<std::size_t>(ci)]) {
            cz += pts[static_cast<std::size_t>(i)].z;
        }
        cz /= static_cast<float>(clusters[static_cast<std::size_t>(ci)].size());
        const bool near_ground =
            std::fabs(cz - ground_z) <= std::max(2.f * cell, 0.15f) ||
            cz <= opt.max_ground_height;
        if (near_ground) {
            for (int i : clusters[static_cast<std::size_t>(ci)]) {
                is_ground[static_cast<std::size_t>(i)] = 1;
            }
        } else if (opt.flat_obstacles) {
            for (int i : clusters[static_cast<std::size_t>(ci)]) {
                const auto& p = pts[static_cast<std::size_t>(i)];
                InsertVoxel(obstacles, p.x, p.y, p.z, p.rgb, cell);
            }
        }
    }

    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const auto& p = pts[static_cast<std::size_t>(i)];
        if (is_ground[static_cast<std::size_t>(i)]) {
            InsertVoxel(ground, p.x, p.y, p.z, p.rgb, cell);
        } else if (!is_flat[static_cast<std::size_t>(i)]) {
            InsertVoxel(obstacles, p.x, p.y, p.z, p.rgb, cell);
        } else if (!opt.flat_obstacles) {
            InsertVoxel(obstacles, p.x, p.y, p.z, p.rgb, cell);
        }
    }
}

}  // namespace

Mat33_t OpticalToLinkR() {
    Mat33_t R;
    R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    return R;
}

Mat44_t OpencvPoseToRosMapOptical(const Mat44_t& T_wc_opencv) {
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = OpticalToLinkR();
    return T * T_wc_opencv;
}

Mat44_t OpencvPoseToRosCameraLink(const Mat44_t& T_wc_opencv) {
    const Mat44_t T_map_optical = OpencvPoseToRosMapOptical(T_wc_opencv);
    Mat44_t T_link_optical = Mat44_t::Identity();
    T_link_optical.block<3, 3>(0, 0) = OpticalToLinkR();
    return T_map_optical * T_link_optical.inverse();
}

LocalGridMaker::LocalGridMaker(Options options)
    : options_(std::move(options)) {
    if (options_.cell_size <= 1e-4f) {
        options_.cell_size = 0.05f;
    }
    if (options_.depth_decimation < 1) {
        options_.depth_decimation = 1;
    }
    if (options_.normal_k < 3) {
        options_.normal_k = 3;
    }
    if (options_.min_cluster_size < 1) {
        options_.min_cluster_size = 1;
    }
}

LocalGrid LocalGridMaker::Create(const cv::Mat& rgb, const cv::Mat& depth_m,
                                 const Mat44_t& T_wc_opencv, camera::base* camera,
                                 double timestamp_sec, uint64_t id) const {
    LocalGrid grid;
    grid.cell_size = options_.cell_size;
    grid.T_wc_opencv = T_wc_opencv;
    grid.timestamp_sec = timestamp_sec;
    grid.id = id;

    if (depth_m.empty() || depth_m.type() != CV_32FC1 || camera == nullptr) {
        return grid;
    }

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    if (!GetPinhole(camera, &fx, &fy, &cx, &cy)) {
        AWARN << "LocalGridMaker: unsupported camera model.";
        return grid;
    }

    const Mat33_t R_lo = OpticalToLinkR();
    VoxelMap ground_voxels;
    VoxelMap obstacle_voxels;
    const int step = options_.depth_decimation;
    std::vector<RawPoint> raw;
    if (options_.normals_segmentation && !options_.ground_is_obstacle) {
        raw.reserve(static_cast<std::size_t>(
            (depth_m.rows / step + 1) * (depth_m.cols / step + 1)));
    }

    for (int v = 0; v < depth_m.rows; v += step) {
        const float* row = depth_m.ptr<float>(v);
        for (int u = 0; u < depth_m.cols; u += step) {
            const float d = row[u];
            if (!std::isfinite(d) || d < options_.range_min ||
                d > options_.range_max) {
                continue;
            }
            const Eigen::Vector3d p_opt((static_cast<double>(u) - cx) / fx * d,
                                       (static_cast<double>(v) - cy) / fy * d,
                                       static_cast<double>(d));
            const Eigen::Vector3d p_link = R_lo * p_opt;
            const float x = static_cast<float>(p_link.x());
            const float y = static_cast<float>(p_link.y());
            const float z = static_cast<float>(p_link.z());
            if (z < options_.min_ground_height ||
                z > options_.max_obstacle_height) {
                continue;
            }
            if (InFootprint(x, y, z, options_.footprint_length,
                            options_.footprint_width,
                            options_.footprint_height)) {
                continue;
            }
            const uint32_t rgb_packed =
                options_.store_rgb ? PackRgb(rgb, v, u) : 0;
            if (options_.normals_segmentation && !options_.ground_is_obstacle) {
                raw.push_back(RawPoint{x, y, z, rgb_packed});
            } else if (z <= options_.max_ground_height) {
                InsertVoxel(&ground_voxels, x, y, z, rgb_packed,
                            options_.cell_size);
            } else {
                InsertVoxel(&obstacle_voxels, x, y, z, rgb_packed,
                            options_.cell_size);
            }
        }
    }

    if (options_.normals_segmentation && !options_.ground_is_obstacle) {
        SegmentByNormals(raw, options_, &ground_voxels, &obstacle_voxels);
    }

    ground_voxels = FilterNoise(ground_voxels, options_.noise_filtering_radius,
                                options_.noise_filtering_min_neighbors);
    obstacle_voxels =
        FilterNoise(obstacle_voxels, options_.noise_filtering_radius,
                    options_.noise_filtering_min_neighbors);

    if (options_.ground_is_obstacle) {
        for (const auto& kv : ground_voxels) {
            obstacle_voxels[kv.first] = kv.second;
        }
        ground_voxels.clear();
    }

    grid.ground =
        FlattenVoxels(ground_voxels, options_.max_cells_per_layer);
    grid.obstacles =
        FlattenVoxels(obstacle_voxels, options_.max_cells_per_layer);
    grid.view_point = Eigen::Vector3f::Zero();

    if (options_.ray_tracing && !grid.obstacles.empty()) {
        RayTraceEmpty(grid.obstacles, grid.view_point, options_.cell_size,
                      options_.max_cells_per_layer, &grid.empty);
    }
    return grid;
}

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
