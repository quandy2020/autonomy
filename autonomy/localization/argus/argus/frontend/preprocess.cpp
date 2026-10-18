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

#include "argus/frontend/preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "argus/sensor/point_cloud.hpp"

namespace argus {
namespace frontend {

Preprocess::Preprocess() {
    jump_up_limit_ = std::cos(170.0 / 180.0 * M_PI);
    jump_down_limit_ = std::cos(8.0 / 180.0 * M_PI);
    cos160_ = std::cos(160.0 / 180.0 * M_PI);
    smallp_intersect_ = std::cos(172.5 / 180.0 * M_PI);
}

Preprocess::~Preprocess() = default;

void Preprocess::Set(bool enable_features, double blind_radius,
                     int filter_stride) {
    feature_enabled = enable_features;
    blind = blind_radius;
    blind_sqr = blind_radius * blind_radius;
    point_filter_num = filter_stride > 0 ? filter_stride : 1;
}

void Preprocess::Process(const PointCloud2ConstPtr& msg,
                         sensor::PointCloudXYZI::Ptr& cloud_out) {
    if (!msg || !cloud_out) {
        return;
    }
    Process(*msg, cloud_out.get());
}

void Preprocess::Process(const PointCloud2& msg,
                         sensor::PointCloudXYZI* cloud_out) {
    if (cloud_out == nullptr) {
        return;
    }
    sensor::PointCloudXYZI decoded;
    sensor::FromProto(msg, &decoded);
    Process(decoded, cloud_out);
}

void Preprocess::Process(const sensor::PointCloudXYZI& cloud_in,
                         sensor::PointCloudXYZI* cloud_out) {
    if (cloud_out == nullptr) {
        return;
    }
    cloud_surf.clear();
    cloud_corners.clear();
    cloud_full.clear();

    if (feature_enabled) {
        ExtractFeatures(cloud_in);
    } else {
        FilterCloud(cloud_in);
    }
    *cloud_out = cloud_surf;
}

void Preprocess::FilterCloud(const sensor::PointCloudXYZI& cloud_in) {
    cloud_surf.reserve(
        cloud_in.size() / static_cast<std::size_t>(point_filter_num) + 1);
    for (std::size_t i = 0; i < cloud_in.size(); ++i) {
        if (static_cast<int>(i) % point_filter_num != 0) {
            continue;
        }
        const auto& pt = cloud_in.points[i];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            continue;
        }
        const double r2 = static_cast<double>(pt.x) * pt.x +
                          static_cast<double>(pt.y) * pt.y +
                          static_cast<double>(pt.z) * pt.z;
        if (r2 < blind_sqr) {
            continue;
        }
        cloud_surf.push_back(pt);
    }
    std::sort(cloud_surf.points.begin(), cloud_surf.points.end(),
              [](const sensor::PointType& a, const sensor::PointType& b) {
                  return a.curvature < b.curvature;
              });
}

void Preprocess::ExtractFeatures(const sensor::PointCloudXYZI& cloud_in) {
    // Treat the whole cloud as one ordered scan (by azimuth). No ring / model
    // specific buffering — input must already be `sensor::PointCloudXYZI`.
    sensor::PointCloudXYZI ordered;
    ordered.reserve(cloud_in.size());
    for (std::size_t i = 0; i < cloud_in.size(); ++i) {
        const auto& pt = cloud_in.points[i];
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            continue;
        }
        const double r2 = static_cast<double>(pt.x) * pt.x +
                          static_cast<double>(pt.y) * pt.y +
                          static_cast<double>(pt.z) * pt.z;
        if (r2 < blind_sqr) {
            continue;
        }
        ordered.push_back(pt);
    }
    std::sort(ordered.points.begin(), ordered.points.end(),
              [](const sensor::PointType& a, const sensor::PointType& b) {
                  return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);
              });

    if (ordered.size() < 2) {
        cloud_surf = ordered;
        return;
    }

    std::vector<PointGeometry> types(ordered.size());
    for (std::size_t i = 0; i + 1 < ordered.size(); ++i) {
        types[i].range =
            std::sqrt(static_cast<double>(ordered[i].x) * ordered[i].x +
                      static_cast<double>(ordered[i].y) * ordered[i].y);
        vx_ = ordered[i].x - ordered[i + 1].x;
        vy_ = ordered[i].y - ordered[i + 1].y;
        vz_ = ordered[i].z - ordered[i + 1].z;
        types[i].next_dist_sq = vx_ * vx_ + vy_ * vy_ + vz_ * vz_;
    }
    const std::size_t last = ordered.size() - 1;
    types[last].range =
        std::sqrt(static_cast<double>(ordered[last].x) * ordered[last].x +
                  static_cast<double>(ordered[last].y) * ordered[last].y);
    GiveFeature(ordered, types);
}

void Preprocess::MarkPlanes(sensor::PointCloudXYZI& pl,
                            std::vector<PointGeometry>& types, unsigned int head,
                            int plsize2) {
    Eigen::Vector3d curr_direct = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_direct = Eigen::Vector3d::Zero();
    unsigned int i_nex = 0;
    unsigned int last_i = 0;
    int last_state = 0;

    for (unsigned int i = head; static_cast<int>(i) < plsize2; ++i) {
        if (types[i].range < blind_sqr) {
            continue;
        }
        const unsigned int i2 = i;
        const int plane_type = PlaneJudge(pl, types, i, i_nex, curr_direct);
        if (plane_type == 1) {
            for (unsigned int j = i; j <= i_nex; ++j) {
                types[j].feature =
                    (j != i && j != i_nex) ? kRealPlane : kPossiblePlane;
            }
            if (last_state == 1 && last_direct.norm() > 0.1) {
                const double mod = last_direct.transpose() * curr_direct;
                types[i].feature =
                    (mod > -0.707 && mod < 0.707) ? kEdgePlane : kRealPlane;
            }
            i = i_nex - 1;
            last_state = 1;
        } else {
            i = i_nex;
            last_state = 0;
        }
        last_i = i2;
        last_direct = curr_direct;
        (void)last_i;
    }
}

void Preprocess::MarkEdgeJumps(sensor::PointCloudXYZI& pl,
                               std::vector<PointGeometry>& types, unsigned int head,
                               int plsize2) {
    for (unsigned int i = head + 3; static_cast<int>(i) < plsize2; ++i) {
        if (types[i].range < blind_sqr || types[i].feature >= kRealPlane) {
            continue;
        }
        if (types[i - 1].next_dist_sq < 1e-16 || types[i].next_dist_sq < 1e-16) {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
        Eigen::Vector3d vecs[2];
        for (int j = 0; j < 2; ++j) {
            const int m = (j == 1) ? 1 : -1;
            if (types[i + m].range < blind_sqr) {
                types[i].edge_jump[j] =
                    (types[i].range > inf_bound_) ? kJumpInf : kJumpBlind;
                continue;
            }
            vecs[j] =
                Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z) - vec_a;
            types[i].angle[j] =
                vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
            if (types[i].angle[j] < jump_up_limit_) {
                types[i].edge_jump[j] = kJump180;
            } else if (types[i].angle[j] > jump_down_limit_) {
                types[i].edge_jump[j] = kJumpZero;
            }
        }

        types[i].intersect =
            vecs[kPrev].dot(vecs[kNext]) / vecs[kPrev].norm() / vecs[kNext].norm();
        if (types[i].edge_jump[kPrev] == kJumpNormal && types[i].edge_jump[kNext] == kJumpZero &&
            types[i].next_dist_sq > 0.0225 &&
            types[i].next_dist_sq > 4 * types[i - 1].next_dist_sq) {
            if (types[i].intersect > cos160_ &&
                EdgeJumpJudge(pl, types, i, kPrev)) {
                types[i].feature = kEdgeJump;
            }
        } else if (types[i].edge_jump[kPrev] == kJumpZero &&
                   types[i].edge_jump[kNext] == kJumpNormal &&
                   types[i - 1].next_dist_sq > 0.0225 &&
                   types[i - 1].next_dist_sq > 4 * types[i].next_dist_sq) {
            if (types[i].intersect > cos160_ &&
                EdgeJumpJudge(pl, types, i, kNext)) {
                types[i].feature = kEdgeJump;
            }
        } else if (types[i].edge_jump[kPrev] == kJumpNormal &&
                   types[i].edge_jump[kNext] == kJumpInf) {
            if (EdgeJumpJudge(pl, types, i, kPrev)) {
                types[i].feature = kEdgeJump;
            }
        } else if (types[i].edge_jump[kPrev] == kJumpInf &&
                   types[i].edge_jump[kNext] == kJumpNormal) {
            if (EdgeJumpJudge(pl, types, i, kNext)) {
                types[i].feature = kEdgeJump;
            }
        } else if (types[i].edge_jump[kPrev] > kJumpNormal &&
                   types[i].edge_jump[kNext] > kJumpNormal) {
            if (types[i].feature == kNormal) {
                types[i].feature = kWire;
            }
        }
    }
}

void Preprocess::MarkSmallPlanes(std::vector<PointGeometry>& types, unsigned int head,
                                 int plsize2) {
    for (unsigned int i = head + 1; static_cast<int>(i) < plsize2; ++i) {
        if (types[i].range < blind_sqr || types[i - 1].range < blind_sqr ||
            types[i + 1].range < blind_sqr) {
            continue;
        }
        if (types[i - 1].next_dist_sq < 1e-8 || types[i].next_dist_sq < 1e-8) {
            continue;
        }
        if (types[i].feature == kNormal) {
            const double ratio = (types[i - 1].next_dist_sq > types[i].next_dist_sq)
                                     ? types[i - 1].next_dist_sq / types[i].next_dist_sq
                                     : types[i].next_dist_sq / types[i - 1].next_dist_sq;
            if (types[i].intersect < smallp_intersect_ &&
                ratio < smallp_ratio_) {
                if (types[i - 1].feature == kNormal) {
                    types[i - 1].feature = kRealPlane;
                }
                if (types[i + 1].feature == kNormal) {
                    types[i + 1].feature = kRealPlane;
                }
                types[i].feature = kRealPlane;
            }
        }
    }
}

void Preprocess::EmitSurfAndCorn(sensor::PointCloudXYZI& pl,
                                 std::vector<PointGeometry>& types, unsigned int head,
                                 int plsize) {
    int last_surface = -1;
    for (unsigned int j = head; static_cast<int>(j) < plsize; ++j) {
        if (types[j].feature == kPossiblePlane || types[j].feature == kRealPlane) {
            if (last_surface == -1) {
                last_surface = static_cast<int>(j);
            }
            if (static_cast<int>(j) == last_surface + point_filter_num - 1) {
                sensor::PointType ap = pl[j];
                cloud_surf.push_back(ap);
                last_surface = -1;
            }
        } else {
            if (types[j].feature == kEdgeJump || types[j].feature == kEdgePlane) {
                cloud_corners.push_back(pl[j]);
            }
            if (last_surface != -1) {
                sensor::PointType ap{};
                for (unsigned int k = static_cast<unsigned int>(last_surface);
                     k < j; ++k) {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.curvature += pl[k].curvature;
                }
                const float n = static_cast<float>(j - last_surface);
                ap.x /= n;
                ap.y /= n;
                ap.z /= n;
                ap.curvature /= n;
                cloud_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void Preprocess::GiveFeature(sensor::PointCloudXYZI& pl,
                             std::vector<PointGeometry>& types) {
    const int plsize = static_cast<int>(pl.size());
    if (plsize == 0) {
        return;
    }

    unsigned int head = 0;
    while (head < static_cast<unsigned int>(plsize) &&
           types[head].range < blind_sqr) {
        ++head;
    }

    const int plsize_plane =
        (plsize > group_size_) ? (plsize - group_size_) : 0;
    MarkPlanes(pl, types, head, plsize_plane);
    MarkEdgeJumps(pl, types, head, plsize > 3 ? plsize - 3 : 0);
    MarkSmallPlanes(types, head, plsize - 1);
    EmitSurfAndCorn(pl, types, head, plsize);
}

int Preprocess::CollectPlaneGroup(const sensor::PointCloudXYZI& pl,
                                  std::vector<PointGeometry>& types,
                                  unsigned int i_cur, unsigned int& i_nex,
                                  double group_dis,
                                  std::vector<double>* disarr,
                                  double* two_dis) {
    *two_dis = 0;
    for (i_nex = i_cur; i_nex < i_cur + static_cast<unsigned int>(group_size_);
         ++i_nex) {
        if (types[i_nex].range < blind_sqr) {
            return 2;
        }
        disarr->push_back(types[i_nex].next_dist_sq);
    }

    for (;;) {
        if (i_cur >= pl.size() || i_nex >= pl.size()) {
            break;
        }
        if (types[i_nex].range < blind_sqr) {
            return 2;
        }
        vx_ = pl[i_nex].x - pl[i_cur].x;
        vy_ = pl[i_nex].y - pl[i_cur].y;
        vz_ = pl[i_nex].z - pl[i_cur].z;
        *two_dis = vx_ * vx_ + vy_ * vy_ + vz_ * vz_;
        if (*two_dis >= group_dis) {
            break;
        }
        disarr->push_back(types[i_nex].next_dist_sq);
        ++i_nex;
    }
    return 0;
}

int Preprocess::PlaneJudge(const sensor::PointCloudXYZI& pl,
                           std::vector<PointGeometry>& types, unsigned int i_cur,
                           unsigned int& i_nex, Eigen::Vector3d& curr_direct) {
    double group_dis = dis_a_ * types[i_cur].range + dis_b_;
    group_dis = group_dis * group_dis;

    std::vector<double> disarr;
    disarr.reserve(20);

    double two_dis = 0;
    const int collect_rc = CollectPlaneGroup(pl, types, i_cur, i_nex, group_dis,
                                             &disarr, &two_dis);
    if (collect_rc != 0) {
        curr_direct.setZero();
        return collect_rc;
    }

    double leng_wid = 0;
    for (unsigned int j = i_cur + 1; j < i_nex; ++j) {
        if (j >= pl.size() || i_cur >= pl.size()) {
            break;
        }
        const double v1[3] = {pl[j].x - pl[i_cur].x, pl[j].y - pl[i_cur].y,
                              pl[j].z - pl[i_cur].z};
        const double v2[3] = {v1[1] * vz_ - vy_ * v1[2],
                              v1[2] * vx_ - v1[0] * vz_,
                              v1[0] * vy_ - vx_ * v1[1]};
        const double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid) {
            leng_wid = lw;
        }
    }

    if ((two_dis * two_dis / leng_wid) < p2l_ratio_) {
        curr_direct.setZero();
        return 0;
    }

    const unsigned int disarrsize = static_cast<unsigned int>(disarr.size());
    for (unsigned int j = 0; j + 1 < disarrsize; ++j) {
        for (unsigned int k = j + 1; k < disarrsize; ++k) {
            if (disarr[j] < disarr[k]) {
                std::swap(disarr[j], disarr[k]);
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16) {
        curr_direct.setZero();
        return 0;
    }

    const double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin_) {
        curr_direct.setZero();
        return 0;
    }

    curr_direct << vx_, vy_, vz_;
    curr_direct.normalize();
    return 1;
}

bool Preprocess::EdgeJumpJudge(const sensor::PointCloudXYZI& /*pl*/,
                               std::vector<PointGeometry>& types, unsigned int i,
                               Surround nor_dir) {
    if (nor_dir == kPrev) {
        if (types[i - 1].range < blind_sqr || types[i - 2].range < blind_sqr) {
            return false;
        }
    } else if (nor_dir == kNext) {
        if (types[i + 1].range < blind_sqr || types[i + 2].range < blind_sqr) {
            return false;
        }
    }
    double d1 = types[i + nor_dir - 1].next_dist_sq;
    double d2 = types[i + 3 * nor_dir - 2].next_dist_sq;
    if (d1 < d2) {
        std::swap(d1, d2);
    }
    d1 = std::sqrt(d1);
    d2 = std::sqrt(d2);
    if (d1 > edge_a_ * d2 || (d1 - d2) > edge_b_) {
        return false;
    }
    return true;
}

}  // namespace frontend
}  // namespace argus
