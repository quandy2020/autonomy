/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/loop_closure/lidar/triangle_descriptor/binary_extractor.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "common/math/kdtree.hpp"

namespace atlas {
namespace vxslam {
namespace loop_closure {
namespace triangle {
namespace {

void MergeGroup(const std::vector<TrianglePlane>& origin, int group_id,
                TrianglePlane* out) {
  bool first = true;
  for (const TrianglePlane& p : origin) {
    if (p.id != group_id) {
      continue;
    }
    if (first) {
      *out = p;
      first = false;
      continue;
    }
    const Eigen::Matrix3d P_PT1 =
        (out->covariance + out->center * out->center.transpose()) *
        out->points_size;
    const Eigen::Matrix3d P_PT2 =
        (p.covariance + p.center * p.center.transpose()) * p.points_size;
    const Eigen::Vector3d merge_center =
        (out->center * out->points_size + p.center * p.points_size) /
        (out->points_size + p.points_size);
    out->covariance = (P_PT1 + P_PT2) / (out->points_size + p.points_size) -
                      merge_center * merge_center.transpose();
    out->center = merge_center;
    out->points_size += p.points_size;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(out->covariance);
    out->normal = saes.eigenvectors().col(0);
    out->radius = static_cast<float>(std::sqrt(saes.eigenvalues()[2]));
    out->d = static_cast<float>(-out->normal.dot(out->center));
    out->min_eigen_value = static_cast<float>(saes.eigenvalues()[0]);
  }
}

void AssignMergeIds(std::vector<TrianglePlane>* planes,
                    const TriangleConfig& cfg) {
  for (TrianglePlane& p : *planes) {
    p.id = 0;
  }
  int current_id = 1;
  for (int i = static_cast<int>(planes->size()) - 1; i > 0; --i) {
    for (int j = 0; j < i; ++j) {
      TrianglePlane& a = (*planes)[static_cast<size_t>(i)];
      TrianglePlane& b = (*planes)[static_cast<size_t>(j)];
      const Eigen::Vector3d normal_diff = a.normal - b.normal;
      const Eigen::Vector3d normal_add = a.normal + b.normal;
      const double dis1 = std::fabs(a.normal.dot(b.center) + a.d);
      const double dis2 = std::fabs(b.normal.dot(a.center) + b.d);
      if ((normal_diff.norm() < cfg.plane_merge_normal_thre ||
           normal_add.norm() < cfg.plane_merge_normal_thre) &&
          dis1 < cfg.plane_merge_dis_thre && dis2 < cfg.plane_merge_dis_thre) {
        if (a.id == 0 && b.id == 0) {
          a.id = current_id;
          b.id = current_id;
          ++current_id;
        } else if (a.id == 0 && b.id != 0) {
          a.id = b.id;
        } else if (a.id != 0 && b.id == 0) {
          b.id = a.id;
        }
      }
    }
  }
}

}  // namespace

void MergePlanes(const std::vector<TrianglePlane>& origin,
                 const TriangleConfig& cfg,
                 std::vector<TrianglePlane>* merged) {
  if (merged == nullptr) {
    return;
  }
  merged->clear();
  if (origin.empty()) {
    return;
  }
  if (origin.size() == 1) {
    *merged = origin;
    return;
  }
  std::vector<TrianglePlane> planes = origin;
  AssignMergeIds(&planes, cfg);
  std::vector<int> seen;
  for (size_t i = 0; i < planes.size(); ++i) {
    if (planes[i].id == 0) {
      merged->push_back(planes[i]);
      continue;
    }
    if (std::find(seen.begin(), seen.end(), planes[i].id) != seen.end()) {
      continue;
    }
    bool has_peer = false;
    for (size_t j = 0; j < planes.size(); ++j) {
      if (i != j && planes[j].id == planes[i].id) {
        has_peer = true;
        break;
      }
    }
    if (!has_peer) {
      continue;
    }
    TrianglePlane m;
    MergeGroup(planes, planes[i].id, &m);
    seen.push_back(planes[i].id);
    merged->push_back(m);
  }
}

void ExtractBinary(const Eigen::Vector3d& project_center,
                   const Eigen::Vector3d& project_normal,
                   const sensor::PointCloudXYZI& cloud,
                   const TriangleConfig& cfg,
                   std::vector<BinaryDescriptor>* binary_list) {
  if (binary_list == nullptr) {
    return;
  }
  binary_list->clear();
  const double binary_min_dis = cfg.summary_min_thre;
  const double resolution = cfg.proj_image_resolution;
  const double dis_threshold_min = cfg.proj_dis_min;
  const double dis_threshold_max = cfg.proj_dis_max;
  const double high_inc = cfg.proj_image_high_inc;
  const bool line_filter_enable = cfg.line_filter_enable != 0;
  const double A = project_normal[0];
  const double B = project_normal[1];
  const double C = project_normal[2];
  const double D =
      -(A * project_center[0] + B * project_center[1] + C * project_center[2]);

  Eigen::Vector3d x_axis(1, 1, 0);
  if (C != 0) {
    x_axis[2] = -(A + B) / C;
  } else if (B != 0) {
    x_axis[1] = -A / B;
  } else {
    x_axis[0] = 0;
    x_axis[1] = 1;
  }
  x_axis.normalize();
  Eigen::Vector3d y_axis = project_normal.cross(x_axis);
  y_axis.normalize();
  const double ax = x_axis[0], bx = x_axis[1], cx = x_axis[2];
  const double dx = -(ax * project_center[0] + bx * project_center[1] +
                      cx * project_center[2]);
  const double ay = y_axis[0], by = y_axis[1], cy = y_axis[2];
  const double dy = -(ay * project_center[0] + by * project_center[1] +
                      cy * project_center[2]);

  std::vector<Eigen::Vector2d> point_list_2d;
  std::vector<double> dis_list_2d;
  const double denom = A * A + B * B + C * C;
  for (const auto& pt : cloud.points) {
    const double x = pt.x, y = pt.y, z = pt.z;
    const double dis = std::fabs(x * A + y * B + z * C + D);
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    }
    Eigen::Vector3d cur_project;
    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C)) / denom;
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C)) / denom;
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B)) / denom;
    const double project_x =
        cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    const double project_y =
        cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;
    point_list_2d.emplace_back(project_x, project_y);
    dis_list_2d.push_back(dis);
  }
  if (point_list_2d.size() <= 5) {
    return;
  }
  double min_x = point_list_2d[0][0], max_x = min_x;
  double min_y = point_list_2d[0][1], max_y = min_y;
  for (const auto& pi : point_list_2d) {
    min_x = std::min(min_x, pi[0]);
    max_x = std::max(max_x, pi[0]);
    min_y = std::min(min_y, pi[1]);
    max_y = std::max(max_y, pi[1]);
  }
  constexpr int kSegBase = 5;
  const double segmen_len = kSegBase * resolution;
  const int x_segment_num = static_cast<int>((max_x - min_x) / segmen_len) + 1;
  const int y_segment_num = static_cast<int>((max_y - min_y) / segmen_len) + 1;
  const int x_axis_len =
      static_cast<int>((max_x - min_x) / resolution + kSegBase);
  const int y_axis_len =
      static_cast<int>((max_y - min_y) / resolution + kSegBase);
  if (x_axis_len <= 0 || y_axis_len <= 0 || x_axis_len > 2000 ||
      y_axis_len > 2000) {
    return;
  }

  const size_t nx = static_cast<size_t>(x_axis_len);
  const size_t ny = static_cast<size_t>(y_axis_len);
  std::vector<std::vector<std::vector<double>>> dis_container(
      nx, std::vector<std::vector<double>>(ny));
  std::vector<std::vector<BinaryDescriptor>> binary_container(
      nx, std::vector<BinaryDescriptor>(ny));
  std::vector<std::vector<double>> img_count(nx, std::vector<double>(ny, 0));
  std::vector<std::vector<double>> dis_array(nx, std::vector<double>(ny, 0));
  std::vector<std::vector<double>> mean_x(nx, std::vector<double>(ny, 0));
  std::vector<std::vector<double>> mean_y(nx, std::vector<double>(ny, 0));

  for (size_t i = 0; i < point_list_2d.size(); ++i) {
    int x_index = static_cast<int>((point_list_2d[i][0] - min_x) / resolution);
    int y_index = static_cast<int>((point_list_2d[i][1] - min_y) / resolution);
    x_index = std::max(0, std::min(x_axis_len - 1, x_index));
    y_index = std::max(0, std::min(y_axis_len - 1, y_index));
    mean_x[static_cast<size_t>(x_index)][static_cast<size_t>(y_index)] +=
        point_list_2d[i][0];
    mean_y[static_cast<size_t>(x_index)][static_cast<size_t>(y_index)] +=
        point_list_2d[i][1];
    img_count[static_cast<size_t>(x_index)][static_cast<size_t>(y_index)]++;
    dis_container[static_cast<size_t>(x_index)][static_cast<size_t>(y_index)]
        .push_back(dis_list_2d[i]);
  }

  const int cut_num = static_cast<int>((dis_threshold_max - dis_threshold_min) /
                                       std::max(high_inc, 1e-6));
  for (int x = 0; x < x_axis_len; ++x) {
    for (int y = 0; y < y_axis_len; ++y) {
      if (img_count[static_cast<size_t>(x)][static_cast<size_t>(y)] <= 0) {
        continue;
      }
      std::vector<double> cnt_list(static_cast<size_t>(std::max(cut_num, 1)),
                                   0.0);
      std::vector<bool> occup_list(cnt_list.size(), false);
      for (double d :
           dis_container[static_cast<size_t>(x)][static_cast<size_t>(y)]) {
        int cnt_index = static_cast<int>((d - dis_threshold_min) /
                                         std::max(high_inc, 1e-6));
        cnt_index = std::max(
            0, std::min(static_cast<int>(cnt_list.size()) - 1, cnt_index));
        cnt_list[static_cast<size_t>(cnt_index)]++;
      }
      double segmnt_dis = 0;
      for (size_t i = 0; i < cnt_list.size(); ++i) {
        if (cnt_list[i] >= 1) {
          segmnt_dis++;
          occup_list[i] = true;
        }
      }
      dis_array[static_cast<size_t>(x)][static_cast<size_t>(y)] = segmnt_dis;
      BinaryDescriptor single_binary;
      single_binary.occupy_array = occup_list;
      single_binary.summary =
          static_cast<unsigned char>(std::min(255.0, segmnt_dis));
      binary_container[static_cast<size_t>(x)][static_cast<size_t>(y)] =
          single_binary;
    }
  }

  std::vector<double> max_dis_list;
  std::vector<int> max_dis_x_index_list;
  std::vector<int> max_dis_y_index_list;
  for (int xs = 0; xs < x_segment_num; ++xs) {
    for (int ys = 0; ys < y_segment_num; ++ys) {
      double max_dis = 0;
      int max_dis_x_index = -10;
      int max_dis_y_index = -10;
      for (int x = xs * kSegBase; x < (xs + 1) * kSegBase && x < x_axis_len;
           ++x) {
        for (int y = ys * kSegBase; y < (ys + 1) * kSegBase && y < y_axis_len;
             ++y) {
          if (dis_array[static_cast<size_t>(x)][static_cast<size_t>(y)] >
              max_dis) {
            max_dis = dis_array[static_cast<size_t>(x)][static_cast<size_t>(y)];
            max_dis_x_index = x;
            max_dis_y_index = y;
          }
        }
      }
      if (max_dis >= binary_min_dis && max_dis_x_index >= 0) {
        bool is_touch = true;
        if (cfg.touch_filter_enable) {
          const auto& arr =
              binary_container[static_cast<size_t>(max_dis_x_index)]
                              [static_cast<size_t>(max_dis_y_index)]
                                  .occupy_array;
          is_touch = false;
          for (int k = 0; k < 4 && k < static_cast<int>(arr.size()); ++k) {
            if (arr[static_cast<size_t>(k)]) {
              is_touch = true;
              break;
            }
          }
        }
        if (is_touch) {
          max_dis_list.push_back(max_dis);
          max_dis_x_index_list.push_back(max_dis_x_index);
          max_dis_y_index_list.push_back(max_dis_y_index);
        }
      }
    }
  }

  const Eigen::Vector2i dirs[4] = {{0, 1}, {1, 0}, {1, 1}, {1, -1}};
  for (size_t i = 0; i < max_dis_list.size(); ++i) {
    const int px = max_dis_x_index_list[i];
    const int py = max_dis_y_index_list[i];
    if (px <= 0 || px >= x_axis_len - 1 || py <= 0 || py >= y_axis_len - 1) {
      continue;
    }
    bool is_add = true;
    if (line_filter_enable) {
      for (int j = 0; j < 4; ++j) {
        const int p1x = px + dirs[j][0];
        const int p1y = py + dirs[j][1];
        const int p2x = px - dirs[j][0];
        const int p2y = py - dirs[j][1];
        if (p1x < 0 || p1x >= x_axis_len || p1y < 0 || p1y >= y_axis_len ||
            p2x < 0 || p2x >= x_axis_len || p2y < 0 || p2y >= y_axis_len) {
          continue;
        }
        const double center =
            dis_array[static_cast<size_t>(px)][static_cast<size_t>(py)];
        const double threshold = center - 3;
        const double d1 =
            dis_array[static_cast<size_t>(p1x)][static_cast<size_t>(p1y)];
        const double d2 =
            dis_array[static_cast<size_t>(p2x)][static_cast<size_t>(p2y)];
        if (d1 >= threshold && d2 >= 0.5 * center) {
          is_add = false;
        }
        if (d2 >= threshold && d1 >= 0.5 * center) {
          is_add = false;
        }
        if (d1 >= threshold && d2 >= threshold) {
          is_add = false;
        }
      }
    }
    if (!is_add) {
      continue;
    }
    const double cnt =
        img_count[static_cast<size_t>(px)][static_cast<size_t>(py)];
    const double mx =
        mean_x[static_cast<size_t>(px)][static_cast<size_t>(py)] / cnt;
    const double my =
        mean_y[static_cast<size_t>(px)][static_cast<size_t>(py)] / cnt;
    BinaryDescriptor single =
        binary_container[static_cast<size_t>(px)][static_cast<size_t>(py)];
    single.location = my * x_axis + mx * y_axis + project_center;
    binary_list->push_back(single);
  }
}

void NonMaxSuppress(std::vector<BinaryDescriptor>* binary_list,
                    const TriangleConfig& cfg) {
  if (binary_list == nullptr || binary_list->empty()) {
    return;
  }
  const double radius = cfg.non_max_suppression_radius;
  const double r2 = radius * radius;
  std::vector<bool> keep(binary_list->size(), true);
  for (size_t i = 0; i < binary_list->size(); ++i) {
    for (size_t j = 0; j < binary_list->size(); ++j) {
      if (i == j || !keep[i]) {
        continue;
      }
      if (((*binary_list)[i].location - (*binary_list)[j].location)
              .squaredNorm() > r2) {
        continue;
      }
      if ((*binary_list)[i].summary <= (*binary_list)[j].summary) {
        keep[i] = false;
      }
    }
  }
  std::vector<BinaryDescriptor> pass;
  for (size_t i = 0; i < keep.size(); ++i) {
    if (keep[i]) {
      pass.push_back((*binary_list)[i]);
    }
  }
  *binary_list = std::move(pass);
}

void ExtractCorners(const std::vector<TrianglePlane>& planes,
                    const sensor::PointCloudXYZI& cloud,
                    const TriangleConfig& cfg,
                    std::vector<BinaryDescriptor>* corners) {
  if (corners == nullptr) {
    return;
  }
  corners->clear();
  std::vector<TrianglePlane> merged;
  MergePlanes(planes, cfg, &merged);
  if (merged.empty() && !planes.empty()) {
    // Fallback like original: single horizontal plane.
    TrianglePlane p;
    p.normal = Eigen::Vector3d(0, 0, 1);
    p.center = Eigen::Vector3d(cloud.points[0].x, cloud.points[0].y,
                               cloud.points[0].z);
    merged.push_back(p);
  }
  std::sort(merged.begin(), merged.end(),
            [](const TrianglePlane& a, const TrianglePlane& b) {
              return a.points_size > b.points_size;
            });

  std::vector<BinaryDescriptor> temp;
  Eigen::Vector3d last_normal = Eigen::Vector3d::Zero();
  int useful_proj_num = 0;
  for (const TrianglePlane& pl : merged) {
    if ((pl.normal - last_normal).norm() < 0.3 ||
        (pl.normal + last_normal).norm() > 0.3) {
      last_normal = pl.normal;
      ++useful_proj_num;
      std::vector<BinaryDescriptor> prep;
      ExtractBinary(pl.center, pl.normal, cloud, cfg, &prep);
      temp.insert(temp.end(), prep.begin(), prep.end());
      if (useful_proj_num == cfg.proj_plane_num) {
        break;
      }
    }
  }
  NonMaxSuppress(&temp, cfg);
  if (static_cast<int>(temp.size()) <= cfg.useful_corner_num) {
    *corners = std::move(temp);
  } else {
    std::sort(temp.begin(), temp.end(),
              [](const BinaryDescriptor& a, const BinaryDescriptor& b) {
                return a.summary > b.summary;
              });
    corners->assign(temp.begin(), temp.begin() + cfg.useful_corner_num);
  }
}

void GenerateStdFromBinary(const std::vector<BinaryDescriptor>& corners,
                           int frame_id, const TriangleConfig& cfg,
                           std::vector<TriangleDescriptor>* stds) {
  if (stds == nullptr || corners.size() < 3) {
    return;
  }
  stds->clear();
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(corners.size());
  for (const auto& c : corners) {
    pts.push_back(c.location);
  }
  math::KdTree kd;
  kd.SetInput(pts);
  const int K = std::min(static_cast<int>(cfg.descriptor_near_num) + 1,
                         static_cast<int>(pts.size()));
  std::unordered_map<int64_t, bool> feat_map;
  for (size_t i = 0; i < pts.size(); ++i) {
    std::vector<int> idx;
    std::vector<float> dist2;
    if (kd.NearestKSearch(pts[i], K, &idx, &dist2) < 3) {
      continue;
    }
    for (int m = 1; m < K - 1; ++m) {
      for (int n = m + 1; n < K; ++n) {
        const Eigen::Vector3d& p1 = pts[i];
        const Eigen::Vector3d& p2 =
            pts[static_cast<size_t>(idx[static_cast<size_t>(m)])];
        const Eigen::Vector3d& p3 =
            pts[static_cast<size_t>(idx[static_cast<size_t>(n)])];
        Eigen::Vector3d sides((p1 - p2).norm(), (p1 - p3).norm(),
                              (p2 - p3).norm());
        if (sides[0] > cfg.descriptor_max_len ||
            sides[1] > cfg.descriptor_max_len ||
            sides[2] > cfg.descriptor_max_len ||
            sides[0] < cfg.descriptor_min_len ||
            sides[1] < cfg.descriptor_min_len ||
            sides[2] < cfg.descriptor_min_len) {
          continue;
        }
        // Canonicalize side order with corresponding vertices.
        Eigen::Vector3d A = p1, B = p2, C = p3;
        if (sides[0] > sides[1]) {
          std::swap(sides[0], sides[1]);
          std::swap(B, C);
        }
        if (sides[1] > sides[2]) {
          std::swap(sides[1], sides[2]);
          std::swap(A, B);
        }
        if (sides[0] > sides[1]) {
          std::swap(sides[0], sides[1]);
          std::swap(B, C);
        }
        const int64_t ha = static_cast<int64_t>(
            std::round(sides[0] / cfg.std_side_resolution));
        const int64_t hb = static_cast<int64_t>(
            std::round(sides[1] / cfg.std_side_resolution));
        const int64_t hc = static_cast<int64_t>(
            std::round(sides[2] / cfg.std_side_resolution));
        const int64_t key = ((ha * 116101) + hb) * 116101 + hc;
        if (feat_map.count(key)) {
          continue;
        }
        feat_map[key] = true;
        TriangleDescriptor s;
        s.triangle = sides;
        s.center = (A + B + C) / 3.0;
        s.frame_number = frame_id;
        s.binary_a.location = A;
        s.binary_b.location = B;
        s.binary_c.location = C;
        // Attach occupy arrays when available.
        auto fill_bin = [&](BinaryDescriptor* dst, const Eigen::Vector3d& loc) {
          for (const auto& c : corners) {
            if ((c.location - loc).norm() < 1e-3) {
              *dst = c;
              dst->location = loc;
              return;
            }
          }
          dst->location = loc;
        };
        fill_bin(&s.binary_a, A);
        fill_bin(&s.binary_b, B);
        fill_bin(&s.binary_c, C);
        stds->push_back(s);
      }
    }
  }
}

}  // namespace triangle
}  // namespace loop_closure
}  // namespace vxslam
}  // namespace atlas
