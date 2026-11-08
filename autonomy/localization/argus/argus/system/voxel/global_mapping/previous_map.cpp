/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/global_mapping/previous_map.hpp"

#include <deque>
#include <iostream>

#include "argus/system/voxel/odometry/downsample.hpp"

namespace argus {
namespace vxslam {

namespace {

bool LoadScanPoses(const std::string& path,
                   std::vector<ScanPosePtr>* out) {
    if (out == nullptr) {
        return false;
    }
    std::vector<SessionPoseEntry> entries;
    if (!LoadSessionPoses(path, &entries)) {
        return false;
    }
    out->clear();
    out->reserve(entries.size());
    for (const SessionPoseEntry& e : entries) {
        auto sp = std::make_shared<ScanPose>(e.x, nullptr);
        sp->v6 = e.v6;
        out->push_back(std::move(sp));
    }
    return !out->empty();
}

}  // namespace

bool PreviousMapRead(const PreviousMapOptions& opt,
                     std::vector<PreviousMapSession>* sessions,
                     PgoEdges* edges) {
    if (sessions == nullptr) {
        return false;
    }
    sessions->clear();
    const auto specs = ParsePreviousMapList(opt.previous_map);
    if (specs.empty()) {
        return true;
    }
    std::vector<std::string> fnames;
    for (const PreviousMapSpec& s : specs) {
        fnames.push_back(s.name);
    }
    if (edges != nullptr) {
        PgoEdgesIo(edges, fnames, /*io=*/0, opt.save_path, /*bagname=*/"");
    }

    const int win = std::max(1, opt.win_size);
    const int acsize = std::max(1, opt.acsize);
    const int mgsize = std::max(1, opt.mgsize);

    for (const PreviousMapSpec& spec : specs) {
        PreviousMapSession sess;
        sess.name = spec.name;
        sess.jud = spec.jud;
        const std::string root = opt.save_path + spec.name;
        if (!LoadScanPoses(root + "/alidarState.txt", &sess.scan_poses)) {
            std::cerr << "PreviousMapRead: missing poses " << root << "\n";
            continue;
        }
        std::cerr << "PreviousMapRead " << root << ": "
                  << sess.scan_poses.size() << " scans\n";

        std::deque<sensor::PointCloudXYZI> plbuf;
        std::deque<ImuState> xxbuf;
        for (int i = 0; i < static_cast<int>(sess.scan_poses.size()); ++i) {
            sensor::PointCloudXYZI cloud;
            const std::string bin =
                root + "/" + std::to_string(i) + ".bin";
            if (!LoadCloudBin(bin, &cloud)) {
                continue;
            }
            xxbuf.push_back(sess.scan_poses[static_cast<size_t>(i)]->x);
            plbuf.push_back(std::move(cloud));
            if (static_cast<int>(xxbuf.size()) < win) {
                continue;
            }
            ImuState xc = xxbuf.back();
            auto* smp = new Keyframe(xc);
            smp->id = i;
            smp->mp = static_cast<int>(sessions->size());
            smp->exist = 1;
            sensor::PointCloudXYZI pl_lc;
            for (int j = 0; j < win; ++j) {
                const Eigen::Vector3d delta_p =
                    xc.R.transpose() * (xxbuf[static_cast<size_t>(j)].p - xc.p);
                const Eigen::Matrix3d delta_R =
                    xc.R.transpose() * xxbuf[static_cast<size_t>(j)].R;
                for (const auto& pp : plbuf[static_cast<size_t>(j)].points) {
                    Eigen::Vector3d v3(pp.x, pp.y, pp.z);
                    v3 = delta_R * v3 + delta_p;
                    sensor::PointType pt;
                    pt.x = static_cast<float>(v3[0]);
                    pt.y = static_cast<float>(v3[1]);
                    pt.z = static_cast<float>(v3[2]);
                    pl_lc.push_back(pt);
                }
            }
            DownsampleVoxel(&pl_lc, opt.voxel_size / 10.0);
            smp->plptr->reserve(pl_lc.size());
            for (const auto& pp : pl_lc.points) {
                smp->plptr->push_back(pp);
            }
            sess.keyframes.push_back(smp);
            for (int j = 0; j < win; ++j) {
                plbuf.pop_front();
                xxbuf.pop_front();
            }
        }

        sess.triangle =
            std::make_unique<loop_closure::triangle::TriangleDetector>(opt.triangle);
        const int subsize = static_cast<int>(sess.keyframes.size());
        for (int i = 0; i + acsize < subsize; i += mgsize) {
            const int up = i + acsize;
            sensor::PointCloudXYZI pl_btc;
            const ImuState& xc = sess.keyframes[static_cast<size_t>(up - 1)]->x0;
            for (int j = i; j < up; ++j) {
                const ImuState& xj = sess.keyframes[static_cast<size_t>(j)]->x0;
                const Eigen::Vector3d delta_p =
                    xc.R.transpose() * (xj.p - xc.p);
                const Eigen::Matrix3d delta_R = xc.R.transpose() * xj.R;
                for (const auto& ap :
                     sess.keyframes[static_cast<size_t>(j)]->plptr->points) {
                    Eigen::Vector3d v3(ap.x, ap.y, ap.z);
                    v3 = delta_R * v3 + delta_p;
                    sensor::PointType pp;
                    pp.x = static_cast<float>(v3[0]);
                    pp.y = static_cast<float>(v3[1]);
                    pp.z = static_cast<float>(v3[2]);
                    pl_btc.push_back(pp);
                }
            }
            std::vector<loop_closure::triangle::TriangleDescriptor> stds;
            sensor::PointCloudXYZI plane;
            sess.triangle->GenerateDescriptors(
                pl_btc, sess.keyframes[static_cast<size_t>(up - 1)]->id, &stds,
                &plane);
            sess.triangle->AddDescriptors(stds, plane);
        }
        sess.triangle->DisableNearSkip();
        sessions->push_back(std::move(sess));
        std::cerr << "PreviousMapRead done: " << root
                  << " keyframes=" << sessions->back().keyframes.size()
                  << " btc=" << sessions->back().triangle->database_size()
                  << "\n";
    }
    return !sessions->empty();
}

}  // namespace vxslam
}  // namespace argus
