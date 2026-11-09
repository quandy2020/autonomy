/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/memory/coverage_gate.hpp"

#include "navigation/common/habitat_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace navigation
{
namespace memory
{
namespace
{

constexpr int kGrid = 16;
constexpr int kXzBits = 14;
constexpr int kYBits = 10;
constexpr int kBearingSlots = 16;
constexpr double kLevels[] = {0.25, 0.5, 1.0, 2.0};
constexpr int kLevelCount = 4;
constexpr int kBearingBins = 8;
constexpr int kBearingTol = 1;
constexpr double kMaxRangeM = 10.0;
constexpr double kMinValidFrac = 0.5;
constexpr double kClaimRangeM = 6.0;
constexpr double kDepthMin = 0.1;
constexpr double kDefaultHfovDeg = 110.0;

std::uint64_t CellKey(int level, int ix, int iy, int iz, int bearing)
{
    const int xz_max = (1 << kXzBits) - 1;
    const int y_max = (1 << kYBits) - 1;
    ix = std::clamp(ix + (1 << (kXzBits - 1)), 0, xz_max);
    iz = std::clamp(iz + (1 << (kXzBits - 1)), 0, xz_max);
    iy = std::clamp(iy + (1 << (kYBits - 1)), 0, y_max);
    const std::uint64_t high =
        static_cast<std::uint64_t>(level * kBearingSlots + bearing)
        << (2 * kXzBits + kYBits);
    return high | (static_cast<std::uint64_t>(ix) << (kXzBits + kYBits)) |
           (static_cast<std::uint64_t>(iz) << kYBits) |
           static_cast<std::uint64_t>(iy);
}

bool ReadDepth(const observation::Observation& observation, int u, int v,
               float* value)
{
    const auto& image = observation.depth();
    if (image.encoding() != "32FC1" || image.width() <= 0 || image.height() <= 0)
    {
        return false;
    }
    const std::size_t index =
        static_cast<std::size_t>(v) * static_cast<std::size_t>(image.width()) +
        static_cast<std::size_t>(u);
    if ((index + 1U) * sizeof(float) > image.data().size())
    {
        return false;
    }
    std::memcpy(value, image.data().data() + index * sizeof(float), sizeof(float));
    return std::isfinite(*value);
}

}  // namespace

CoverageFrame FrameCells(const observation::Observation& observation,
                         const common::Pose2D& ros_pose, double camera_height_m)
{
    CoverageFrame frame;
    if (!observation.has_depth())
    {
        return frame;
    }
    const auto& image = observation.depth();
    const int width = static_cast<int>(image.width());
    const int height = static_cast<int>(image.height());
    if (width < kGrid || height < kGrid)
    {
        return frame;
    }

    double fx = 0.0;
    double fy = 0.0;
    double cx = width * 0.5 - 0.5;
    double cy = height * 0.5 - 0.5;
    if (observation.has_camera_info() && observation.camera_info().k_size() >= 6)
    {
        fx = observation.camera_info().k(0);
        fy = observation.camera_info().k(4);
        cx = observation.camera_info().k(2);
        cy = observation.camera_info().k(5);
    }
    if (fx <= 1.0 || fy <= 1.0)
    {
        const double hfov = kDefaultHfovDeg * M_PI / 180.0;
        fx = width / (2.0 * std::tan(hfov * 0.5));
        fy = fx;
    }

    const common::Pose2D habitat = common::RosToHabitat(ros_pose);
    const double yaw = habitat.theta();
    const double cyaw = std::cos(yaw);
    const double syaw = std::sin(yaw);
    // Habitat R_y(yaw): camera x right, y up, z back.
    const double r00 = cyaw;
    const double r02 = syaw;
    const double r20 = -syaw;
    const double r22 = cyaw;
    const double tx = habitat.x();
    const double ty = camera_height_m;
    const double tz = habitat.y();

    const int cell_w = width / kGrid;
    const int cell_h = height / kGrid;
    const double tan_patch = std::tan((kDefaultHfovDeg * M_PI / 180.0) / kGrid);

    struct Patch
    {
        double wx;
        double wy;
        double wz;
        int bearing;
        int level;
        bool claim;
    };
    std::vector<Patch> patches;
    patches.reserve(kGrid * kGrid);

    for (int i = 0; i < kGrid; ++i)
    {
        for (int j = 0; j < kGrid; ++j)
        {
            int valid = 0;
            int count = 0;
            std::vector<float> samples;
            samples.reserve(static_cast<std::size_t>(cell_w * cell_h));
            for (int dv = 0; dv < cell_h; ++dv)
            {
                for (int du = 0; du < cell_w; ++du)
                {
                    const int u = j * cell_w + du;
                    const int v = i * cell_h + dv;
                    float depth = 0.0F;
                    ++count;
                    if (!ReadDepth(observation, u, v, &depth) || depth <= kDepthMin)
                    {
                        continue;
                    }
                    ++valid;
                    samples.push_back(depth);
                }
            }
            if (count == 0 || static_cast<double>(valid) / count < kMinValidFrac ||
                samples.empty())
            {
                continue;
            }
            std::nth_element(samples.begin(), samples.begin() + samples.size() / 2,
                             samples.end());
            const double depth = samples[samples.size() / 2];
            if (depth <= kDepthMin || depth >= kMaxRangeM)
            {
                continue;
            }
            const double u = cell_w * j + (cell_w - 1) * 0.5;
            const double v = cell_h * i + (cell_h - 1) * 0.5;
            const double a = (u - cx) / fx;
            const double b = -(v - cy) / fy;
            const double cam_x = a * depth;
            const double cam_y = b * depth;
            const double cam_z = -depth;
            const double wx = r00 * cam_x + r02 * cam_z + tx;
            const double wy = cam_y + ty;
            const double wz = r20 * cam_x + r22 * cam_z + tz;
            const double ray_x = wx - tx;
            const double ray_z = wz - tz;
            const double width_bin = 2.0 * M_PI / kBearingBins;
            int bearing = static_cast<int>(
                std::floor((std::atan2(-ray_x, -ray_z) + width_bin * 0.5) / width_bin));
            bearing = ((bearing % kBearingBins) + kBearingBins) % kBearingBins;
            int level = 0;
            const double footprint = depth * tan_patch;
            while (level < kLevelCount && kLevels[level] < footprint)
            {
                ++level;
            }
            if (level >= kLevelCount)
            {
                level = kLevelCount - 1;
            }
            const double range = std::sqrt(ray_x * ray_x + (wy - ty) * (wy - ty) +
                                           ray_z * ray_z);
            patches.push_back(
                Patch{wx, wy, wz, bearing, level, range < kClaimRangeM});
        }
    }

    const int neighbor_count = 9 * (2 * kBearingTol + 1);
    frame.patches = static_cast<int>(patches.size());
    frame.neighbors = neighbor_count * 2;
    frame.lookup.assign(static_cast<std::size_t>(frame.patches) *
                            static_cast<std::size_t>(frame.neighbors),
                        -1);
    std::vector<std::uint64_t> claims;
    claims.reserve(patches.size() * kLevelCount);

    auto neighborhood = [&](int patch, int lv, int slot_base) {
        const Patch& p = patches[static_cast<std::size_t>(patch)];
        const int ix = static_cast<int>(std::floor(p.wx / kLevels[lv]));
        const int iy = static_cast<int>(std::floor(p.wy / kLevels[lv]));
        const int iz = static_cast<int>(std::floor(p.wz / kLevels[lv]));
        int slot = slot_base;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                for (int db = -kBearingTol; db <= kBearingTol; ++db)
                {
                    const int bearing =
                        ((p.bearing + db) % kBearingBins + kBearingBins) % kBearingBins;
                    const std::size_t index =
                        static_cast<std::size_t>(patch) * frame.neighbors + slot;
                    frame.lookup[index] = static_cast<std::int64_t>(
                        CellKey(lv, ix + dx, iy, iz + dz, bearing));
                    ++slot;
                }
            }
        }
    };

    for (int p = 0; p < frame.patches; ++p)
    {
        const Patch& pt = patches[static_cast<std::size_t>(p)];
        neighborhood(p, pt.level, 0);
        if (pt.level < kLevelCount - 1)
        {
            neighborhood(p, pt.level + 1, neighbor_count);
        }
        if (!pt.claim)
        {
            continue;
        }
        for (int lv = pt.level; lv < kLevelCount; ++lv)
        {
            const int ix = static_cast<int>(std::floor(pt.wx / kLevels[lv]));
            const int iy = static_cast<int>(std::floor(pt.wy / kLevels[lv]));
            const int iz = static_cast<int>(std::floor(pt.wz / kLevels[lv]));
            claims.push_back(CellKey(lv, ix, iy, iz, pt.bearing));
        }
    }
    std::sort(claims.begin(), claims.end());
    claims.erase(std::unique(claims.begin(), claims.end()), claims.end());
    frame.claims = std::move(claims);
    return frame;
}

CoverageGate::CoverageGate(int eta) : eta_(eta > 0 ? eta : 10) {}

bool CoverageGate::Offer(const CoverageFrame& frame, bool force)
{
    int unseen = 0;
    for (int p = 0; p < frame.patches; ++p)
    {
        bool seen = false;
        for (int n = 0; n < frame.neighbors; ++n)
        {
            const std::int64_t key =
                frame.lookup[static_cast<std::size_t>(p) * frame.neighbors + n];
            if (key < 0)
            {
                continue;
            }
            if (claimed_.count(static_cast<std::uint64_t>(key)) != 0)
            {
                seen = true;
                break;
            }
        }
        if (!seen)
        {
            ++unseen;
        }
    }
    const bool keep = force || unseen >= eta_;
    if (keep)
    {
        claimed_.insert(frame.claims.begin(), frame.claims.end());
    }
    return keep;
}

void CoverageGate::Reset()
{
    claimed_.clear();
}

}  // namespace memory
}  // namespace navigation
