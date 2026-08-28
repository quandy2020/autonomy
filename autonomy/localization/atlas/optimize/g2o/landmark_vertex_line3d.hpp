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

#ifndef PLPSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_LINE3D_H
#define PLPSLAM_OPTIMIZER_G2O_LANDMARK_VERTEX_LINE3D_H

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/optimize/g2o/line3d.hpp"

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

namespace autonomy::localization::atlas
{
    namespace optimize
    {
        namespace plp_g2o
        {
            // FW:
            //  (This file re-written from g2o library)

            class VertexLine3D : public ::g2o::BaseVertex<4, Line3D>
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

                VertexLine3D();
                virtual bool read(std::istream &is);
                virtual bool write(std::ostream &os) const;

                virtual void setToOriginImpl()
                {
                    _estimate = Line3D(); // Plücker coordinates
                }

                void oplusImpl(const double* update_) override {
                    Eigen::Map<const Eigen::Vector4d> update(update_);
                    _estimate.oplus(update);
                }

                virtual bool setEstimateDataImpl(const double* est) override {
                    _estimate = Line3D(PluckerVec6(Eigen::Map<const Vec6_t>(est)));
                    return true;
                }

                virtual bool getEstimateData(double* est) const override {
                    Eigen::Map<Vec6_t> est_map(est);
                    est_map = static_cast<PluckerVec6>(_estimate);
                    return true;
                }

                int estimateDimension() const override { return 6; }
            };
        }
    }
}

#endif