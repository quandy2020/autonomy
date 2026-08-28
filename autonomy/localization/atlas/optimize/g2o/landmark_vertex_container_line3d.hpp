/**
 * g2o vertex container for 3D line landmarks (Plücker / orthonormal).
 */
#pragma once

#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/optimize/g2o/landmark_vertex_line3d.hpp"
#include "autonomy/localization/atlas/optimize/g2o/line3d.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <unordered_map>

namespace autonomy::localization::atlas::optimize::plp_g2o {

class landmark_vertex_container_line3d {
public:
    explicit landmark_vertex_container_line3d(const std::shared_ptr<unsigned int>& offset,
                                                unsigned int num_reserve = 200);

    VertexLine3D* create_vertex(const std::shared_ptr<data::landmark_line>& lm, bool is_constant);
    VertexLine3D* create_vertex(unsigned int id, const Vec6_t& pluecker, bool is_constant);

    VertexLine3D* get_vertex(const std::shared_ptr<data::landmark_line>& lm) const;
    VertexLine3D* get_vertex(unsigned int id) const;

    bool contain(const std::shared_ptr<data::landmark_line>& lm) const;

private:
    const std::shared_ptr<unsigned int> offset_;
    std::unordered_map<unsigned int, VertexLine3D*> vtx_container_;
};

}  // namespace autonomy::localization::atlas::optimize::plp_g2o
