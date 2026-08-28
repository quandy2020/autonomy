#include "autonomy/localization/atlas/optimize/g2o/landmark_vertex_container_line3d.hpp"

namespace autonomy::localization::atlas::optimize::plp_g2o {

landmark_vertex_container_line3d::landmark_vertex_container_line3d(const std::shared_ptr<unsigned int>& offset,
                                                                     const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
}

VertexLine3D* landmark_vertex_container_line3d::create_vertex(const std::shared_ptr<data::landmark_line>& lm,
                                                                const bool is_constant) {
    return create_vertex(lm->id_, lm->get_pluecker_coord(), is_constant);
}

VertexLine3D* landmark_vertex_container_line3d::create_vertex(const unsigned int id, const Vec6_t& pluecker,
                                                               const bool is_constant) {
    const auto vtx_id = (*offset_)++;
    auto* vtx = new VertexLine3D();
    vtx->setId(static_cast<int>(vtx_id));
    vtx->setEstimate(Line3D(pluecker));
    vtx->setFixed(is_constant);
    vtx->setMarginalized(true);
    vtx_container_[id] = vtx;
    return vtx;
}

VertexLine3D* landmark_vertex_container_line3d::get_vertex(const std::shared_ptr<data::landmark_line>& lm) const {
    return get_vertex(lm->id_);
}

VertexLine3D* landmark_vertex_container_line3d::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

bool landmark_vertex_container_line3d::contain(const std::shared_ptr<data::landmark_line>& lm) const {
    return vtx_container_.count(lm->id_) != 0;
}

}  // namespace autonomy::localization::atlas::optimize::plp_g2o
