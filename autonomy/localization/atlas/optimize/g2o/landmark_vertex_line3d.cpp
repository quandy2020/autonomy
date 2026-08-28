#include "autonomy/localization/atlas/optimize/g2o/landmark_vertex_line3d.hpp"

namespace autonomy::localization::atlas::optimize::plp_g2o {

VertexLine3D::VertexLine3D() : ::g2o::BaseVertex<4, Line3D>() {}

bool VertexLine3D::read(std::istream& is) {
    Vec6_t lv;
    for (int i = 0; i < 6; ++i) {
        is >> lv(i);
    }
    setEstimate(Line3D(PluckerVec6(lv)));
    return is.good();
}

bool VertexLine3D::write(std::ostream& os) const {
    const PluckerVec6 v = static_cast<PluckerVec6>(_estimate);
    for (int i = 0; i < 6; ++i) {
        os << v(i) << ' ';
    }
    return os.good();
}

}  // namespace autonomy::localization::atlas::optimize::plp_g2o
