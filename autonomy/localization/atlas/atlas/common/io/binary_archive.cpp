#include "atlas/common/io/binary_archive.hpp"

#include "atlas/sensor/camera/geometric_camera.hpp"
#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"

namespace autonomy::localization::atlas {

void BinaryOutputArchive::WritePointedObject(GeometricCamera* pointer) {
    std::uint8_t type = 0;
    if (dynamic_cast<PinholeCamera*>(pointer) != nullptr)
        type = 1;
    else if (dynamic_cast<KannalaBrandtCamera*>(pointer) != nullptr)
        type = 2;
    else
        throw std::runtime_error("unsupported Atlas camera type");
    *this& type;
    if (type == 1)
        *this&* static_cast<PinholeCamera*>(pointer);
    else
        *this&* static_cast<KannalaBrandtCamera*>(pointer);
}

void BinaryInputArchive::ReadNewPointer(std::uint64_t id,
                                        GeometricCamera*& pointer) {
    std::uint8_t type = 0;
    *this& type;
    if (type == 1)
        pointer = new PinholeCamera();
    else if (type == 2)
        pointer = new KannalaBrandtCamera();
    else
        throw std::runtime_error("unsupported Atlas camera type");
    pointers_[id] = pointer;
    if (type == 1)
        *this&* static_cast<PinholeCamera*>(pointer);
    else
        *this&* static_cast<KannalaBrandtCamera*>(pointer);
}

}  // namespace autonomy::localization::atlas
