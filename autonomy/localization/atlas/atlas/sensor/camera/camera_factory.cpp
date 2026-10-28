#include "atlas/sensor/camera/camera_factory.hpp"

#include "atlas/sensor/camera/kannala_brandt_camera.hpp"
#include "atlas/sensor/camera/pinhole_camera.hpp"

namespace autonomy::localization::atlas {

std::unique_ptr<GeometricCamera> CameraFactory::Create(
    const std::string& model, const std::vector<float>& parameters) {
    if ((model == "Pinhole" || model == "PinHole" ||
         model == "PinholeCamera") &&
        parameters.size() == 4) {
        return std::make_unique<PinholeCamera>(parameters);
    }
    if ((model == "KannalaBrandt8" || model == "KannalaBrandtCamera") &&
        parameters.size() == 8) {
        return std::make_unique<KannalaBrandtCamera>(parameters);
    }
    return nullptr;
}

}  // namespace autonomy::localization::atlas
