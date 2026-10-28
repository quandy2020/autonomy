#pragma once

#include <memory>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {

class GeometricCamera;

class CameraFactory
{
public:
    static std::unique_ptr<GeometricCamera> Create(
        const std::string& model, const std::vector<float>& parameters);
};

}  // namespace autonomy::localization::atlas
