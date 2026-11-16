#ifndef AUTONOMY_PERCEPTION_ENGINE_BINDING_HPP_
#define AUTONOMY_PERCEPTION_ENGINE_BINDING_HPP_

#include <optional>
#include <string>

#include "autonomy/perception/proto/perception.pb.h"

namespace autonomy::perception::base {

std::string OutputName(const proto::BaseOptions& options, proto::TaskKind kind,
                       const char* fallback);

struct ModelBinding {
    std::string id;
    std::string path;
    proto::Backend backend;
    proto::ModelFamily family;
    uint32_t width;
    uint32_t height;
};

std::optional<ModelBinding> ResolveBinding(const proto::BaseOptions& options,
                                           const proto::TaskOptions& task);

}  // namespace autonomy::perception::base

#endif
