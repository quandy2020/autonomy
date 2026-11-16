#include "autonomy/perception/engine/binding.hpp"

#include "autonomy/perception/engine/options.hpp"

namespace autonomy::perception::base {

std::string OutputName(const proto::BaseOptions& options, proto::TaskKind kind,
                       const char* fallback) {
    const auto* task = FindTask(options, kind);
    return task && !task->output_name().empty() ? task->output_name() : fallback;
}

std::optional<ModelBinding> ResolveBinding(const proto::BaseOptions& options,
                                           const proto::TaskOptions& task) {
    ModelBinding binding{task.model_id(), task.model_path(), options.backend(),
                         proto::MODEL_FAMILY_UNSPECIFIED,
                         options.input_width(), options.input_height()};
    if (!binding.id.empty()) {
        bool found = false;
        for (const auto& model : options.models()) {
            if (model.id() != binding.id)
                continue;
            binding.path = model.path();
            binding.family = model.family();
            if (model.backend() != proto::BACKEND_UNSPECIFIED)
                binding.backend = model.backend();
            if (model.input_width()) {
                binding.width = model.input_width();
                binding.height = model.input_height();
            }
            found = true;
            break;
        }
        if (!found)
            return std::nullopt;
    }
    if (task.input_width()) {
        binding.width = task.input_width();
        binding.height = task.input_height();
    }
    return binding;
}

}  // namespace autonomy::perception::base
