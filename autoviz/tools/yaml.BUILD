cc_library(
    name = "yaml_cpp",
    hdrs = [
        "autonomy/common/yaml.hpp",
        "yaml_cpp_shim/yaml-cpp/yaml.h",
        "yaml_cpp_shim/yaml-cpp/emitter.h",
    ],
    includes = [
        ".",
        "yaml_cpp_shim",
    ],
    visibility = ["//visibility:public"],
    deps = ["@autolink//:fkYAML"],
)
