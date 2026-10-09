"""Package macros for //autonomy/* (Apollo tools/apollo_package.bzl counterpart).

Public API:
  autonomy_cc_library / autonomy_cc_binary / autonomy_cc_test
  autonomy_runtime_data
  autonomy_domain_library
  autonomy_module_copts
  AUTONOMY_DOMAIN_NAMES  (also parsed by autonomy.sh)

Domain libraries are named ``autonomy_<domain>``. CMake owns packaging and
``*.pb.h`` codegen; Bazel links ``@autonomy_prefix`` for automsgs / autolink.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:dependencies.bzl", "AUTONOMY_THIRD_PARTY_DEPS")

# ---------------------------------------------------------------------------
# Domain graph (//autonomy/<name>:autonomy_<name>)
# ---------------------------------------------------------------------------

AUTONOMY_DOMAIN_NAMES = [
    "audio",
    "bridge",
    "common",
    "control",
    "localization",
    "map",
    "perception",
    "planning",
    "prediction",
    "sensor",
    "system",
    "task",
    "transform",
    "vehicle",
]

# domain → peer domains (CMake AUTONOMY_BUILD_* edges).
AUTONOMY_DOMAIN_DEPS = {
    "common": [],
    "transform": ["common"],
    "map": ["common", "transform"],
    "vehicle": ["common"],
    "prediction": ["common"],
    "control": ["common", "transform", "map"],
    "planning": ["common", "transform", "map"],
    "sensor": ["common", "control"],
    "task": ["common", "transform", "map", "control"],
    "system": ["common", "task"],
    "bridge": ["common", "system"],
    "localization": ["common", "transform"],
    "perception": ["common"],
    "audio": ["common"],
}

# Non-domain labels appended per domain (e.g. middleware).
AUTONOMY_DOMAIN_EXTRA_DEPS = {
    "bridge": ["//:autolink"],
    "control": ["//:autolink"],
    "localization": ["//:autolink"],
    "planning": ["//:autolink"],
    "sensor": ["//:autolink"],
    "system": ["//:autolink"],
    "task": ["//:autolink"],
    "transform": ["//:autolink"],
}

AUTONOMY_DOMAIN_SRC_EXCLUDES = {
    "common": [
        "**/async_grpc/**",
        "**/optimization/ipopt/**",
        "**/network/**",
        "**/lua_parameter_dictionary.cpp",
        "**/configuration_file_resolver.cpp",
        "**/version.cpp",
    ],
    "map": [
        "**/grid_map_demos/**",
        "**/strata/**",
    ],
    "localization": [
        "**/atlas/thirdparty/**",
    ],
}

def autonomy_domain_label(name):
    """Return ``//autonomy/<name>:autonomy_<name>``."""
    return "//autonomy/%s:autonomy_%s" % (name, name)

def autonomy_domain_deps(name):
    """Peer domain labels + AUTONOMY_DOMAIN_EXTRA_DEPS for ``name``."""
    peers = [autonomy_domain_label(d) for d in AUTONOMY_DOMAIN_DEPS.get(name, [])]
    return peers + AUTONOMY_DOMAIN_EXTRA_DEPS.get(name, [])

# ---------------------------------------------------------------------------
# Shared excludes / macros
# ---------------------------------------------------------------------------

AUTONOMY_SRC_EXCLUDES = [
    "**/*_main.cpp",
    "**/*_test.cpp",
    "**/test/**",
    "**/testing/**",
    "**/tests/**",
    "**/fake/**",
    "**/mock/**",
]

AUTONOMY_HDR_EXCLUDES = [
    "**/test/**",
    "**/testing/**",
    "**/tests/**",
]

def autonomy_module_copts(name):
    """Return ``-DMODULE_NAME=\"name\"`` copts."""
    return ['-DMODULE_NAME=\\"%s\\"' % name]

def autonomy_cc_library(
        name,
        srcs = [],
        hdrs = [],
        deps = [],
        copts = [],
        defines = [],
        includes = [],
        include_prefix = None,
        strip_include_prefix = None,
        textual_hdrs = [],
        data = [],
        linkopts = [],
        alwayslink = False,
        visibility = None,
        **kwargs):
    """C++ library wrapper (default public visibility)."""
    lib_kwargs = dict(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps,
        copts = copts,
        defines = defines,
        includes = includes,
        textual_hdrs = textual_hdrs,
        data = data,
        linkopts = linkopts,
        alwayslink = alwayslink,
        visibility = visibility if visibility != None else ["//visibility:public"],
    )
    if include_prefix != None:
        lib_kwargs["include_prefix"] = include_prefix
    if strip_include_prefix != None:
        lib_kwargs["strip_include_prefix"] = strip_include_prefix
    lib_kwargs.update(kwargs)
    cc_library(**lib_kwargs)

def autonomy_cc_binary(
        name,
        srcs = [],
        deps = [],
        copts = [],
        data = [],
        linkopts = [],
        visibility = None,
        **kwargs):
    """C++ binary wrapper (default public visibility)."""
    cc_binary(
        name = name,
        srcs = srcs,
        deps = deps,
        copts = copts,
        data = data,
        linkopts = linkopts,
        visibility = visibility if visibility != None else ["//visibility:public"],
        **kwargs
    )

def autonomy_cc_test(
        name,
        srcs = [],
        deps = [],
        copts = [],
        data = [],
        size = "small",
        visibility = None,
        **kwargs):
    """C++ test wrapper (default public visibility)."""
    cc_test(
        name = name,
        srcs = srcs,
        deps = deps,
        copts = copts,
        data = data,
        size = size,
        visibility = visibility if visibility != None else ["//visibility:public"],
        **kwargs
    )

def autonomy_runtime_data(
        name = "runtime_data",
        srcs = None,
        visibility = None):
    """Conf / dag / launch filegroup."""
    if srcs == None:
        srcs = native.glob(
            [
                "conf/**",
                "config/**",
                "dag/**",
                "launch/**",
                "**/*.yaml",
                "**/*.yml",
                "**/*.json",
                "**/*.pb.txt",
            ],
            allow_empty = True,
        )
    native.filegroup(
        name = name,
        srcs = srcs,
        visibility = visibility if visibility != None else ["//visibility:public"],
    )

def autonomy_domain_library(
        name,
        deps = None,
        exclude_srcs = [],
        extra_srcs = [],
        extra_hdrs = [],
        extra_deps = [],
        copts = None,
        **kwargs):
    """Declare ``autonomy_<name>`` + short alias + runtime_data + protobuf_sources.

    Prefer listing ``deps`` explicitly in each domain BUILD so libraries are
    visible at a glance. When ``deps`` is omitted, peer domains +
    AUTONOMY_THIRD_PARTY_DEPS are filled in from this file.
    """
    lib = "autonomy_" + name
    src_excludes = AUTONOMY_SRC_EXCLUDES + AUTONOMY_DOMAIN_SRC_EXCLUDES.get(name, []) + exclude_srcs
    if deps == None:
        all_deps = autonomy_domain_deps(name) + AUTONOMY_THIRD_PARTY_DEPS + extra_deps
    else:
        # Explicit deps in BUILD.bazel — do not auto-append (avoids duplicates).
        all_deps = deps + extra_deps
    if copts == None:
        copts = autonomy_module_copts(name)

    autonomy_cc_library(
        name = lib,
        srcs = native.glob(
            ["**/*.cpp", "**/*.cc", "**/*.c"],
            exclude = src_excludes,
            allow_empty = True,
        ) + extra_srcs,
        hdrs = native.glob(
            ["**/*.hpp", "**/*.h", "**/*.hh", "**/*.inl"],
            exclude = AUTONOMY_HDR_EXCLUDES,
            allow_empty = True,
        ) + extra_hdrs,
        copts = copts,
        include_prefix = "autonomy/" + name,
        deps = all_deps,
        **kwargs
    )

    native.alias(
        name = name,
        actual = ":" + lib,
    )

    native.filegroup(
        name = "protobuf_sources",
        srcs = native.glob(["**/*.proto"], allow_empty = True),
    )

    autonomy_runtime_data()
