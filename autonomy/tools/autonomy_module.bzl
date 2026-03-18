"""
Autonomy Bazel Macros

This file provides reusable macros for building autonomy modules.
It simplifies BUILD.bazel files by encapsulating common patterns.
"""

load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_binary")
load("@rules_python//python:proto.bzl", "py_proto_library")

# Common glob patterns
_DEFAULT_HEADER_PATTERNS = [
    "*.hpp",
    "*.h",
    "**/*.hpp",
    "**/*.h",
]

_DEFAULT_SOURCE_PATTERNS = [
    "*.cpp",
    "*.cc",
    "**/*.cpp",
    "**/*.cc",
]

_DEFAULT_EXCLUDES = [
    "**/*_test.hpp",
    "**/*_test.h",
    "**/*_test.cpp",
    "**/*_test.cc",
    "**/test/**",
    "**/testing/**",
]

def _get_module_path(package_name):
    """Extract module path from package name (e.g., 'autonomy/control' from 'autonomy/control/BUILD.bazel')."""
    # Package name is already the full path (e.g., "autonomy/commsgs")
    return package_name

def autonomy_proto_library(
        name,
        proto_srcs = None,
        proto_glob = ["proto/*.proto"],
        proto_glob_recursive = None,
        import_prefix = None,
        deps = None,
        visibility = ["//visibility:public"]):
    """
    Creates a proto_library for an autonomy module.

    Args:
        name: Name of the proto_library target (e.g., "control_proto")
        proto_srcs: Explicit list of proto files (if None, uses glob)
        proto_glob: Glob pattern for proto files (default: ["proto/*.proto"])
        proto_glob_recursive: Recursive glob pattern (e.g., ["proto/**/*.proto"])
        import_prefix: Import prefix for proto files (auto-detected from package if None)
        deps: Dependencies for proto_library
        visibility: Visibility of the target
    """
    if proto_srcs == None:
        if proto_glob_recursive:
            proto_srcs = native.glob(proto_glob_recursive)
        else:
            proto_srcs = native.glob(proto_glob)

    # Auto-detect import_prefix from package name if not provided
    if import_prefix == None:
        package_name = native.package_name()
        import_prefix = _get_module_path(package_name)

    proto_library(
        name = name,
        srcs = proto_srcs,
        import_prefix = import_prefix,
        visibility = visibility,
        deps = deps or ["@com_google_protobuf//:any_proto"],
    )

def autonomy_cc_proto_library(
        name,
        proto_dep,
        visibility = ["//visibility:public"]):
    """
    Creates a cc_proto_library for an autonomy module.

    Args:
        name: Name of the cc_proto_library target
        proto_dep: The proto_library target to depend on
        visibility: Visibility of the target
    """
    cc_proto_library(
        name = name,
        deps = [proto_dep],
        visibility = visibility,
    )

def autonomy_cc_library(
        name,
        srcs = None,
        hdrs = None,
        srcs_glob = None,
        hdrs_glob = None,
        exclude_patterns = None,
        deps = None,
        includes = ["."],
        copts = ["-std=c++17"],
        linkstatic = True,
        alwayslink = 0,
        visibility = ["//visibility:public"]):
    """
    Creates a cc_library for an autonomy module with common patterns.

    Args:
        name: Name of the cc_library target
        srcs: Explicit list of source files (if None, uses glob)
        hdrs: Explicit list of header files (if None, uses glob)
        srcs_glob: Custom glob patterns for sources (default: all .cpp/.cc files)
        hdrs_glob: Custom glob patterns for headers (default: all .hpp/.h files)
        exclude_patterns: Additional patterns to exclude (merged with defaults)
        deps: Dependencies for the library
        includes: Include directories
        copts: Compiler options
        linkstatic: Whether to link statically (True) or as shared library (False)
        alwayslink: Whether to always link all symbols (useful for plugins)
        visibility: Visibility of the target
    """
    # Build exclude list
    excludes = _DEFAULT_EXCLUDES[:]
    if exclude_patterns:
        excludes.extend(exclude_patterns)

    # Collect headers
    if hdrs == None:
        if hdrs_glob:
            hdrs = native.glob(hdrs_glob, exclude = excludes)
        else:
            hdrs = native.glob(_DEFAULT_HEADER_PATTERNS, exclude = excludes)

    # Collect sources
    if srcs == None:
        if srcs_glob:
            srcs = native.glob(srcs_glob, exclude = excludes)
        else:
            srcs = native.glob(_DEFAULT_SOURCE_PATTERNS, exclude = excludes)

    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps or [],
        includes = includes,
        copts = copts,
        linkstatic = linkstatic,
        alwayslink = alwayslink,
        visibility = visibility,
    )

def autonomy_module(
        name,
        module_path = None,
        has_proto = True,
        proto_glob = ["proto/*.proto"],
        proto_glob_recursive = None,
        proto_deps = None,
        srcs_glob = None,
        hdrs_glob = None,
        exclude_patterns = None,
        deps = None,
        extra_deps = None,
        cc_library_name = None,
        py_proto = False,
        visibility = ["//visibility:public"]):
    """
    Creates a complete autonomy module with proto and C++ library.

    This is the main macro that should be used in most BUILD.bazel files.

    Args:
        name: Base name for the module (e.g., "control", "driver")
        module_path: Module path for import_prefix (auto-detected if None)
        has_proto: Whether the module has proto files
        proto_glob: Glob pattern for proto files
        proto_glob_recursive: Recursive glob pattern for proto files
        proto_deps: Dependencies for proto_library
        srcs_glob: Custom glob patterns for C++ sources
        hdrs_glob: Custom glob patterns for C++ headers
        exclude_patterns: Additional patterns to exclude
        deps: Base dependencies (common, commsgs, etc.)
        extra_deps: Additional module-specific dependencies
        cc_library_name: Name of the cc_library (defaults to name)
        py_proto: Whether to generate Python proto bindings
        visibility: Visibility of targets
    """
    if module_path == None:
        package_name = native.package_name()
        module_path = _get_module_path(package_name)

    if cc_library_name == None:
        cc_library_name = name

    # Build dependencies
    all_deps = deps or []
    if extra_deps:
        all_deps = all_deps + extra_deps

    # Create proto library if needed
    proto_dep = None
    cc_proto_dep = None
    if has_proto:
        proto_name = name + "_proto"
        cc_proto_name = name + "_cc_proto"

        autonomy_proto_library(
            name = proto_name,
            proto_glob = proto_glob,
            proto_glob_recursive = proto_glob_recursive,
            import_prefix = module_path,
            deps = proto_deps,
            visibility = visibility,
        )

        autonomy_cc_proto_library(
            name = cc_proto_name,
            proto_dep = ":" + proto_name,
            visibility = visibility,
        )

        proto_dep = ":" + proto_name
        cc_proto_dep = ":" + cc_proto_name

        # Add cc_proto to library deps
        all_deps.append(cc_proto_dep)

        # Generate Python bindings if requested
        if py_proto:
            py_proto_library(
                name = name + "_py_proto",
                deps = [":" + proto_name],
                visibility = visibility,
            )

    # Build exclude patterns
    excludes = _DEFAULT_EXCLUDES[:]
    if has_proto:
        excludes.append("proto/**")
    if exclude_patterns:
        excludes.extend(exclude_patterns)

    # Create C++ library
    autonomy_cc_library(
        name = cc_library_name,
        srcs_glob = srcs_glob,
        hdrs_glob = hdrs_glob,
        exclude_patterns = excludes,
        deps = all_deps,
        visibility = visibility,
    )

def autonomy_binary(
        name,
        srcs,
        deps = None,
        copts = ["-std=c++17"],
        visibility = ["//visibility:public"]):
    """
    Creates a cc_binary for an autonomy module.

    Args:
        name: Name of the binary target
        srcs: Source files for the binary
        deps: Dependencies for the binary
        copts: Compiler options
        visibility: Visibility of the target
    """
    cc_binary(
        name = name,
        srcs = srcs,
        deps = deps or [],
        copts = copts,
        visibility = visibility,
    )
