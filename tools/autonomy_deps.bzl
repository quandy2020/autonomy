"""Locate the CMake install/build prefix and system header trees."""

def _autonomy_prefix_impl(repository_ctx):
    workspace = repository_ctx.path(Label("//:MODULE.bazel")).dirname
    candidates = []

    env_prefix = repository_ctx.os.environ.get("AUTONOMY_PREFIX", "").strip()
    if env_prefix:
        candidates.append(repository_ctx.path(env_prefix))

    # src/autonomy → ../../install/autonomy or ../../build/autonomy
    candidates.append(workspace.get_child("../..").get_child("install").get_child("autonomy"))
    candidates.append(workspace.get_child("../..").get_child("build").get_child("autonomy"))
    candidates.append(workspace.get_child("install"))
    candidates.append(workspace.get_child("build"))

    prefix = None
    for candidate in candidates:
        # Prefer libautomsgs.so — autolink itself is built from source via Bazel.
        lib = candidate.get_child("lib").get_child("libautomsgs.so")
        if lib.exists:
            prefix = candidate
            break
        # Fall back to a CMake-built libautolink.so for older prefixes.
        lib = candidate.get_child("lib").get_child("libautolink.so")
        if lib.exists:
            prefix = candidate
            break

    if prefix == None:
        fail(
            "autonomy_prefix: could not find libautomsgs.so (or libautolink.so). " +
            "Build/install automsgs with CMake first, or set " +
            "AUTONOMY_PREFIX to the prefix that contains lib/ and include/.",
        )

    repository_ctx.symlink(prefix, "prefix")

    repository_ctx.file(
        "BUILD.bazel",
        """
load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_import(
    name = "autolink_so",
    shared_library = "prefix/lib/libautolink.so",
)

cc_import(
    name = "automsgs_so",
    shared_library = "prefix/lib/libautomsgs.so",
)

cc_library(
    name = "automsgs_headers",
    hdrs = glob(
        ["prefix/include/automsgs/**"],
        allow_empty = True,
    ),
    includes = ["prefix/include"],
)

cc_library(
    name = "automsgs",
    deps = [
        ":automsgs_headers",
        ":automsgs_so",
    ],
)

cc_library(
    name = "autolink_installed_headers",
    hdrs = glob(
        ["prefix/include/autolink/**"],
        allow_empty = True,
    ),
    includes = ["prefix/include"],
)
""",
    )

    repository_ctx.file(
        "WORKSPACE",
        'workspace(name = "autonomy_prefix")\n',
    )

def _system_eigen_impl(repository_ctx):
    eigen_root = repository_ctx.path("/usr/include/eigen3")
    if not eigen_root.exists:
        fail("system_eigen: /usr/include/eigen3 not found (install libeigen3-dev)")
    repository_ctx.symlink(eigen_root, "eigen3")
    repository_ctx.file(
        "BUILD.bazel",
        """
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigen",
    hdrs = glob(["eigen3/**"], allow_empty = True),
    includes = ["eigen3"],
)
""",
    )
    repository_ctx.file("WORKSPACE", 'workspace(name = "system_eigen")\n')

autonomy_prefix = repository_rule(
    implementation = _autonomy_prefix_impl,
    environ = ["AUTONOMY_PREFIX"],
    local = True,
)

system_eigen = repository_rule(
    implementation = _system_eigen_impl,
    local = True,
)

def _autonomy_deps_impl(module_ctx):
    autonomy_prefix(name = "autonomy_prefix")
    system_eigen(name = "system_eigen")

autonomy_deps = module_extension(
    implementation = _autonomy_deps_impl,
)
