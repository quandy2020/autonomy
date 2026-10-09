"""External repos for the standalone autodriver Bazel workspace.

Provides:
  @autonomy_prefix  — CMake install/build (libautolink.so, libautomsgs.so, headers)
  @local_autolink   — sibling ../autolink source headers + link the prefix .so
  @local_cli11      — sibling CLI11 headers (header-only)
  @system_eigen     — /usr/include/eigen3
"""

def _find_prefix(repository_ctx, workspace):
    candidates = []
    env_prefix = repository_ctx.os.environ.get("AUTONOMY_PREFIX", "").strip()
    if env_prefix:
        candidates.append(repository_ctx.path(env_prefix))

    # autodriver/ → ../../../install|build/autonomy (monorepo root)
    monorepo = workspace.get_child("..").get_child("..").get_child("..")
    candidates.append(monorepo.get_child("install").get_child("autonomy"))
    candidates.append(monorepo.get_child("build").get_child("autonomy"))
    # Fallback: under src/autonomy or autodriver itself
    parent = workspace.get_child("..")
    candidates.append(parent.get_child("install"))
    candidates.append(parent.get_child("build"))
    candidates.append(workspace.get_child("install"))
    candidates.append(workspace.get_child("build"))

    for candidate in candidates:
        lib = candidate.get_child("lib").get_child("libautolink.so")
        if lib.exists:
            return candidate
    return None

def _autonomy_prefix_impl(repository_ctx):
    workspace = repository_ctx.path(Label("//:MODULE.bazel")).dirname
    prefix = _find_prefix(repository_ctx, workspace)
    if prefix == None:
        fail(
            "autonomy_prefix: could not find libautolink.so. " +
            "Build/install autolink+automsgs with CMake first, or set " +
            "AUTONOMY_PREFIX to the prefix that contains lib/ and include/.\n" +
            "Typical: export AUTONOMY_PREFIX=$PWD/../../../install/autonomy",
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
    hdrs = glob(["prefix/include/automsgs/**"], allow_empty = True),
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
    hdrs = glob(["prefix/include/autolink/**"], allow_empty = True),
    includes = ["prefix/include"],
)
""",
    )
    repository_ctx.file("WORKSPACE", 'workspace(name = "autonomy_prefix")\n')

def _local_autolink_impl(repository_ctx):
    workspace = repository_ctx.path(Label("//:MODULE.bazel")).dirname
    autolink_root = workspace.get_child("..").get_child("autolink")
    headers = autolink_root.get_child("autolink")
    if not headers.exists:
        fail(
            "local_autolink: sibling headers not found at " +
            str(headers) +
            " (expected ../autolink/autolink next to autodriver)",
        )
    # Symlink only the header tree — avoid ../autolink/BUILD.bazel package boundary.
    repository_ctx.symlink(headers, "autolink")

    # Inline system linkopts so this external repo does not depend on //tools.
    repository_ctx.file(
        "BUILD.bazel",
        """
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autolink_headers",
    hdrs = glob(
        [
            "autolink/**/*.hpp",
            "autolink/**/*.h",
        ],
        exclude = [
            "**/test/**",
            "**/testing/**",
            "**/examples/**",
        ],
        allow_empty = True,
    ),
    includes = ["."],
    linkopts = [
        "/usr/local/lib/libglog.so",
        "/usr/local/lib/libprotobuf.so",
        "-Wl,-rpath,/usr/local/lib",
    ],
)

cc_library(
    name = "autolink",
    deps = [
        ":autolink_headers",
        "@autonomy_prefix//:autolink_so",
    ],
)
""",
    )
    repository_ctx.file("WORKSPACE", 'workspace(name = "local_autolink")\n')

def _local_cli11_impl(repository_ctx):
    workspace = repository_ctx.path(Label("//:MODULE.bazel")).dirname
    cli11_include = (
        workspace.get_child("..")
        .get_child("autolink")
        .get_child("thirdparty")
        .get_child("CLI11")
        .get_child("include")
    )
    if not cli11_include.exists:
        fail("local_cli11: CLI11 include not found at " + str(cli11_include))
    # Symlink only include/ — CLI11 root has its own BUILD.bazel / MODULE.bazel.
    repository_ctx.symlink(cli11_include, "include")
    repository_ctx.file(
        "BUILD.bazel",
        """
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cli11",
    hdrs = glob(["include/**/*.hpp"], allow_empty = True),
    includes = ["include"],
)
""",
    )
    repository_ctx.file("WORKSPACE", 'workspace(name = "local_cli11")\n')

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

local_autolink = repository_rule(
    implementation = _local_autolink_impl,
    local = True,
)

local_cli11 = repository_rule(
    implementation = _local_cli11_impl,
    local = True,
)

system_eigen = repository_rule(
    implementation = _system_eigen_impl,
    local = True,
)

def _autodriver_deps_impl(module_ctx):
    autonomy_prefix(name = "autonomy_prefix")
    local_autolink(name = "local_autolink")
    local_cli11(name = "local_cli11")
    system_eigen(name = "system_eigen")

autodriver_deps = module_extension(
    implementation = _autodriver_deps_impl,
)
