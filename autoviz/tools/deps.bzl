"""External repositories for the Autoviz workspace.

  @eigen — /usr/include/eigen3
  @qt6   — /usr/include/<multiarch>/qt6 (+ system .so via linkopts)
  @ogre  — prebuilt Ogre 1.12 prefix (install tree, not FetchContent build dir)

Override the Ogre prefix:
  export AUTOVIZ_OGRE_ROOT=/path/to/ogre-1.12
  # or: bazel build --repo_env=AUTOVIZ_OGRE_ROOT=...
"""

def _workspace(ctx):
    return ctx.path(Label("//:MODULE.bazel")).dirname

def _write_build(ctx, build_label):
    ctx.file("BUILD.bazel", ctx.read(build_label))
    ctx.file("WORKSPACE", "")

def _is_complete_ogre_prefix(path):
    """True when lib + full public headers are present (not a CMake build/ tree)."""
    lib = path.get_child("lib").get_child("libOgreMain.so")
    if not lib.exists:
        return False
    # Install / vendor layout: include/OGRE/Ogre.h
    ogre_h = path.get_child("include").get_child("OGRE").get_child("Ogre.h")
    if ogre_h.exists:
        return True
    # Flat include/Ogre.h (some prefixes)
    return path.get_child("include").get_child("Ogre.h").exists

def _find_ogre(ctx, workspace):
    """Locate a complete Ogre 1.12.x *install* prefix."""
    candidates = []

    env = ctx.os.environ.get("AUTOVIZ_OGRE_ROOT", "").strip()
    if env:
        candidates.append(ctx.path(env))

    # ROS vendor (proper install prefix with include/OGRE/*).
    candidates.append(ctx.path("/opt/ros/humble/opt/rviz_ogre_vendor"))
    candidates.append(ctx.path("/opt/ros/jazzy/opt/rviz_ogre_vendor"))

    # Optional staged install under the monorepo.
    root = workspace.get_child("..").get_child("..").get_child("..")
    candidates.append(root.get_child("install").get_child("ogre"))
    candidates.append(root.get_child("install").get_child("autonomy"))
    candidates.append(ctx.path("/usr/local"))

    for path in candidates:
        if _is_complete_ogre_prefix(path):
            return path
    return None

def _eigen_impl(ctx):
    include = ctx.path("/usr/include/eigen3")
    if not include.exists:
        fail("Eigen3 headers not found at /usr/include/eigen3")
    ctx.symlink(include, "eigen3")
    _write_build(ctx, Label("//tools:eigen.BUILD"))

def _qt6_impl(ctx):
    candidates = [
        ctx.path("/usr/include/x86_64-linux-gnu/qt6"),
        ctx.path("/usr/include/aarch64-linux-gnu/qt6"),
        ctx.path("/usr/include/qt6"),
    ]
    include = None
    for path in candidates:
        if path.exists:
            include = path
            break
    if include == None:
        fail(
            "Qt6 headers not found.\n" +
            "Install qt6 base/tools, e.g. qt6-base-dev qt6-tools-dev.",
        )
    ctx.symlink(include, "include")
    _write_build(ctx, Label("//tools:qt6.BUILD"))

def _ogre_impl(ctx):
    workspace = _workspace(ctx)
    prefix = _find_ogre(ctx, workspace)
    if prefix == None:
        fail(
            "Could not find a complete Ogre 1.12 install prefix.\n" +
            "Need lib/libOgreMain.so and include/OGRE/Ogre.h.\n" +
            "Note: CMake FetchContent `*-build` dirs are NOT enough " +
            "(headers live in `*-src`). Prefer:\n" +
            "  export AUTOVIZ_OGRE_ROOT=/opt/ros/humble/opt/rviz_ogre_vendor\n" +
            "or point AUTOVIZ_OGRE_ROOT at a staged Ogre install.",
        )
    ctx.symlink(prefix, "prefix")
    _write_build(ctx, Label("//tools:ogre.BUILD"))

_eigen = repository_rule(implementation = _eigen_impl, local = True)
_qt6 = repository_rule(implementation = _qt6_impl, local = True)
_ogre = repository_rule(
    implementation = _ogre_impl,
    local = True,
    environ = ["AUTOVIZ_OGRE_ROOT"],
)

def _deps_impl(ctx):
    _eigen(name = "eigen")
    _qt6(name = "qt6")
    _ogre(name = "ogre")

deps = module_extension(implementation = _deps_impl)
