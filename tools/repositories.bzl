"""Bzlmod module extension for local repos that are not clean BCR pins.

Provides:
  @autonomy_prefix  — CMake automsgs / autolink / autonomy headers
  @autonomy_opencv   — system OpenCV from install_opencv.sh (/usr/local)

OpenCV is *not* a root ``bazel_dep``: BCR opencv 4.13+/5.x pulls eigen 5 and
protobuf ≥33, which conflicts with grpc@1.70 / protobuf@29 used by //autonomy.

Override:
  export AUTONOMY_PREFIX=/path/to/install
  export OPENCV_ROOT=/usr/local   # optional; defaults search /usr/local then /usr
"""

def workspace_root(repository_ctx):
    return repository_ctx.path(Label("//:MODULE.bazel")).dirname

def write_build(repository_ctx, build_label):
    repository_ctx.file("BUILD.bazel", repository_ctx.read(build_label))
    repository_ctx.file("WORKSPACE", "")

def find_cmake_prefix(repository_ctx):
    """Locate a prefix that contains libautomsgs.so or libautolink.so."""
    workspace = workspace_root(repository_ctx)
    candidates = []

    env = repository_ctx.os.environ.get("AUTONOMY_PREFIX", "").strip()
    if env:
        candidates.append(repository_ctx.path(env))

    candidates.extend([
        workspace.get_child("../..").get_child("install").get_child("autonomy"),
        workspace.get_child("../..").get_child("build").get_child("autonomy"),
        workspace.get_child("install"),
        workspace.get_child("build"),
    ])

    for path in candidates:
        lib = path.get_child("lib")
        if lib.get_child("libautomsgs.so").exists:
            return path
        if lib.get_child("libautolink.so").exists:
            return path
    return None

def autonomy_prefix_impl(repository_ctx):
    prefix = find_cmake_prefix(repository_ctx)
    if prefix == None:
        repository_ctx.file(
            "README",
            "No CMake prefix found. Set AUTONOMY_PREFIX to enable installed libs.\n",
        )
        write_build(
            repository_ctx,
            Label("//tools:prefix_stub.BUILD"),
        )
        return

    repository_ctx.symlink(prefix, "prefix")
    write_build(repository_ctx, Label("//tools:prefix.BUILD"))

def _has_opencv(path):
    """True if path looks like an OpenCV install prefix."""
    if not path.exists:
        return False
    lib = path.get_child("lib")
    for name in ("libopencv_core.so", "libopencv_core.dylib", "libopencv_core.a"):
        if lib.get_child(name).exists:
            return True
    # Multiarch / versioned sonames
    if lib.exists:
        # Best-effort: opencv4 headers are enough to declare the target.
        pass
    include = path.get_child("include")
    if include.get_child("opencv4").exists:
        return True
    if include.get_child("opencv2").exists:
        return True
    return False

def find_opencv_prefix(repository_ctx):
    """Locate OpenCV from install_deps (/usr/local) or OPENCV_ROOT."""
    candidates = []
    env = repository_ctx.os.environ.get("OPENCV_ROOT", "").strip()
    if env:
        candidates.append(repository_ctx.path(env))

    # install_opencv.sh default prefix
    candidates.extend([
        repository_ctx.path("/usr/local"),
        repository_ctx.path("/usr"),
    ])

    for path in candidates:
        if _has_opencv(path):
            return path
    return None

def autonomy_opencv_impl(repository_ctx):
    prefix = find_opencv_prefix(repository_ctx)
    if prefix == None:
        repository_ctx.file(
            "README",
            "OpenCV not found. Run scripts/install_deps (install_opencv.sh) " +
            "or set OPENCV_ROOT.\n",
        )
        write_build(repository_ctx, Label("//tools:opencv_stub.BUILD"))
        return

    # Symlink include/ + lib (linkopts use -Lprefix/lib).
    include = prefix.get_child("include")
    if include.exists:
        repository_ctx.symlink(include, "include")
    repository_ctx.file("prefix/.keep", "")
    lib = prefix.get_child("lib")
    if lib.exists:
        repository_ctx.symlink(lib, "prefix/lib")
    else:
        repository_ctx.file("prefix/lib/.keep", "")
    write_build(repository_ctx, Label("//tools:opencv.BUILD"))

autonomy_prefix_repository = repository_rule(
    implementation = autonomy_prefix_impl,
    environ = ["AUTONOMY_PREFIX"],
    local = True,
)

autonomy_opencv_repository = repository_rule(
    implementation = autonomy_opencv_impl,
    environ = ["OPENCV_ROOT"],
    local = True,
)

def autonomy_repositories_impl(module_ctx):
    autonomy_prefix_repository(name = "autonomy_prefix")
    autonomy_opencv_repository(name = "autonomy_opencv")

autonomy_repositories = module_extension(
    implementation = autonomy_repositories_impl,
)
