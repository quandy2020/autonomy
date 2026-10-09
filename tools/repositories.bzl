"""Local repositories that cannot be expressed as ``bazel_dep``.

BCR covers glog / protobuf / eigen / abseil / gflags / googletest.
This extension only materializes ``@autonomy_prefix`` — the CMake
install or build tree that provides ``libautomsgs`` / ``libautolink``.

Override:
  export AUTONOMY_PREFIX=/path/to/install
  # or: bazel build --repo_env=AUTONOMY_PREFIX=...
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

autonomy_prefix_repository = repository_rule(
    implementation = autonomy_prefix_impl,
    environ = ["AUTONOMY_PREFIX"],
    local = True,
)

def autonomy_repositories_impl(module_ctx):
    autonomy_prefix_repository(name = "autonomy_prefix")

autonomy_repositories = module_extension(
    implementation = autonomy_repositories_impl,
)
