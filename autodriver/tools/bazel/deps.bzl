"""External repositories for the autodriver workspace (module extension).

  @prefix   — CMake prefix with libautolink.so / libautomsgs.so
  @autolink — sibling ../autolink source headers (+ link @prefix)
  @cli11    — sibling CLI11 headers

Eigen comes from BCR: bazel_dep(name = "eigen") → @eigen//:eigen

Override the CMake prefix:
  export AUTONOMY_PREFIX=/path/to/install/autonomy
  # or: bazel build --repo_env=AUTONOMY_PREFIX=...
"""

def _workspace(ctx):
    return ctx.path(Label("//:MODULE.bazel")).dirname

def _write_build(ctx, build_label):
    ctx.file("BUILD.bazel", ctx.read(build_label))
    ctx.file("WORKSPACE", "")

def _find_prefix(ctx, workspace):
    """Locate a prefix that contains lib/libautolink.so."""
    candidates = []

    env = ctx.os.environ.get("AUTONOMY_PREFIX", "").strip()
    if env:
        candidates.append(ctx.path(env))

    # autodriver/ → monorepo root → install|build/autonomy
    root = workspace.get_child("..").get_child("..").get_child("..")
    candidates.append(root.get_child("install").get_child("autonomy"))
    candidates.append(root.get_child("build").get_child("autonomy"))

    # Fallbacks: sibling or local trees
    parent = workspace.get_child("..")
    for base in (parent, workspace):
        candidates.append(base.get_child("install"))
        candidates.append(base.get_child("build"))

    for path in candidates:
        if path.get_child("lib").get_child("libautolink.so").exists:
            return path
    return None

def _prefix_impl(ctx):
    workspace = _workspace(ctx)
    prefix = _find_prefix(ctx, workspace)
    if prefix == None:
        fail(
            "Could not find libautolink.so.\n" +
            "Build/install autolink+automsgs first, then either:\n" +
            "  export AUTONOMY_PREFIX=$PWD/../../../install/autonomy\n" +
            "or place libs under install/autonomy or build/autonomy.",
        )
    ctx.symlink(prefix, "prefix")
    _write_build(ctx, Label("//tools/bazel:prefix.BUILD"))

def _autolink_impl(ctx):
    # Symlink only the header tree — avoid ../autolink/BUILD.bazel package boundary.
    headers = _workspace(ctx).get_child("..").get_child("autolink").get_child("autolink")
    if not headers.exists:
        fail("Sibling autolink headers not found at %s" % headers)
    ctx.symlink(headers, "autolink")
    _write_build(ctx, Label("//tools/bazel:autolink.BUILD"))

def _cli11_impl(ctx):
    # Symlink only include/ — CLI11 root ships its own MODULE.bazel.
    include = (
        _workspace(ctx)
            .get_child("..")
            .get_child("autolink")
            .get_child("thirdparty")
            .get_child("CLI11")
            .get_child("include")
    )
    if not include.exists:
        fail("CLI11 include not found at %s" % include)
    ctx.symlink(include, "include")
    _write_build(ctx, Label("//tools/bazel:cli11.BUILD"))

_prefix = repository_rule(
    implementation = _prefix_impl,
    environ = ["AUTONOMY_PREFIX"],
    local = True,
)

_autolink = repository_rule(implementation = _autolink_impl, local = True)
_cli11 = repository_rule(implementation = _cli11_impl, local = True)

def _deps_impl(module_ctx):
    _prefix(name = "prefix")
    _autolink(name = "autolink")
    _cli11(name = "cli11")

deps = module_extension(implementation = _deps_impl)
