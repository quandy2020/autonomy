"""Shared Bazel helpers."""

def clean_dep(dep):
    """Sanitize a dependency label for submodule / Bzlmod use."""
    return str(Label(dep))
