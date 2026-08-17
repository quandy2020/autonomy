"""Test bootstrap: tolerate protobuf gencode/runtime version skew."""
from __future__ import annotations

try:
    from google.protobuf import runtime_version as _runtime_version

    def _allow_gencode(*_args, **_kwargs):
        return None

    _runtime_version.ValidateProtobufRuntimeVersion = _allow_gencode  # type: ignore[method-assign]
except Exception:
    pass
