"""ONNX Runtime session factory."""

from __future__ import annotations


def session(path: str, providers: tuple[str, ...] = ("CPUExecutionProvider",)):
    import onnxruntime as ort

    return ort.InferenceSession(path, providers=list(providers))
