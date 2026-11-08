"""Product-level exception hierarchy."""

from __future__ import annotations


class VNTError(Exception):
    """Base class for all vntpa errors."""


class InvalidConfigError(VNTError, ValueError):
    """Configuration is missing or internally inconsistent."""


class ModelNotFoundError(VNTError, FileNotFoundError):
    """A local planner/DeFM model could not be found."""


class ModelValidationError(VNTError, ValueError):
    """ONNX model exists but does not match the expected contract."""


class UnsupportedFormatError(VNTError, ValueError):
    """Requested export format is not implemented."""
