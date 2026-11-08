import pytest

from vntpa.runtime.deployment.onnx_validator import inspect_onnx
from vntpa.utils.errors import ModelNotFoundError


def test_missing_model():
    with pytest.raises(ModelNotFoundError):
        inspect_onnx("/nonexistent/vntpa.onnx")
