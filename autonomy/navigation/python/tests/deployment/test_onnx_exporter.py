from vntpa.runtime.deployment.tensorrt_exporter import export_tensorrt


def test_tensorrt_is_explicitly_unsupported():
    try:
        export_tensorrt()
    except Exception as exc:
        assert "TensorRT" in str(exc)
    else:
        raise AssertionError("expected UnsupportedFormatError")
