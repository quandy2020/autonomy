import torch
import pytest

from depth import MDMModel
from depth.api import infer


class FakeModel:
    def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
        assert image.shape == (1, 3, 2, 3)
        assert image.dtype == torch.float32
        assert depth_in.shape == (1, 2, 3)
        return {"depth": depth_in + 1.0, "mask": depth_in > 0}


def test_infer_normalizes_shapes_and_invalid_depth():
    image = torch.full((2, 3, 3), 255, dtype=torch.uint8)
    raw_depth = torch.tensor([[1.0, 0.0, float("nan")], [2.0, -1.0, 3.0]])
    output = infer(FakeModel(), image, raw_depth, use_fp16=False)
    assert output["depth"].shape == (2, 3)
    assert torch.isfinite(output["raw_depth"]).all()
    assert output["raw_depth"][0, 1].item() == 0.0
    assert output["raw_depth"][1, 1].item() == 0.0


def test_infer_rejects_mismatched_spatial_dimensions():
    image = torch.zeros((3, 2, 3), dtype=torch.float32)
    raw_depth = torch.ones((2, 2), dtype=torch.float32)

    with pytest.raises(ValueError, match="matching spatial dimensions"):
        infer(FakeModel(), image, raw_depth, use_fp16=False)


def test_infer_batches_intrinsics():
    class IntrinsicsModel:
        def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
            assert intrinsics.shape == (1, 3, 3)
            return {"depth": depth_in, "mask": depth_in > 0}

    output = infer(
        IntrinsicsModel(),
        torch.zeros((3, 2, 3), dtype=torch.float32),
        torch.ones((2, 3), dtype=torch.float32),
        intrinsics=torch.eye(3),
        use_fp16=False,
    )

    assert output["depth"].shape == (2, 3)


def test_depth_exports_model_type():
    assert issubclass(MDMModel, torch.nn.Module)
