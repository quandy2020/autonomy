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
    raw_depth = torch.tensor([[1.0, 0.0, float("nan")], [2.0, -1.0, float("inf")]])
    output = infer(FakeModel(), image, raw_depth, use_fp16=False)
    assert output["depth"].shape == (2, 3)
    assert torch.isfinite(output["raw_depth"]).all()
    assert output["raw_depth"][0, 1].item() == 0.0
    assert output["raw_depth"][0, 2].item() == 0.0
    assert output["raw_depth"][1, 1].item() == 0.0
    assert output["raw_depth"][1, 2].item() == 0.0


def test_infer_normalizes_uint8_rgb_values():
    class ImageValueModel:
        def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
            return {"depth": depth_in + image.mean(), "mask": depth_in > 0}

    output = infer(
        ImageValueModel(),
        torch.full((2, 3, 3), 255, dtype=torch.uint8),
        torch.ones((2, 3), dtype=torch.float32),
        use_fp16=False,
    )

    assert torch.equal(output["depth"], torch.full((2, 3), 2.0))


def test_infer_rejects_out_of_range_float_rgb():
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        infer(
            FakeModel(),
            torch.full((3, 2, 3), 255.0, dtype=torch.float32),
            torch.ones((2, 3), dtype=torch.float32),
            use_fp16=False,
        )


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


def test_infer_preserves_batched_inputs_and_outputs():
    class BatchedModel:
        def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
            assert image.shape == (2, 3, 2, 3)
            assert depth_in.shape == (2, 2, 3)
            assert intrinsics.shape == (2, 3, 3)
            return {"depth": depth_in + image[:, 0], "mask": depth_in > 0}

    image = torch.stack(
        [
            torch.zeros((3, 2, 3), dtype=torch.float32),
            torch.ones((3, 2, 3), dtype=torch.float32),
        ]
    )
    raw_depth = torch.tensor(
        [
            [[1.0, 0.0, float("nan")], [2.0, 3.0, 4.0]],
            [[5.0, -1.0, 6.0], [float("inf"), 7.0, 8.0]],
        ]
    )
    intrinsics = torch.stack([torch.eye(3), 2.0 * torch.eye(3)])

    output = infer(
        BatchedModel(),
        image,
        raw_depth,
        intrinsics=intrinsics,
        use_fp16=False,
    )

    assert output["depth"].shape == (2, 2, 3)
    assert output["mask"].shape == (2, 2, 3)
    assert output["raw_depth"].shape == (2, 2, 3)
    assert torch.isfinite(output["raw_depth"]).all()
    assert output["raw_depth"][0, 0, 2].item() == 0.0
    assert output["raw_depth"][1, 0, 1].item() == 0.0
    assert output["raw_depth"][1, 1, 0].item() == 0.0


def test_infer_rejects_image_and_depth_batch_mismatch():
    with pytest.raises(ValueError, match="batch dimensions"):
        infer(
            FakeModel(),
            torch.zeros((2, 3, 2, 3), dtype=torch.float32),
            torch.ones((3, 2, 3), dtype=torch.float32),
            use_fp16=False,
        )


def test_infer_rejects_intrinsics_batch_mismatch():
    with pytest.raises(ValueError, match="intrinsics batch dimension"):
        infer(
            FakeModel(),
            torch.zeros((2, 3, 2, 3), dtype=torch.float32),
            torch.ones((2, 2, 3), dtype=torch.float32),
            intrinsics=torch.eye(3).unsqueeze(0),
            use_fp16=False,
        )


@pytest.mark.parametrize("intrinsics", [torch.eye(2), torch.ones((2, 3, 2))])
def test_infer_rejects_intrinsics_with_invalid_matrix_shape(intrinsics):
    with pytest.raises(ValueError, match="intrinsics"):
        infer(
            FakeModel(),
            torch.zeros((3, 2, 3), dtype=torch.float32),
            torch.ones((2, 3), dtype=torch.float32),
            intrinsics=intrinsics,
            use_fp16=False,
        )


def test_infer_moves_batched_inputs_to_model_device():
    class DeviceAwareModel:
        device = torch.device("meta")

        def infer(self, image, depth_in, intrinsics, apply_mask, use_fp16):
            refined_depth = depth_in + image[:, 0] + intrinsics[:, 0, 0, None, None]
            return {"depth": refined_depth, "mask": depth_in > 0}

    model = DeviceAwareModel()
    output = infer(
        model,
        torch.zeros((3, 2, 3), dtype=torch.float32),
        torch.ones((2, 3), dtype=torch.float32),
        intrinsics=torch.eye(3),
        use_fp16=False,
    )

    assert output["depth"].device == model.device
    assert output["raw_depth"].device == model.device


def test_depth_exports_model_type():
    assert issubclass(MDMModel, torch.nn.Module)
