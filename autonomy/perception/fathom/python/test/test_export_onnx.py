import pytest

torch = pytest.importorskip("torch")

from depth.export_onnx import _FixedProfileModel, export_onnx


class _FakeDepthModel(torch.nn.Module):
    def forward(
        self, image: torch.Tensor, num_tokens: int, depth: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        assert num_tokens == 12
        refined_depth = depth + image.mean(dim=1)
        return {
            "depth_reg": refined_depth,
            "mask": torch.ones_like(refined_depth),
        }


class _FakeDepthModelWithoutMask(torch.nn.Module):
    def forward(
        self, image: torch.Tensor, num_tokens: int, depth: torch.Tensor
    ) -> dict[str, torch.Tensor]:
        assert num_tokens == 12
        return {"depth_reg": depth + image.mean(dim=1)}


def test_fixed_profile_model_supplies_all_one_validity_when_model_omits_mask():
    image = torch.linspace(0.0, 1.0, steps=3 * 4 * 6).reshape(1, 3, 4, 6)
    raw_depth = torch.linspace(1.0, 2.0, steps=4 * 6).reshape(1, 4, 6)

    refined_depth, validity = _FixedProfileModel(_FakeDepthModelWithoutMask(), 12)(
        image, raw_depth
    )

    assert refined_depth.shape == (1, 4, 6)
    assert validity.shape == (1, 4, 6)
    assert validity.dtype == torch.float32
    assert torch.equal(validity, torch.ones((1, 4, 6)))


def test_export_onnx_creates_checked_model_with_pytorch_parity(tmp_path):
    onnx = pytest.importorskip("onnx", reason="ONNX export validation requires onnx")
    ort = pytest.importorskip(
        "onnxruntime", reason="ONNX export validation requires onnxruntime"
    )
    model = _FakeDepthModel().eval()
    output = export_onnx(
        model,
        tmp_path / "fake-depth.onnx",
        height=4,
        width=6,
        num_tokens=12,
    )

    onnx.checker.check_model(onnx.load(output))

    image = torch.linspace(0.0, 1.0, steps=3 * 4 * 6).reshape(1, 3, 4, 6)
    raw_depth = torch.linspace(1.0, 2.0, steps=4 * 6).reshape(1, 4, 6)
    expected = model(image, num_tokens=12, depth=raw_depth)
    session = ort.InferenceSession(output.as_posix(), providers=["CPUExecutionProvider"])
    refined_depth, validity = session.run(
        ["refined_depth", "validity"],
        {
            "image": image.numpy(),
            "raw_depth": raw_depth.numpy(),
        },
    )

    torch.testing.assert_close(
        torch.from_numpy(refined_depth),
        expected["depth_reg"],
        rtol=1e-5,
        atol=1e-6,
    )
    torch.testing.assert_close(
        torch.from_numpy(validity),
        expected["mask"],
        rtol=1e-5,
        atol=1e-6,
    )
