import torch
from torch import nn
from torch.utils.data import DataLoader

from train.trainer import save_checkpoint, train_epoch


class FakeDepthModel(nn.Module):
    """Small trainable stand-in for the released depth model."""

    def __init__(self) -> None:
        super().__init__()
        self.depth_scale = nn.Parameter(torch.tensor(0.5))
        self.mask_bias = nn.Parameter(torch.tensor(0.0))

    def forward(
        self, image: torch.Tensor, depth: torch.Tensor, num_tokens: int
    ) -> dict[str, torch.Tensor]:
        del image, num_tokens
        return {
            "depth_reg": depth * self.depth_scale,
            "mask": torch.sigmoid(torch.ones_like(depth) * self.mask_bias),
        }


def test_train_epoch_updates_fake_depth_model_parameters():
    model = FakeDepthModel()
    loader = DataLoader(
        [
            {
                "image": torch.zeros((3, 2, 2)),
                "raw_depth": torch.ones((2, 2)),
                "target_depth": torch.full((2, 2), 2.0),
                "valid_mask": torch.ones((2, 2), dtype=torch.bool),
            }
        ],
        batch_size=1,
    )
    optimizer = torch.optim.SGD(model.parameters(), lr=0.1)
    original_depth_scale = model.depth_scale.detach().clone()

    losses = train_epoch(model, loader, optimizer, device="cpu", mask_weight=0.25)

    assert not torch.equal(model.depth_scale.detach(), original_depth_scale)
    assert set(losses) == {"total", "depth", "mask"}
    assert all(isinstance(value, float) for value in losses.values())


def test_save_checkpoint_preserves_released_model_keys_and_training_state(tmp_path):
    model = FakeDepthModel()
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.01)
    model_config = {"encoder": {"name": "fake"}}
    checkpoint_path = tmp_path / "fine-tuned.pt"

    save_checkpoint(checkpoint_path, model, model_config, optimizer, step=7)

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    assert {"model_config", "model", "optimizer", "step"} <= checkpoint.keys()
    assert checkpoint["model_config"] == model_config
    assert checkpoint["step"] == 7

    restored = FakeDepthModel()
    restored.load_state_dict(checkpoint["model"])
    for original, reloaded in zip(model.parameters(), restored.parameters()):
        torch.testing.assert_close(original, reloaded)
