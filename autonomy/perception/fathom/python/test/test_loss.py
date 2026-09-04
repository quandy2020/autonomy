import pytest
import torch
import torch.nn.functional as F

from train.loss import fathom_loss


def test_loss_ignores_invalid_target_pixels():
    pred = torch.tensor([[1.5, 100.0]])
    target = torch.tensor([[1.0, 0.0]])
    valid = torch.tensor([[True, False]])

    result = fathom_loss(pred, target, valid)

    assert result.valid_pixels == 1
    assert torch.isclose(result.total, F.smooth_l1_loss(pred[:, :1], target[:, :1]))
    assert torch.isclose(result.depth, F.smooth_l1_loss(pred[:, :1], target[:, :1]))
    assert result.mask is None


def test_loss_rejects_all_invalid_batches():
    pred = torch.tensor([[1.5, 2.0]])
    target = torch.tensor([[0.0, 0.0]])
    valid = torch.tensor([[False, False]])

    with pytest.raises(ValueError, match="valid target pixels"):
        fathom_loss(pred, target, valid)


def test_loss_uses_valid_mask_as_binary_cross_entropy_target():
    pred = torch.tensor([[1.5, 2.0]])
    target = torch.tensor([[1.0, 0.0]])
    valid = torch.tensor([[True, False]])
    pred_mask = torch.tensor([[0.2, -0.4]])

    result = fathom_loss(pred, target, valid, pred_mask=pred_mask, mask_weight=0.25)

    expected_mask = F.binary_cross_entropy_with_logits(pred_mask, valid.float())
    expected_depth = F.smooth_l1_loss(pred[:, :1], target[:, :1])
    assert torch.isclose(result.mask, expected_mask)
    assert torch.isclose(result.total, expected_depth + 0.25 * expected_mask)
