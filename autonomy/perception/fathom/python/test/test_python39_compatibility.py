"""Compatibility contracts for the declared Python 3.9 minimum."""

import pytest

from depth import api as depth_api
from depth import export_onnx
from train import dataset, loss, trainer


@pytest.mark.parametrize(
    "annotated_object",
    [
        depth_api.load_model,
        depth_api.infer,
        export_onnx.export_onnx,
        dataset.RgbdManifestDataset.__init__,
        loss.LossOutput,
        loss.fathom_loss,
        trainer.save_checkpoint,
        trainer.train_epoch,
    ],
)
def test_locally_authored_annotations_are_postponed(annotated_object):
    annotations = annotated_object.__annotations__

    assert annotations
    assert all(isinstance(annotation, str) for annotation in annotations.values())
