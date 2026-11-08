from vntpa.training.checkpoint import CheckpointCallback


def test_checkpoint_callback_noop_without_file(tmp_path):
    callback = CheckpointCallback(str(tmp_path / "missing.pt"), str(tmp_path / "out.pt"))
    callback.on_train_end(None, 0)
    assert not (tmp_path / "out.pt").exists()
