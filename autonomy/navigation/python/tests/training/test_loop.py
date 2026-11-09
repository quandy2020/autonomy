# Copyright 2026 The Navigation Authors (duyongquan)

import json
from pathlib import Path

import pytest

from vntpa.data.synthetic import write_corpus
from vntpa.engine.loop import LoopConfig, run


@pytest.mark.slow
def test_tiny_bank_train(tmp_path, monkeypatch):
    pytest.importorskip("torch")
    pytest.importorskip("h5py")
    paths = write_corpus(tmp_path, tours=2, frames=8, queries=4, seed=0)
    monkeypatch.chdir(tmp_path)
    final = run(
        LoopConfig(
            token_root=paths["tokens_train"],
            train_bank=paths["bank_train"],
            val_token_root=paths["tokens_val"],
            val_bank=paths["bank_val"],
            run_name="smoke",
            steps=2,
            batch_scenes=1,
            tiny=True,
            device="cpu",
            log_every=1,
            val_every=2,
        )
    )
    assert Path(final).is_file()
    log_dir = tmp_path / "logs" / "smoke"
    assert (log_dir / "metrics.jsonl").is_file()
    assert (log_dir / "data_report.json").is_file()
    lines = (log_dir / "metrics.jsonl").read_text().splitlines()
    kinds = {json.loads(line)["kind"] for line in lines}
    assert {"setup", "data", "train", "val"} <= kinds
