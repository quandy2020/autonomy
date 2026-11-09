# Copyright 2026 The Navigation Authors (duyongquan)

import json

from vntpa.utils.tensorboard import _flatten, write_curves_from_jsonl


def test_flatten_does_not_nest_slashes():
    tags = dict(_flatten("data", {"frames": {"mean": 8.0, "max": 16.0}, "n_tours": 2}))
    assert "data/frames_mean" in tags
    assert "data/n_tours" in tags
    assert all("/" not in tag.split("/", 1)[1] for tag in tags)


def test_write_curves_from_jsonl(tmp_path):
    jsonl = tmp_path / "metrics.jsonl"
    jsonl.write_text(
        json.dumps({"kind": "data", "step": 0, "n_tours": 4})
        + "\n"
        + json.dumps({"kind": "train", "step": 50, "loss": 1.2, "acc0": 0.4})
        + "\n"
        + json.dumps({"kind": "val", "step": 100, "ce": 2.1, "acc_slots": [0.5, 0.6]})
        + "\n"
    )
    out = tmp_path / "curves"
    count = write_curves_from_jsonl(jsonl, out)
    assert count >= 4
    assert any(out.glob("events.out.tfevents*"))
