# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json

from vntpa.cli.train import build_parser
from vntpa.engine import TrainConfig, build_jsonl_end_hook, build_jsonl_start_hook


def test_train_command():
    command = " ".join(TrainConfig(["--steps", "10"]).build_command())
    assert "thirdparty.pan.planner.train" in command
    assert "torch.distributed.run" in command


def test_cli_defaults_local_synthetic():
    args, unknown = build_parser().parse_known_args(["--synthetic", "--tiny"])
    assert args.synthetic and args.tiny
    assert unknown == []


def test_jsonl_hooks(tmp_path):
    config = TrainConfig(["--steps", "10"])
    log = tmp_path / "run.jsonl"
    build_jsonl_start_hook(str(log))(config)
    build_jsonl_end_hook(str(log))(config, 0)
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert [event["event"] for event in events] == ["start", "end"]
