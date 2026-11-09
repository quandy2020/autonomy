# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Single-process imitation loop over a frozen bank. No Habitat, no NCCL."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Subset

from thirdparty.pan.planner.dataset import PAD_TARGET, collate
from thirdparty.pan.planner.model import Planner, PlannerConfig
from thirdparty.pan.planner.train import TrainConfig, lr_scale, save_checkpoint
from thirdparty.pan.util.constants import N_ACTIONS
from vntpa.data.analyze import analyze
from vntpa.data.dataset import FrozenBankDataset
from vntpa.utils.tensorboard import EventLogger


def tiny_planner_config(chunk: int = 8) -> PlannerConfig:
    """CPU-sized planner; head dim 24 divides by 6 for spatial RoPE."""
    return PlannerConfig(
        summary_dim=96,
        model_dim=96,
        num_heads=4,
        map_layers=1,
        read_layers=1,
        ffn_ratio=2.0,
        chunk=chunk,
        act_emb_dim=32,
    )


def pick_device(name: str = "auto") -> torch.device:
    """Resolve ``auto`` to CUDA when a GPU is visible."""
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(name)


@dataclass
class LoopConfig:
    """Local trainer settings."""

    token_root: str
    train_bank: str
    val_token_root: str = ""
    val_bank: str = ""
    run_name: str = "vntpa-local"
    steps: int = 30000
    batch_scenes: int = 2
    accum: int = 1
    lr: float = 3e-4
    wd: float = 0.05
    warmup: int = 1000
    decay_frac: float = 0.15
    clip: float = 1.0
    seed: int = 0
    chunk: int = 8
    eta: int = 10
    max_kf: int = 64
    log_every: int = 10
    val_every: int = 100
    workers: int = 0
    tiny: bool = False
    device: str = "auto"
    tensorboard_dir: Optional[str] = None


@torch.no_grad()
def bank_ce(model: Planner, loader: DataLoader, device: torch.device) -> dict:
    """Teacher-forced CE and first-slot accuracy on a frozen bank."""
    model.eval()
    ce_sum = 0.0
    correct = 0.0
    counted = 0.0
    n_targets = 0.0
    slot_hits = None
    slot_seen = None
    use_amp = device.type == "cuda"
    for batch in loader:
        packed = batch.to(device)
        with torch.autocast(device.type, enabled=use_amp):
            logits = model(
                packed.kf_tokens,
                packed.kf_pose,
                packed.kf_valid,
                packed.query_pose,
                packed.goal_position,
                teacher_actions=packed.targets,
            )
        live = packed.targets != PAD_TARGET
        ce_sum += F.cross_entropy(
            logits.float().reshape(-1, N_ACTIONS),
            packed.targets.reshape(-1),
            ignore_index=PAD_TARGET,
            reduction="sum",
        ).item()
        pred = logits.argmax(-1)
        first = live[:, :, 0]
        correct += float(((pred == packed.targets)[:, :, 0] & first).sum())
        counted += float(first.sum())
        n_targets += float(live.sum())
        hits = ((pred == packed.targets) & live).sum(dim=(0, 1)).double().cpu()
        seen = live.sum(dim=(0, 1)).double().cpu()
        slot_hits = hits if slot_hits is None else slot_hits + hits
        slot_seen = seen if slot_seen is None else slot_seen + seen
    model.train()
    acc_slots = [] if slot_hits is None else (slot_hits / slot_seen.clamp_min(1.0)).tolist()
    return {
        "ce": ce_sum / max(n_targets, 1.0),
        "acc0": correct / max(counted, 1.0),
        "acc_slots": [round(float(value), 4) for value in acc_slots],
    }


def run(config: LoopConfig) -> Path:
    """Train and return the ``final.pt`` path."""
    device = pick_device(config.device)
    torch.manual_seed(config.seed)
    np.random.seed(config.seed)

    model_cfg = tiny_planner_config(config.chunk) if config.tiny else PlannerConfig(chunk=config.chunk)
    train_cfg = TrainConfig(
        run_name=config.run_name,
        steps=config.steps,
        lr=config.lr,
        wd=config.wd,
        warmup=min(config.warmup, max(config.steps // 10, 1)),
        decay_frac=config.decay_frac,
        batch_scenes=config.batch_scenes,
        accum=config.accum,
        clip=config.clip,
        seed=config.seed,
        wandb_project="",
        val_token_root=config.token_root,
        val_bank=config.val_bank or config.train_bank,
        model=model_cfg,
    )
    train_cfg = replace(
        train_cfg,
        data=replace(
            train_cfg.data,
            token_root=config.token_root,
            chunk=config.chunk,
            eta=config.eta,
            max_kf=config.max_kf,
            seed=config.seed,
        ),
    )

    dataset = FrozenBankDataset(
        config.train_bank,
        config.token_root,
        chunk=config.chunk,
        eta=config.eta,
        max_kf=config.max_kf,
        seed=config.seed,
    )
    loader = DataLoader(
        dataset,
        batch_size=min(config.batch_scenes, len(dataset)),
        shuffle=True,
        num_workers=config.workers,
        collate_fn=collate,
        drop_last=len(dataset) >= config.batch_scenes,
    )
    val_loader: Optional[DataLoader] = None
    val_bank = config.val_bank or config.train_bank
    val_root = config.val_token_root or config.token_root
    if val_bank:
        val_set = FrozenBankDataset(
            val_bank, val_root, chunk=config.chunk, eta=config.eta, max_kf=config.max_kf
        )
        val_loader = DataLoader(
            Subset(val_set, list(range(len(val_set)))),
            batch_size=min(8, max(len(val_set), 1)),
            shuffle=False,
            collate_fn=collate,
        )

    model = Planner(model_cfg).to(device)
    adamw = dict(lr=config.lr, weight_decay=config.wd, betas=(0.9, 0.95))
    try:
        opt = torch.optim.AdamW(model.parameters(), fused=device.type == "cuda", **adamw)
    except TypeError:
        opt = torch.optim.AdamW(model.parameters(), **adamw)
    run_dir = Path("logs") / config.run_name
    ckpt_dir = run_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "config.json").write_text(json.dumps(asdict(train_cfg), indent=2, default=str))
    logger = EventLogger(run_dir, config.tensorboard_dir)
    n_params = sum(param.numel() for param in model.parameters())
    report = analyze(
        config.token_root,
        config.train_bank,
        tensorboard_dir=None,
        out=run_dir / "data_report.json",
    )
    logger.log(
        0,
        "data",
        n_tours=report.n_tours,
        frames=report.frames,
        keyframes=report.keyframes,
        path_length_m=report.path_length_m,
        coverage_frac=report.coverage_frac,
        n_errors=len(report.errors),
    )
    if report.bank:
        logger.log(
            0,
            "bank",
            n_rows=report.bank["n_rows"],
            n_tours=report.bank["n_tours"],
            pad_frac=report.bank["pad_frac"],
            geo0=report.bank.get("geo0") or {},
            **{f"action_{name}": frac for name, frac in (report.bank.get("action_frac") or {}).items()},
        )
    logger.log(
        0,
        "setup",
        n_params=n_params,
        n_train=len(dataset),
        n_val=0 if val_loader is None else len(val_loader.dataset),
        device=str(device),
        tiny=int(config.tiny),
    )
    logger.text("setup/config", (run_dir / "config.json").read_text(), 0)

    iterator = iter(loader)
    model.train()
    use_amp = device.type == "cuda"
    best_ce = float("inf")
    ema = None
    started = time.time()
    last_metrics = {"val/ce": 0.0, "val/acc0": 0.0}
    for step in range(1, config.steps + 1):
        opt.zero_grad(set_to_none=True)
        step_loss = 0.0
        step_acc0 = 0.0
        step_counted = 0.0
        step_kf = 0.0
        step_live = 0.0
        for micro in range(config.accum):
            try:
                batch = next(iterator)
            except StopIteration:
                iterator = iter(loader)
                batch = next(iterator)
            packed = batch.to(device)
            with torch.autocast(device.type, enabled=use_amp):
                logits = model(
                    packed.kf_tokens,
                    packed.kf_pose,
                    packed.kf_valid,
                    packed.query_pose,
                    packed.goal_position,
                    teacher_actions=packed.targets,
                )
                loss = F.cross_entropy(
                    logits.float().reshape(-1, N_ACTIONS),
                    packed.targets.reshape(-1),
                    ignore_index=PAD_TARGET,
                )
            (loss / config.accum).backward()
            step_loss += float(loss.item()) / config.accum
            live = packed.targets != PAD_TARGET
            first = live[:, :, 0]
            step_acc0 += float(((logits.argmax(-1) == packed.targets)[:, :, 0] & first).sum())
            step_counted += float(first.sum())
            step_kf += float(packed.kf_valid.sum())
            step_live += float(live.sum())
        for group in opt.param_groups:
            group["lr"] = config.lr * lr_scale(train_cfg, step, 0)
        grad_norm = float(torch.nn.utils.clip_grad_norm_(model.parameters(), config.clip))
        opt.step()
        ema = step_loss if ema is None else 0.98 * ema + 0.02 * step_loss
        if step % config.log_every == 0 or step == config.steps:
            elapsed = max(time.time() - started, 1e-6)
            logger.log(
                step,
                "train",
                loss=round(ema, 4),
                loss_raw=round(step_loss, 4),
                acc0=round(step_acc0 / max(step_counted, 1.0), 4),
                lr=config.lr * lr_scale(train_cfg, step, 0),
                sps=round(step / elapsed, 3),
                grad_norm=round(grad_norm, 4),
                kf_mean=round(step_kf / max(config.accum, 1), 2),
                live_targets=round(step_live / max(config.accum, 1), 1),
            )
        if val_loader is not None and (step % config.val_every == 0 or step == config.steps):
            metrics = bank_ce(model, val_loader, device)
            logger.log(step, "val", **metrics)
            last_metrics = {"val/ce": float(metrics["ce"]), "val/acc0": float(metrics["acc0"])}
            save_checkpoint(str(ckpt_dir / "last.pt"), model, train_cfg, step, opt)
            if metrics["ce"] < best_ce:
                best_ce = metrics["ce"]
                save_checkpoint(str(ckpt_dir / "best.pt"), model, train_cfg, step, opt)

    final_path = ckpt_dir / "final.pt"
    save_checkpoint(str(final_path), model, train_cfg, config.steps, opt)
    logger.hparams(
        {
            "lr": config.lr,
            "steps": config.steps,
            "batch_scenes": config.batch_scenes,
            "tiny": config.tiny,
            "run_name": config.run_name,
        },
        {
            "best_ce": best_ce if best_ce < float("inf") else last_metrics["val/ce"],
            "final_ce": last_metrics["val/ce"],
            "final_acc0": last_metrics["val/acc0"],
        },
    )
    logger.close()
    print("training done", flush=True)
    return final_path
