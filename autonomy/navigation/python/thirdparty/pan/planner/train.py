"""DDP imitation training of the planner, or of an arch registered by the baselines repository.

Usage: torchrun --nproc_per_node=2 -m pan.planner.train     # the defaults train the planner
       torchrun ... -m pan.planner.train --config logs/<run>/config.json [--resume] [flags]
       torchrun ... -m baselines train --arch lstm [flags]

A run is one `TrainConfig`, saved in every checkpoint and as logs/<run>/config.json;
`--config` replays it, with flags overriding its values. Validation uses the frozen bank
`val_bank`: CE on `val_rows` evenly spaced rows every `val_every` steps, closed loop on
`cl_rows` rows every `cl_every` steps.
"""
from __future__ import annotations

import argparse
import contextlib
import dataclasses
import itertools
import json
import os
import time
from dataclasses import dataclass, field
from typing import List

import numpy as np
import torch
import torch.distributed as dist
import torch.nn.functional as F
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data import DataLoader, DistributedSampler, Subset

from thirdparty.pan.util.config import add_config_args, arg, config_from_args
from thirdparty.pan.util.constants import N_ACTIONS

from .dataset import PAD_TARGET, DataConfig, EvalBank, TourDataset, TourStore, collate, worker_init_fn
from .model import Planner, PlannerConfig, compile_rope

AMP = torch.bfloat16                    # autocast dtype of training and evaluation

# arch -> (model class, its config class, the prefix of that config's flags, the config fields
# the arch fixes). Importing the baselines repository registers its archs here.
ARCHS = {"attn": (Planner, PlannerConfig, "", {})}


@dataclass
class TrainConfig:
    run_name: str = arg("pan", "run directory under logs/")
    val_token_root: str = arg("dataset/habitat/tokens/val",
                              "validation token stores")
    val_bank: str = arg("dataset/habitat/banks/bank_val.npz",
                        "frozen validation bank (CE and closed loop)")
    arch: str = arg("attn", "attn = the Planner; the separate baselines repository registers more")
    seed: int = arg(0, "seed of the initialization and the data streams")
    wandb_project: str = arg("pan", "'' disables wandb")
    tensorboard_dir: str = arg("", "TensorBoard run directory; '' disables TensorBoard")
    # optimization
    steps: int = arg(30000, "optimizer steps")
    lr: float = arg(3e-4, "peak learning rate")
    wd: float = arg(0.05, "AdamW weight decay")
    warmup: int = arg(1000, "linear warmup steps")
    decay_frac: float = arg(0.15, "final fraction of the steps over which the LR decays linearly "
                                  "to a 0.05 floor")
    batch_scenes: int = arg(12, "items per micro-batch per rank")
    accum: int = arg(2, "micro-batches per optimizer step")
    clip: float = arg(1.0, "grad-norm clip")
    init_from: str = arg("", "when not resuming, start from this checkpoint's model weights and step "
                             "with a fresh optimizer; the LR ramps back up over init_rewarm")
    init_rewarm: int = arg(500, "steps of linear LR ramp after an --init-from start")
    workers: int = arg(8, "data-loader workers per rank")
    prefetch_factor: int = arg(2, "batches queued per worker")
    compile_rope: bool = arg(True, "torch.compile the RoPE kernels (attn)")
    # evaluation
    val_every: int = arg(1000, "steps between CE validations, each writing last.pt")
    cl_every: int = arg(2000, "steps between closed-loop evaluations, each writing step_*.pt")
    log_every: int = arg(100, "steps between training logs")
    val_rows: int = arg(512, "bank rows for CE validation, evenly spaced over the tour-sorted bank; "
                             "0 = all")
    cl_rows: int = arg(256, "bank rows for the closed-loop evaluation, spaced likewise; 0 = all")
    exec_horizon: int = arg(4, "actions executed per decoded chunk in the closed loop")
    data: DataConfig = field(default_factory=DataConfig)
    model: object = None                # the arch's config; its class default when None

    def __post_init__(self):
        """Accepts the nested dicts of a saved config; applies the arch's fixed model fields and
        sets the data's chunk, zero_tokens and seed from the model and the run."""
        assert self.arch in ARCHS, f"unknown arch {self.arch!r} (registered: {sorted(ARCHS)})"
        _, cfg_cls, _, fixed = ARCHS[self.arch]
        data = DataConfig(**self.data) if isinstance(self.data, dict) else self.data
        model = cfg_cls(**self.model) if isinstance(self.model, dict) else self.model or cfg_cls()
        self.model = model = dataclasses.replace(model, **fixed)
        self.data = dataclasses.replace(data, chunk=model.chunk, seed=self.seed,
                                        zero_tokens=getattr(model, "zero_content", False))


def parse_config(argv=None):
    """-> (TrainConfig, resume). Flags default to the `--config` file's values when given."""
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config")
    pre.add_argument("--arch")
    known, _ = pre.parse_known_args(argv)
    if known.arch and known.arch not in ARCHS:
        pre.error(f"unknown --arch {known.arch} (registered: {sorted(ARCHS)}; the baselines "
                  f"repository adds its archs: python -m baselines train)")
    base = TrainConfig()
    if known.config:
        with open(known.config) as fh:
            base = TrainConfig(**json.load(fh))
    if known.arch and known.arch != base.arch:
        base = TrainConfig(**{**dataclasses.asdict(base), "arch": known.arch, "model": None})
    _, cfg_cls, prefix, _ = ARCHS[base.arch]
    p = argparse.ArgumentParser(description="DDP imitation training of a planner.")
    p.add_argument("--config", help="a run's config.json to replay; flags override its values")
    p.add_argument("--resume", action="store_true", help="continue from <run>/checkpoints/last.pt")
    add_config_args(p, TrainConfig, defaults=base)
    add_config_args(p, DataConfig, defaults=base.data)
    add_config_args(p, cfg_cls, prefix, defaults=base.model)
    args = p.parse_args(argv)
    if not (args.run_name and args.token_root and args.val_token_root):
        p.error("--run-name, --token-root and --val-token-root are required")
    cfg = config_from_args(TrainConfig, args, data=config_from_args(DataConfig, args),
                           model=config_from_args(cfg_cls, args, prefix))
    return cfg, args.resume


def build_model(cfg: TrainConfig) -> torch.nn.Module:
    return ARCHS[cfg.arch][0](cfg.model)


def save_checkpoint(path: str, model, cfg: TrainConfig, step: int, opt=None, **state) -> None:
    """Model and config; with the optimizer and the run state, a resumable checkpoint."""
    ckpt = dict(model=model.state_dict(), config=dataclasses.asdict(cfg), step=step)
    if opt is not None:
        ckpt.update(opt=opt.state_dict(), **state)
    torch.save(ckpt, path)


def load_planner(path: str):
    """A checkpoint -> (its model, its run's TrainConfig)."""
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    cfg = TrainConfig(**ckpt["config"])
    model = build_model(cfg)
    model.load_state_dict(ckpt["model"])
    return model, cfg


def lr_scale(cfg: TrainConfig, step: int, init_step: int) -> float:
    """Warmup-stable-decay (linear warmup, hold, a linear decay to 0.05 over the final `decay_frac`
    of the steps), times a linear ramp over the first `init_rewarm` steps after an --init-from start."""
    decay_start = cfg.steps * (1.0 - cfg.decay_frac)
    if step < cfg.warmup:
        scale = (step + 1) / cfg.warmup
    elif step < decay_start:
        scale = 1.0
    else:
        scale = 1.0 - 0.95 * min((step - decay_start) / max(cfg.steps - decay_start, 1), 1.0)
    if cfg.init_from and cfg.init_rewarm > 0:
        scale *= min(1.0, (step - init_step + 1) / cfg.init_rewarm)
    return scale


def spaced_rows(n_rows: int, k: int) -> np.ndarray:
    """k row indices evenly spaced over a tour-sorted bank; all rows when k <= 0 or k >= n_rows."""
    if k <= 0 or k >= n_rows:
        return np.arange(n_rows)
    return np.unique(np.linspace(0, n_rows - 1, k).round().astype(np.int64))


@torch.no_grad()
def effective_rank(x: torch.Tensor) -> float:
    """Centered effective (entropy) rank of a (K, D) feature matrix."""
    x = x.float()
    x = x - x.mean(0, keepdim=True)
    s = torch.linalg.svdvals(x)
    p = (s * s) / (s * s).sum().clamp_min(1e-12)
    return float(torch.exp(-(p * p.clamp_min(1e-12).log()).sum()))


def batches(loader: DataLoader, sampler: DistributedSampler):
    for epoch in itertools.count(1):
        sampler.set_epoch(epoch)
        yield from loader


@torch.no_grad()
def validate(model, loader: DataLoader, device, rank_batches: int = 4) -> dict:
    """Teacher-forced CE and per-slot accuracy over real targets; median effective rank of the
    encoded keyframe entries over the first `rank_batches` batches."""
    model.eval()
    ce_sum, correct, counted, ranks = 0.0, 0.0, 0.0, []
    for bi, batch in enumerate(loader):
        b = batch.to(device)
        with torch.autocast(device.type, dtype=AMP):
            memory = model.encode_map(model.summarize(b.kf_tokens, b.kf_valid), b.kf_pose,
                                      b.kf_valid, b.goal_position)
            logits = model.read(memory, b.query_pose, b.goal_position, teacher_actions=b.targets)
        logits = logits.float()
        live = b.targets != PAD_TARGET
        ce_sum += F.cross_entropy(logits.reshape(-1, N_ACTIONS), b.targets.reshape(-1),
                                  ignore_index=PAD_TARGET, reduction="sum").item()
        correct = correct + ((logits.argmax(-1) == b.targets) & live).sum((0, 1)).double()
        counted = counted + live.sum((0, 1)).double()
        if bi < rank_batches:
            entries = memory[0][:, :b.kf_valid.shape[1]]
            for i in range(entries.shape[0]):
                e = entries[i][b.kf_valid[i]]
                if e.shape[0] > 1:
                    ranks.append(effective_rank(e))
    model.train()
    n_targets = float(counted.sum())
    return dict(ce=ce_sum / max(n_targets, 1.0), acc_slots=(correct / counted.clamp_min(1)).tolist(),
                entry_rank=float(np.median(ranks)) if ranks else 0.0)


def make_logger(cfg: TrainConfig, run_dir: str, n_params: float, world: int):
    """-> log(step, kind, **values): to stdout, <run>/metrics.jsonl and, when enabled, wandb."""
    path, run = os.path.join(run_dir, "metrics.jsonl"), None
    writer = None
    if cfg.tensorboard_dir:
        from torch.utils.tensorboard import SummaryWriter
        writer = SummaryWriter(cfg.tensorboard_dir)
    if cfg.wandb_project:
        import wandb
        id_path = os.path.join(run_dir, "wandb_id.txt")      # stable across --resume
        if not os.path.exists(id_path):
            with open(id_path, "w") as fh:
                fh.write(f"{cfg.run_name.replace('/', '-')}-{wandb.util.generate_id()}")
        with open(id_path) as fh:
            run = wandb.init(project=cfg.wandb_project, name=cfg.run_name, id=fh.read().strip(),
                             resume="allow", dir=run_dir,
                             config=dict(dataclasses.asdict(cfg), params_m=round(n_params, 2), world=world))

    def log(step: int, kind: str, **values) -> None:
        record = dict(kind=kind, **values, step=step)
        print(json.dumps(record), flush=True)
        with open(path, "a") as fh:
            fh.write(json.dumps(record) + "\n")
        flat = {}
        for k, v in values.items():
            if isinstance(v, (list, tuple)):
                flat.update({f"{kind}/{k}_{i}": float(x) for i, x in enumerate(v)})
            else:
                flat[f"{kind}/{k}"] = v
        if run is not None:
            run.log(flat, step=step)
        if writer is not None:
            for key, value in flat.items():
                try:
                    writer.add_scalar(key, float(value), step)
                except (TypeError, ValueError):
                    pass
            writer.flush()
    return log


def make_validation(cfg: TrainConfig, log):
    """Rank 0's evaluations on the validation bank -> run(step, model, device, best): logs
    those due at `step`, updates the running best metrics in `best`, and returns the
    checkpoints to write (step_*.pt are model-only)."""
    from .eval import closed_loop_eval        # deferred: eval imports this module
    store = TourStore(dataclasses.replace(cfg.data, token_root=cfg.val_token_root,
                                          kf_noise_pos=0.0, kf_noise_yaw=0.0))     # clean keyframe poses
    bank = EvalBank(cfg.val_bank, store)
    ce_rows, cl_rows = spaced_rows(len(bank), cfg.val_rows), spaced_rows(len(bank), cfg.cl_rows)
    loader = DataLoader(Subset(bank, ce_rows.tolist()), batch_size=8, num_workers=2,
                        collate_fn=collate)
    print(f"val bank {cfg.val_bank}: CE on {len(ce_rows)} rows every {cfg.val_every}, "
          f"closed loop on {len(cl_rows)} rows every {cfg.cl_every}", flush=True)

    def run(step: int, model, device, best: dict) -> List[str]:
        names = []
        if step % cfg.val_every == 0:
            v = validate(model, loader, device)
            log(step, "val", ce=round(v["ce"], 4), acc0=round(v["acc_slots"][0], 4),
                acc_slots=[round(a, 3) for a in v["acc_slots"]], entry_rank=round(v["entry_rank"], 2))
            names.append("last.pt")
            if v["ce"] < best["best_ce"]:
                best["best_ce"] = v["ce"]
                names.append("best.pt")
        if step % cfg.cl_every == 0:
            r = closed_loop_eval(model, store, bank, device, rows=cl_rows, exec_horizon=cfg.exec_horizon)
            log(step, "cl", val_success=round(r["success"], 3), val_spl=round(r["spl"], 3),
                val_stopped=round(r["stopped"], 3), entry_rank=round(r["entry_rank_median"], 2))
            names.append(f"step_{step:06d}.pt")
            if r["success"] > best["best_cl"]:
                best["best_cl"] = r["success"]
                names.append("best_cl.pt")
        return names
    return run


def main():
    cfg, resume = parse_config()
    dist.init_process_group("nccl")
    rank, world = dist.get_rank(), dist.get_world_size()
    local = int(os.environ["LOCAL_RANK"])
    device = torch.device(f"cuda:{local}")
    torch.cuda.set_device(device)
    torch.manual_seed(cfg.seed + rank)

    run_dir = os.path.join("logs", cfg.run_name)
    ckpt_dir = os.path.join(run_dir, "checkpoints")
    if rank == 0:
        os.makedirs(ckpt_dir, exist_ok=True)

    data = TourDataset(dataclasses.replace(cfg.data, seed=cfg.seed + 7919 * rank))
    sampler = DistributedSampler(data, world, rank, shuffle=True, drop_last=True)
    loader = DataLoader(data, batch_size=cfg.batch_scenes, sampler=sampler,
                        num_workers=cfg.workers, collate_fn=collate, pin_memory=True,
                        persistent_workers=cfg.workers > 0,
                        prefetch_factor=cfg.prefetch_factor if cfg.workers else None,
                        worker_init_fn=worker_init_fn)
    if cfg.arch == "attn" and cfg.compile_rope:
        compile_rope()
    model = build_model(cfg).to(device)
    n_params = sum(p.numel() for p in model.parameters()) / 1e6
    ddp = DDP(model, device_ids=[local])
    opt = torch.optim.AdamW(model.parameters(), lr=cfg.lr, weight_decay=cfg.wd,
                            betas=(0.9, 0.95), fused=True)
    step, init_step, best = 0, 0, dict(best_ce=float("inf"), best_cl=-1.0, targets_seen=0)
    started = None
    last = os.path.join(ckpt_dir, "last.pt")
    if resume and os.path.exists(last):
        ckpt = torch.load(last, map_location="cpu", weights_only=False)
        model.load_state_dict(ckpt["model"])
        opt.load_state_dict(ckpt["opt"])
        step, init_step, best = ckpt["step"], ckpt.get("init_step", 0), {k: ckpt[k] for k in best}
        started = f"resumed at step {step}"
    elif cfg.init_from:
        ckpt = torch.load(cfg.init_from, map_location="cpu", weights_only=False)
        source = TrainConfig(**ckpt["config"])
        assert (source.arch, source.model) == (cfg.arch, cfg.model), (
            f"--init-from {cfg.init_from} is a {source.arch} {source.model}, "
            f"not this run's {cfg.arch} {cfg.model}")
        model.load_state_dict(ckpt["model"])
        step = init_step = ckpt["step"]
        started = (f"initialized from {cfg.init_from} at step {step} "
                   f"(fresh optimizer, LR ramp over {cfg.init_rewarm} steps)")
    if rank == 0:
        if started:
            print(started, flush=True)
        print(f"{type(model).__name__} {n_params:.1f}M params | {len(data)} tours | world {world}",
              flush=True)
        with open(os.path.join(run_dir, "config.json"), "w") as fh:
            json.dump(dataclasses.asdict(cfg), fh, indent=2)
        log = make_logger(cfg, run_dir, n_params, world)
        validation = make_validation(cfg, log)

    ddp.train()
    batch_iter = batches(loader, sampler)
    start_step, t_start = step, time.time()
    t_log = t_start
    seen = torch.tensor(best["targets_seen"], dtype=torch.int64, device=device)
    ema, data_wait = None, 0.0
    while step < cfg.steps:
        opt.zero_grad(set_to_none=True)
        step_loss = 0.0
        for micro in range(cfg.accum):
            t_wait = time.perf_counter()
            batch = next(batch_iter)
            data_wait += time.perf_counter() - t_wait
            b = batch.to(device)                          # pinned -> async copy
            # DDP arms the all-reduce in forward: no_sync must wrap both passes
            with contextlib.nullcontext() if micro == cfg.accum - 1 else ddp.no_sync():
                with torch.autocast(device.type, dtype=AMP):
                    logits = ddp(b.kf_tokens, b.kf_pose, b.kf_valid, b.query_pose, b.goal_position,
                                 teacher_actions=b.targets)
                    loss = F.cross_entropy(logits.reshape(-1, N_ACTIONS), b.targets.reshape(-1),
                                           ignore_index=PAD_TARGET)
                (loss / cfg.accum).backward()
            step_loss += loss.item() / cfg.accum        # a sync per micro-batch bounds CPU run-ahead
            seen += (b.targets[:, :, 0] != PAD_TARGET).sum() * world
        for group in opt.param_groups:
            group["lr"] = cfg.lr * lr_scale(cfg, step, init_step)
        norm = torch.nn.utils.clip_grad_norm_(model.parameters(), cfg.clip)
        opt.step()
        step += 1
        ema = step_loss if ema is None else 0.98 * ema + 0.02 * step_loss

        if rank == 0 and step % cfg.log_every == 0:
            live = b.targets[:, :, 0] != PAD_TARGET
            acc0 = ((logits[:, :, 0].float().argmax(-1) == b.targets[:, :, 0]) & live
                    ).float().sum() / live.float().sum().clamp_min(1)
            now = time.time()
            log(step, "train", loss=round(ema, 4), acc0=round(acc0.item(), 4),
                lr=cfg.lr * lr_scale(cfg, step, init_step),
                grad_norm=round(float(norm), 2), sps=round((step - start_step) / (now - t_start), 3),
                targets_seen=int(seen), step_ms=round(1e3 * (now - t_log) / cfg.log_every, 1),
                data_wait_ms=round(1e3 * data_wait / cfg.log_every, 1),
                mem_alloc_gb=round(torch.cuda.max_memory_allocated(device) / 1e9, 1),
                mem_reserved_gb=round(torch.cuda.memory_reserved(device) / 1e9, 1))
            torch.cuda.reset_peak_memory_stats(device)
            t_log, data_wait = now, 0.0

        if step % cfg.val_every == 0 or step % cfg.cl_every == 0:
            if rank == 0:
                best["targets_seen"] = int(seen)
                for name in validation(step, model, device, best):
                    state = {} if name.startswith("step_") else dict(opt=opt, init_step=init_step, **best)
                    save_checkpoint(os.path.join(ckpt_dir, name), model, cfg, step, **state)
                ddp.train()
            dist.barrier()

    if rank == 0:
        best["targets_seen"] = int(seen)
        save_checkpoint(os.path.join(ckpt_dir, "final.pt"), model, cfg, step, opt,
                        init_step=init_step, **best)
        if cfg.wandb_project:
            import wandb
            wandb.finish()
        print("training done", flush=True)
    dist.destroy_process_group()


if __name__ == "__main__":
    main()
