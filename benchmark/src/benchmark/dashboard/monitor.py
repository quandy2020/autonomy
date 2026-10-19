# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Background job runner + SSE event bus for live dashboard updates."""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
import uuid
from collections.abc import Generator
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
from typing import Any

from benchmark.common.config import load_config, load_yaml, package_root
from benchmark.evaluation.evaluator import Evaluator
from benchmark.tasks.manager import run_task

_LOG = logging.getLogger(__name__)


@dataclass
class JobEvent:
    ts: float
    kind: str
    message: str
    job_id: str = ""
    payload: dict[str, Any] = field(default_factory=dict)

    def to_sse(self) -> str:
        data = {
            "ts": self.ts,
            "kind": self.kind,
            "message": self.message,
            "job_id": self.job_id,
            "payload": self.payload,
        }
        return f"data: {json.dumps(data, default=str)}\n\n"


@dataclass
class JobState:
    job_id: str
    status: str = "queued"  # queued | running | success | failed | cancelled
    tasks: list[str] = field(default_factory=list)
    task_states: dict[str, str] = field(
        default_factory=dict
    )  # pending|running|done|failed|cancelled
    current_task: str = ""
    progress: float = 0.0
    summary: dict[str, Any] = field(default_factory=dict)
    curves: dict[str, Any] = field(default_factory=dict)  # task → control curves
    map_tracks: dict[str, Any] = field(default_factory=dict)  # task → path/goal for BicMap
    error: str = ""
    started_at: float = 0.0
    finished_at: float = 0.0
    events: list[dict[str, Any]] = field(default_factory=list)


class MonitorHub:
    """Singleton-ish hub: one active job, many SSE subscribers."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subscribers: list[queue.Queue[JobEvent | None]] = []
        self._job: JobState | None = None
        self._history: list[dict[str, Any]] = []
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self._metrics: dict[str, Any] = {
            "uptime_s": 0.0,
            "jobs_total": 0,
            "jobs_ok": 0,
            "jobs_failed": 0,
            "last_heartbeat": time.time(),
        }
        self._started = time.time()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._metrics["uptime_s"] = round(time.time() - self._started, 1)
            self._metrics["last_heartbeat"] = time.time()
            job = asdict(self._job) if self._job else None
            return {
                "metrics": dict(self._metrics),
                "job": job,
                "history": list(self._history),
            }

    def _archive_job(self, job: JobState) -> None:
        """Keep a short in-memory history of finished jobs (incl. curves for restore)."""
        entry = {
            "job_id": job.job_id,
            "status": job.status,
            "tasks": list(job.tasks),
            "task_states": dict(job.task_states),
            "progress": job.progress,
            "error": job.error,
            "started_at": job.started_at,
            "finished_at": job.finished_at,
            "summary": job.summary,
            "curves": dict(job.curves),
            "map_tracks": dict(job.map_tracks),
            "has_curves": bool(job.curves),
        }
        self._history.insert(0, entry)
        self._history = self._history[:8]

    def get_history_entry(self, job_id: str) -> dict[str, Any] | None:
        with self._lock:
            for entry in self._history:
                if entry.get("job_id") == job_id:
                    return dict(entry)
            if self._job and self._job.job_id == job_id:
                return asdict(self._job)
        return None

    def subscribe(self) -> queue.Queue[JobEvent | None]:
        q: queue.Queue[JobEvent | None] = queue.Queue(maxsize=256)
        with self._lock:
            self._subscribers.append(q)
        return q

    def unsubscribe(self, q: queue.Queue[JobEvent | None]) -> None:
        with self._lock:
            if q in self._subscribers:
                self._subscribers.remove(q)

    def publish(self, event: JobEvent) -> None:
        with self._lock:
            if self._job is not None:
                self._job.events.append(
                    {
                        "ts": event.ts,
                        "kind": event.kind,
                        "message": event.message,
                        "payload": event.payload,
                    }
                )
                # Cap history for memory.
                if len(self._job.events) > 200:
                    self._job.events = self._job.events[-200:]
            dead: list[queue.Queue[JobEvent | None]] = []
            for q in self._subscribers:
                try:
                    q.put_nowait(event)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                self._subscribers.remove(q)

    def cancel(self) -> bool:
        with self._lock:
            if self._job is None or self._job.status not in ("queued", "running"):
                return False
        self._cancel.set()
        self.publish(
            JobEvent(
                time.time(),
                "warn",
                "Cancel requested",
                job_id=self._job.job_id if self._job else "",
            )
        )
        return True

    def start_batch(
        self,
        *,
        tasks: list[str],
        config_path: str | None = None,
        mode: str | None = None,
        episodes: int | None = None,
        collect: dict[str, Any] | None = None,
        planner: str | None = None,
        controller: str | None = None,
        model_path: str | None = None,
        simulator: str | None = None,
        humans: str | None = None,
        record: bool = False,
        params: dict[str, Any] | None = None,
        replay: dict[str, Any] | None = None,
        execution: str = "sync",
        kind: str = "orchestration",
        named_calls: list[dict[str, Any]] | None = None,
        schedule: dict[str, Any] | None = None,
    ) -> JobState:
        with self._lock:
            if self._thread and self._thread.is_alive():
                raise RuntimeError("a job is already running")
            kind = (kind or "orchestration").strip().lower()
            execution = "async" if str(execution).lower() == "async" else "sync"
            run_tasks = list(tasks)
            if kind == "named":
                calls = list(named_calls or [])
                if not calls:
                    raise ValueError("select at least one named gRPC task")
                run_tasks = [
                    str(c.get("id") or c.get("method") or f"rpc-{i}")
                    for i, c in enumerate(calls)
                ]
            elif kind == "schedule":
                steps = list((schedule or {}).get("steps") or run_tasks)
                if not steps:
                    raise ValueError("schedule requires at least one step")
                run_tasks = [
                    str(s.get("task") if isinstance(s, dict) else s) for s in steps
                ]
            else:
                if not run_tasks:
                    raise ValueError("select at least one task")
                if record and "collect" not in run_tasks:
                    run_tasks.append("collect")
            job_id = uuid.uuid4().hex[:10]
            self._job = JobState(
                job_id=job_id,
                tasks=run_tasks,
                status="queued",
                task_states={t: "pending" for t in run_tasks},
            )
            self._cancel.clear()
            self._metrics["jobs_total"] += 1
            target = self._run_named_batch if kind == "named" else self._run_batch
            kwargs: dict[str, Any] = {
                "job_id": job_id,
                "tasks": run_tasks,
                "config_path": config_path,
                "mode": mode,
                "episodes": episodes,
                "collect": collect or {},
                "planner": planner,
                "controller": controller,
                "model_path": model_path,
                "simulator": simulator,
                "humans": humans,
                "params": params or {},
                "replay": replay or {},
                "execution": execution,
                "kind": kind,
                "named_calls": list(named_calls or []),
                "schedule": dict(schedule or {}),
            }
            self._thread = threading.Thread(
                target=target,
                kwargs=kwargs,
                name=f"dashboard-job-{job_id}",
                daemon=True,
            )
            self._thread.start()
            return self._job

    def _mark_running(self, job_id: str) -> None:
        with self._lock:
            if self._job and self._job.job_id == job_id:
                self._job.status = "running"
                self._job.started_at = time.time()

    def _finish_success(self, job_id: str, results_all: dict[str, Any]) -> None:
        curves_snap: dict[str, Any] = {}
        history_snap: list[dict[str, Any]] = []
        with self._lock:
            if self._job and self._job.job_id == job_id:
                self._job.status = "success"
                self._job.progress = 1.0
                self._job.summary = results_all
                self._job.finished_at = time.time()
                self._job.current_task = ""
                curves_snap = dict(self._job.curves)
                self._archive_job(self._job)
            history_snap = list(self._history)
            self._metrics["jobs_ok"] += 1
        self.publish(
            JobEvent(
                time.time(),
                "job_done",
                "Batch completed",
                job_id=job_id,
                payload={
                    "results": results_all,
                    "curves": curves_snap,
                    "history": history_snap,
                },
            )
        )

    def _run_one_sim_task(
        self,
        *,
        job_id: str,
        task_name: str,
        index: int,
        total: int,
        path: str,
        mode: str | None,
        episodes: int | None,
        collect: dict[str, Any],
        planner: str | None,
        controller: str | None,
        model_path: str | None,
        simulator: str | None,
        humans: str | None,
        params: dict[str, Any] | None,
        replay: dict[str, Any] | None,
    ) -> tuple[str, dict[str, Any]]:
        if self._cancel.is_set():
            raise InterruptedError("cancelled")
        with self._lock:
            if self._job and self._job.job_id == job_id:
                self._job.current_task = task_name
                self._job.progress = index / max(total, 1)
                self._job.task_states[task_name] = "running"
                task_states_snap = dict(self._job.task_states)
            else:
                task_states_snap = {}
        self.publish(
            JobEvent(
                time.time(),
                "task_start",
                f"Running task `{task_name}` ({index + 1}/{total})",
                job_id=job_id,
                payload={
                    "task": task_name,
                    "index": index,
                    "task_states": task_states_snap,
                },
            )
        )
        from benchmark.dashboard.catalog import params_to_overrides

        overrides = [f"task.name={task_name}"]
        merges: list[dict[str, Any]] = []
        if mode:
            overrides.append(f"task.mode={mode}")
        if episodes is not None:
            overrides.append(f"task.episodes={int(episodes)}")
        if planner:
            planner_block: dict[str, Any] = {"name": planner}
            planner_cfg = package_root() / "configs" / "planners" / f"{planner}.yaml"
            if planner_cfg.is_file():
                try:
                    planner_block = {
                        **load_yaml(planner_cfg),
                        "name": planner,
                        "config": f"configs/planners/{planner}.yaml",
                    }
                except Exception:  # noqa: BLE001
                    planner_block = {
                        "name": planner,
                        "config": f"configs/planners/{planner}.yaml",
                    }
            if controller is not None:
                planner_block["controller"] = controller
            if model_path:
                planner_block["model_path"] = model_path
            merges.append({"planner": planner_block})
        else:
            if controller is not None:
                overrides.append(f"planner.controller={controller}")
            if model_path:
                overrides.append(f"planner.model_path={model_path}")
        if simulator:
            overrides.append(f"simulator.name={simulator}")
        if humans:
            overrides.append(f"humans.name={humans}")
            overrides.append("humans.enabled=true")
        if task_name == "collect" and collect:
            for key in (
                "include_laser",
                "include_image",
                "lerobot",
                "pose",
                "velocity",
                "action",
                "output",
                "fps",
                "episode_frames",
                "episodes",
            ):
                if key not in collect or collect[key] is None:
                    continue
                val = collect[key]
                if isinstance(val, bool):
                    overrides.append(f"task.{key}={'true' if val else 'false'}")
                else:
                    overrides.append(f"task.{key}={val}")
        if task_name == "replay" and replay:
            for key in ("source", "rate", "loop", "dry_run"):
                if key not in replay or replay[key] is None:
                    continue
                val = replay[key]
                if isinstance(val, bool):
                    overrides.append(f"task.{key}={'true' if val else 'false'}")
                else:
                    overrides.append(f"task.{key}={val}")
        overrides.extend(params_to_overrides(params))
        # Apply non-planner merges first; planner section is replaced wholesale
        # so keys from the previous planner YAML do not leak across.
        other_merges = [m for m in merges if "planner" not in m]
        cfg = load_config(path, overrides=overrides, merges=other_merges)
        for m in merges:
            if "planner" in m and isinstance(m["planner"], dict):
                cfg.raw["planner"] = dict(m["planner"])
        t0 = time.time()
        episode_results = run_task(cfg)
        summary = Evaluator().summarize(episode_results)
        curves = _extract_curves(episode_results)
        track = _extract_map_track(episode_results, task_name)
        elapsed = round(time.time() - t0, 3)
        result = {
            "summary": summary,
            "elapsed_s": elapsed,
            "curves": curves,
            "map_track": track,
        }
        with self._lock:
            if self._job and self._job.job_id == job_id:
                self._job.curves[task_name] = curves
                self._job.map_tracks[task_name] = track
                self._job.task_states[task_name] = "done"
                self._job.progress = (index + 1) / max(total, 1)
                task_states_snap = dict(self._job.task_states)
            else:
                task_states_snap = {}
        self.publish(
            JobEvent(
                time.time(),
                "task_done",
                f"Task `{task_name}` finished in {elapsed}s",
                job_id=job_id,
                payload={
                    "task": task_name,
                    "summary": summary,
                    "elapsed_s": elapsed,
                    "curves": curves.get("latest") or {},
                    "curve_stats": (curves.get("latest") or {}).get("stats") or {},
                    "map_track": track,
                    "task_states": task_states_snap,
                },
            )
        )
        return task_name, result

    def _run_batch(
        self,
        *,
        job_id: str,
        tasks: list[str],
        config_path: str | None,
        mode: str | None,
        episodes: int | None,
        collect: dict[str, Any],
        planner: str | None = None,
        controller: str | None = None,
        model_path: str | None = None,
        simulator: str | None = None,
        humans: str | None = None,
        params: dict[str, Any] | None = None,
        replay: dict[str, Any] | None = None,
        execution: str = "sync",
        kind: str = "orchestration",
        named_calls: list[dict[str, Any]] | None = None,
        schedule: dict[str, Any] | None = None,
    ) -> None:
        del named_calls  # unused in sim batch
        path = config_path or str(package_root() / "configs" / "default.yaml")
        self._mark_running(job_id)
        sched = schedule or {}
        loop_forever = bool(sched.get("loop"))
        repeat = max(1, int(sched.get("repeat") or 1))
        interval_s = float(sched.get("interval_s") or 0)
        step_interval_s = max(0.0, float(sched.get("step_interval_s") or 0))
        delay_s = max(0.0, float(sched.get("delay_s") or 0))
        start_at_raw = sched.get("start_at") or ""
        end_at_raw = sched.get("end_at") or ""
        raw_steps = sched.get("steps") or [{"task": t} for t in tasks]
        steps: list[dict[str, Any]] = []
        for s in raw_steps:
            if isinstance(s, dict):
                name = str(s.get("task") or "").strip()
                if not name:
                    continue
                ep_raw = s.get("episodes")
                step_ep = int(ep_raw) if ep_raw is not None else episodes
                steps.append({"task": name, "episodes": step_ep})
            else:
                name = str(s).strip()
                if name:
                    steps.append({"task": name, "episodes": episodes})
        if not steps:
            steps = [{"task": t, "episodes": episodes} for t in tasks if str(t).strip()]
        step_tasks = [s["task"] for s in steps]
        if loop_forever and interval_s <= 0:
            interval_s = 1.0  # avoid tight spin
        self.publish(
            JobEvent(
                time.time(),
                "info",
                f"Job {job_id} started ({kind}, {execution})",
                job_id=job_id,
                payload={
                    "tasks": step_tasks,
                    "steps": steps,
                    "execution": execution,
                    "kind": kind,
                    "repeat": None if loop_forever else repeat,
                    "loop": loop_forever,
                    "interval_s": interval_s,
                    "step_interval_s": step_interval_s,
                    "delay_s": delay_s,
                    "start_at": start_at_raw or None,
                    "end_at": end_at_raw or None,
                },
            )
        )

        def _parse_local_ts(raw: str) -> float | None:
            try:
                from datetime import datetime

                return datetime.fromisoformat(str(raw)).timestamp()
            except ValueError:
                return None

        def _wait_until(deadline: float, label: str) -> None:
            while time.time() < deadline:
                if self._cancel.is_set():
                    raise InterruptedError("cancelled")
                left = deadline - time.time()
                if left <= 0:
                    break
                time.sleep(min(0.25, left))
            if label:
                self.publish(
                    JobEvent(
                        time.time(),
                        "info",
                        label,
                        job_id=job_id,
                    )
                )

        results_all: dict[str, Any] = {}
        try:
            end_ts: float | None = None
            if end_at_raw:
                end_ts = _parse_local_ts(str(end_at_raw))
                if end_ts is None:
                    self.publish(
                        JobEvent(
                            time.time(),
                            "warn",
                            f"无效的结束时间: {end_at_raw}",
                            job_id=job_id,
                        )
                    )
                elif end_ts <= time.time():
                    self.publish(
                        JobEvent(
                            time.time(),
                            "warn",
                            f"结束时间已过 · {end_at_raw}，跳过执行",
                            job_id=job_id,
                        )
                    )
                    self._finish_success(job_id, results_all)
                    return

            # Absolute start time (datetime-local → local wall clock).
            if start_at_raw:
                start_ts = _parse_local_ts(str(start_at_raw))
                if start_ts is None:
                    self.publish(
                        JobEvent(
                            time.time(),
                            "warn",
                            f"无效的开始时间: {start_at_raw}",
                            job_id=job_id,
                        )
                    )
                else:
                    if end_ts is not None and end_ts <= start_ts:
                        self.publish(
                            JobEvent(
                                time.time(),
                                "warn",
                                "结束时间必须晚于开始时间",
                                job_id=job_id,
                            )
                        )
                        self._finish_success(job_id, results_all)
                        return
                    if start_ts > time.time():
                        _wait_until(start_ts, f"开始时间到达 · {start_at_raw}")
            if delay_s > 0:
                _wait_until(time.time() + delay_s, f"延迟 {delay_s:g}s 结束，开始执行")

            def _past_end() -> bool:
                return end_ts is not None and time.time() >= end_ts

            round_i = 0
            stopped_by_end = False
            while True:
                if not loop_forever and round_i >= repeat:
                    break
                if self._cancel.is_set():
                    raise InterruptedError("cancelled")
                if _past_end():
                    stopped_by_end = True
                    break
                n = len(steps)
                base_common = dict(
                    job_id=job_id,
                    total=n,
                    path=path,
                    mode=mode,
                    collect=collect,
                    planner=planner,
                    controller=controller,
                    model_path=model_path,
                    simulator=simulator,
                    humans=humans,
                    params=params,
                    replay=replay,
                )
                if execution == "async" and n > 1:
                    with ThreadPoolExecutor(max_workers=min(4, n)) as pool:
                        futs = {
                            pool.submit(
                                self._run_one_sim_task,
                                task_name=str(step["task"]),
                                index=i,
                                episodes=step.get("episodes"),
                                **base_common,
                            ): str(step["task"])
                            for i, step in enumerate(steps)
                        }
                        for fut in as_completed(futs):
                            if self._cancel.is_set():
                                raise InterruptedError("cancelled")
                            name, result = fut.result()
                            key = name if name not in results_all else f"{name}#{round_i}"
                            results_all[key] = result
                else:
                    for i, step in enumerate(steps):
                        if _past_end():
                            stopped_by_end = True
                            break
                        name, result = self._run_one_sim_task(
                            task_name=str(step["task"]),
                            index=i,
                            episodes=step.get("episodes"),
                            **base_common,
                        )
                        key = (
                            name
                            if (not loop_forever and repeat == 1 and n == 1)
                            else f"{name}#{round_i}.{i}"
                        )
                        results_all[key] = result
                        if (
                            step_interval_s > 0
                            and i + 1 < n
                            and execution != "async"
                            and not _past_end()
                        ):
                            _wait_until(
                                time.time() + step_interval_s,
                                f"步骤间隔 {step_interval_s:g}s · 下一步 {i + 2}/{n}",
                            )
                    if stopped_by_end:
                        break
                round_i += 1
                more = loop_forever or round_i < repeat
                if more and not _past_end() and interval_s > 0:
                    _wait_until(
                        time.time() + interval_s,
                        f"循环间隔 {interval_s:g}s · 下一轮 {round_i + 1}",
                    )
                elif not more or _past_end():
                    if _past_end():
                        stopped_by_end = True
                    break
            if stopped_by_end:
                self.publish(
                    JobEvent(
                        time.time(),
                        "info",
                        f"到达结束时间 · {end_at_raw}，停止调度",
                        job_id=job_id,
                    )
                )
            self._finish_success(job_id, results_all)
        except InterruptedError:
            history_snap = []
            with self._lock:
                if self._job and self._job.job_id == job_id:
                    self._job.status = "cancelled"
                    self._job.finished_at = time.time()
                    self._job.error = "cancelled"
                    for t, st in list(self._job.task_states.items()):
                        if st in ("pending", "running"):
                            self._job.task_states[t] = "cancelled"
                    self._archive_job(self._job)
                history_snap = list(self._history)
            self.publish(
                JobEvent(
                    time.time(),
                    "job_cancelled",
                    "Batch cancelled",
                    job_id=job_id,
                    payload={"history": history_snap},
                )
            )
        except Exception as exc:  # noqa: BLE001
            _LOG.exception("dashboard job failed")
            history_snap = []
            with self._lock:
                if self._job and self._job.job_id == job_id:
                    self._job.status = "failed"
                    self._job.error = str(exc)
                    self._job.finished_at = time.time()
                    cur = self._job.current_task
                    if cur:
                        self._job.task_states[cur] = "failed"
                    for t, st in list(self._job.task_states.items()):
                        if st == "pending":
                            self._job.task_states[t] = "cancelled"
                    self._archive_job(self._job)
                history_snap = list(self._history)
                self._metrics["jobs_failed"] += 1
            self.publish(
                JobEvent(
                    time.time(),
                    "job_failed",
                    f"Job failed: {exc}",
                    job_id=job_id,
                    payload={"error": str(exc), "history": history_snap},
                )
            )

    def _run_named_batch(
        self,
        *,
        job_id: str,
        tasks: list[str],
        named_calls: list[dict[str, Any]] | None = None,
        execution: str = "sync",
        **_kwargs: Any,
    ) -> None:
        from benchmark.dashboard.ota_client import OtaClient, OtaClientError

        calls = list(named_calls or [])
        self._mark_running(job_id)
        self.publish(
            JobEvent(
                time.time(),
                "info",
                f"Named gRPC job {job_id} started ({execution})",
                job_id=job_id,
                payload={"tasks": tasks, "execution": execution, "kind": "named"},
            )
        )
        results_all: dict[str, Any] = {}
        client = OtaClient()
        try:

            def _one(i: int, call: dict[str, Any]) -> tuple[str, dict[str, Any]]:
                if self._cancel.is_set():
                    raise InterruptedError("cancelled")
                name = str(call.get("id") or call.get("method") or f"rpc-{i}")
                method = str(call.get("rpc") or call.get("method") or "")
                data = call.get("data") if isinstance(call.get("data"), dict) else {}
                target = str(call.get("target") or "") or None
                with self._lock:
                    if self._job and self._job.job_id == job_id:
                        self._job.current_task = name
                        self._job.task_states[name] = "running"
                        self._job.progress = i / max(len(calls), 1)
                self.publish(
                    JobEvent(
                        time.time(),
                        "task_start",
                        f"gRPC `{method}`",
                        job_id=job_id,
                        payload={"task": name, "method": method},
                    )
                )
                t0 = time.time()
                cli = OtaClient(target=target) if target else client
                try:
                    raw = cli.call(method, data)
                    ok = True
                    err = ""
                except OtaClientError as exc:
                    raw = {}
                    ok = False
                    err = str(exc)
                elapsed = round(time.time() - t0, 3)
                result = {
                    "ok": ok,
                    "error": err,
                    "elapsed_s": elapsed,
                    "method": method,
                    "response": raw,
                    "summary": {"ok": int(ok), "fail": int(not ok)},
                }
                with self._lock:
                    if self._job and self._job.job_id == job_id:
                        self._job.task_states[name] = "done" if ok else "failed"
                        self._job.progress = (i + 1) / max(len(calls), 1)
                self.publish(
                    JobEvent(
                        time.time(),
                        "task_done",
                        f"gRPC `{method}` → {'ok' if ok else 'fail'}",
                        job_id=job_id,
                        payload={"task": name, "result": result},
                    )
                )
                if not ok:
                    raise OtaClientError(err or f"{method} failed")
                return name, result

            if execution == "async" and len(calls) > 1:
                with ThreadPoolExecutor(max_workers=min(4, len(calls))) as pool:
                    futs = [pool.submit(_one, i, c) for i, c in enumerate(calls)]
                    for fut in as_completed(futs):
                        name, result = fut.result()
                        results_all[name] = result
            else:
                for i, call in enumerate(calls):
                    name, result = _one(i, call)
                    results_all[name] = result
            self._finish_success(job_id, results_all)
        except InterruptedError:
            history_snap = []
            with self._lock:
                if self._job and self._job.job_id == job_id:
                    self._job.status = "cancelled"
                    self._job.finished_at = time.time()
                    self._job.error = "cancelled"
                    for t, st in list(self._job.task_states.items()):
                        if st in ("pending", "running"):
                            self._job.task_states[t] = "cancelled"
                    self._archive_job(self._job)
                history_snap = list(self._history)
            self.publish(
                JobEvent(
                    time.time(),
                    "job_cancelled",
                    "Named batch cancelled",
                    job_id=job_id,
                    payload={"history": history_snap},
                )
            )
        except Exception as exc:  # noqa: BLE001
            _LOG.exception("named gRPC job failed")
            history_snap = []
            with self._lock:
                if self._job and self._job.job_id == job_id:
                    self._job.status = "failed"
                    self._job.error = str(exc)
                    self._job.finished_at = time.time()
                    cur = self._job.current_task
                    if cur:
                        self._job.task_states[cur] = "failed"
                    self._archive_job(self._job)
                history_snap = list(self._history)
                self._metrics["jobs_failed"] += 1
            self.publish(
                JobEvent(
                    time.time(),
                    "job_failed",
                    f"Named job failed: {exc}",
                    job_id=job_id,
                    payload={"error": str(exc), "history": history_snap},
                )
            )


def _extract_curves(episode_results: list[Any]) -> dict[str, Any]:
    from benchmark.evaluation.control_curves import merge_episode_curves

    curves_list = []
    for ep in episode_results:
        info = getattr(ep, "info", None) or {}
        curves_list.append(info.get("control_curves") or {})
    return merge_episode_curves(curves_list)


def _extract_map_track(episode_results: list[Any], task_name: str) -> dict[str, Any]:
    """Last episode path/goal for BicMap layers."""
    if not episode_results:
        return {"task": task_name, "path": [], "goal": None, "start": None}
    ep = episode_results[-1]
    info = getattr(ep, "info", None) or {}
    path = info.get("path") or []
    pose = path[-1] if path else None
    return {
        "task": task_name,
        "mission": info.get("mission") or task_name,
        "status": getattr(getattr(ep, "status", None), "value", str(getattr(ep, "status", ""))),
        "path": path,
        "goal": info.get("goal"),
        "start": info.get("start"),
        "pose": pose,
        "waypoints": info.get("waypoints") or info.get("n_waypoints"),
    }


_HUB: MonitorHub | None = None
_HUB_LOCK = threading.Lock()


def get_hub() -> MonitorHub:
    global _HUB
    with _HUB_LOCK:
        if _HUB is None:
            _HUB = MonitorHub()
        return _HUB


def event_stream(hub: MonitorHub) -> Generator[str, None, None]:
    q = hub.subscribe()
    # Initial snapshot so clients paint immediately.
    snap = hub.snapshot()
    yield JobEvent(time.time(), "snapshot", "connected", payload=snap).to_sse()
    try:
        while True:
            try:
                event = q.get(timeout=15.0)
            except queue.Empty:
                yield JobEvent(time.time(), "heartbeat", "ok", payload=hub.snapshot()).to_sse()
                continue
            if event is None:
                break
            yield event.to_sse()
    finally:
        hub.unsubscribe(q)
