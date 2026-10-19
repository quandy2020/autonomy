# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Native Python gRPC client for ``automsgs/proto/rpcs``.

Loads RPC descriptors via ``protoc --descriptor_set_out`` (no grpcurl),
then calls methods with JSON dict request/response (protobuf JSON mapping).
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Any, Iterable, Iterator

from benchmark.common.config import package_root

SYSTEM_SERVICE = "automsgs.rpcs.system.SystemService"

_pool_lock = threading.Lock()
_pool_cache: dict[str, Any] = {}  # key → DescriptorPool
_request_desc_cache: dict[str, dict[str, Any]] = {}
_request_desc_lock = threading.Lock()


class RpcClientError(RuntimeError):
    """Raised when proto layout / protoc / gRPC call fails."""


def default_target() -> str:
    return os.environ.get("AUTOMSGS_RPC_TARGET", "localhost:50051")


def find_automsgs_root() -> Path | None:
    env = os.environ.get("AUTOMSGS_ROOT") or os.environ.get("AUTOMSGS_PROTO_ROOT")
    if env:
        root = Path(env).resolve()
        if (root / "proto" / "rpcs").is_dir():
            return root
        if (root / "rpcs").is_dir() and (root / "msgs").is_dir():
            return root.parent if root.name == "proto" else root
    sibling = package_root().parent / "automsgs"
    if (sibling / "proto" / "rpcs").is_dir():
        return sibling.resolve()
    return None


def find_protoc() -> str | None:
    return shutil.which("protoc")


def grpc_available() -> bool:
    try:
        import grpc  # noqa: F401
        from google.protobuf import json_format  # noqa: F401

        return True
    except ImportError:
        return False


def ensure_proto_include(automsgs_root: Path, include_dir: Path) -> Path:
    """Mirror CMake layout: ``include/automsgs/{msgs,rpcs,…}`` → proto sources."""
    env = os.environ.get("AUTOMSGS_PROTO_INCLUDE")
    if env:
        root = Path(env).resolve()
        if not (root / "automsgs" / "rpcs").is_dir():
            raise RpcClientError(f"AUTOMSGS_PROTO_INCLUDE missing automsgs/rpcs: {root}")
        return root

    proto_src = automsgs_root / "proto"
    if not proto_src.is_dir():
        raise RpcClientError(f"automsgs proto not found: {proto_src}")

    root = include_dir.resolve()
    am = root / "automsgs"
    am.mkdir(parents=True, exist_ok=True)
    for name in ("msgs", "srvs", "rpcs", "actions"):
        src = proto_src / name
        if not src.is_dir():
            continue
        dst = am / name
        if dst.is_symlink() and dst.resolve() == src.resolve():
            continue
        if dst.exists() or dst.is_symlink():
            if dst.is_dir() and not dst.is_symlink():
                shutil.rmtree(dst)
            else:
                dst.unlink()
        dst.symlink_to(src, target_is_directory=True)
    return root


def _descriptor_set_path(include: Path) -> Path:
    rpcs = include / "automsgs" / "rpcs"
    stamp = ""
    if rpcs.is_dir():
        mtimes = [p.stat().st_mtime_ns for p in rpcs.glob("*.proto")]
        stamp = hashlib.sha1(str(max(mtimes) if mtimes else 0).encode()).hexdigest()[:12]
    cache = Path(tempfile.gettempdir()) / "benchmark-automsgs-rpc"
    cache.mkdir(parents=True, exist_ok=True)
    return cache / f"rpcs_{stamp}.pb"


def build_descriptor_set(include: Path, out: Path | None = None) -> Path:
    protoc = find_protoc()
    if not protoc:
        raise RpcClientError(
            "protoc not found in PATH. Install protobuf compiler to call automsgs RPCs."
        )
    rpcs = sorted((include / "automsgs" / "rpcs").glob("*.proto"))
    if not rpcs:
        raise RpcClientError(f"no rpcs protos under {include / 'automsgs' / 'rpcs'}")
    dest = out or _descriptor_set_path(include)
    if dest.is_file() and dest.stat().st_size > 0:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        protoc,
        f"-I{include}",
        "--include_imports",
        f"--descriptor_set_out={dest}",
        *[p.relative_to(include).as_posix() for p in rpcs],
    ]
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise RpcClientError("failed to execute protoc") from exc
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout or "").strip() or f"exit {proc.returncode}"
        raise RpcClientError(f"protoc descriptor set failed: {err}")
    return dest


def load_descriptor_pool(include: Path) -> Any:
    key = str(include.resolve())
    with _pool_lock:
        cached = _pool_cache.get(key)
        if cached is not None:
            return cached

    from google.protobuf import descriptor_pb2, descriptor_pool

    desc_path = build_descriptor_set(include)
    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(desc_path.read_bytes())
    pool = descriptor_pool.DescriptorPool()
    pending = list(fds.file)
    # Multi-pass add for dependency order.
    for _ in range(len(pending) + 2):
        if not pending:
            break
        next_pending: list[Any] = []
        for fd in pending:
            try:
                pool.Add(fd)
            except Exception:  # noqa: BLE001 — retry later if deps missing
                next_pending.append(fd)
        if len(next_pending) == len(pending):
            # Force remaining via serialized add (last resort).
            for fd in next_pending:
                pool.AddSerializedFile(fd.SerializeToString())
            pending = []
            break
        pending = next_pending
    if pending:
        raise RpcClientError(f"failed to load {len(pending)} proto descriptors")

    with _pool_lock:
        _pool_cache[key] = pool
    return pool


def _message_class(pool: Any, descriptor: Any) -> Any:
    from google.protobuf import message_factory

    if hasattr(message_factory, "GetMessageClass"):
        return message_factory.GetMessageClass(descriptor)
    return message_factory.MessageFactory(pool).GetPrototype(descriptor)


def _split_method(method: str, default_service: str = SYSTEM_SERVICE) -> tuple[str, str]:
    full = method if "/" in method else f"{default_service}/{method}"
    service, name = full.rsplit("/", 1)
    return service, name


def _to_dict(msg: Any) -> dict[str, Any]:
    from google.protobuf import json_format

    return json_format.MessageToDict(
        msg,
        preserving_proto_field_name=True,
    )


def _from_dict(cls: Any, data: dict[str, Any] | None) -> Any:
    from google.protobuf import json_format

    msg = cls()
    if data:
        json_format.ParseDict(data, msg, ignore_unknown_fields=True)
    return msg


class RpcClient:
    """JSON-friendly gRPC client for all ``automsgs.rpcs.*`` services."""

    def __init__(
        self,
        target: str | None = None,
        *,
        plaintext: bool = True,
        timeout_s: float = 30.0,
        include_dir: Path | None = None,
    ) -> None:
        self.target = target or default_target()
        self.plaintext = plaintext
        self.timeout_s = timeout_s
        self._include_dir = include_dir
        self._automsgs = find_automsgs_root()
        self._channel = None
        self._pool = None

    def probe(self) -> dict[str, Any]:
        has_grpc = grpc_available()
        protoc = find_protoc()
        return {
            "target": self.target,
            "transport": "python-grpc",
            "grpcio": has_grpc,
            "protoc": bool(protoc),
            "protoc_path": protoc or "",
            "automsgs_root": str(self._automsgs) if self._automsgs else "",
            "ready": bool(has_grpc and protoc and self._automsgs),
            "service": SYSTEM_SERVICE,
        }

    def check_link(self, *, timeout_s: float = 2.5) -> dict[str, Any]:
        """Probe local deps + live gRPC connectivity (Heartbeat)."""
        import time

        probe = self.probe()
        out: dict[str, Any] = {
            "ok": False,
            "linked": False,
            "state": "offline",
            "label": "未连接",
            "target": self.target,
            "latency_ms": None,
            "error": "",
            "probe": probe,
            "heartbeat": None,
        }
        if not probe["ready"]:
            missing = []
            if not probe.get("grpcio"):
                missing.append("grpcio")
            if not probe.get("protoc"):
                missing.append("protoc")
            if not probe.get("automsgs_root"):
                missing.append("automsgs/proto")
            out["state"] = "unready"
            out["label"] = "依赖缺失"
            out["error"] = "缺少: " + ", ".join(missing)
            return out

        import grpc

        # Fresh channel so readiness reflects current target.
        self.close()
        try:
            channel = self._get_channel()
            t0 = time.perf_counter()
            grpc.channel_ready_future(channel).result(timeout=timeout_s)
            ready_ms = round((time.perf_counter() - t0) * 1000.0, 1)
        except Exception as exc:  # noqa: BLE001
            self.close()
            out["state"] = "unreachable"
            out["label"] = "不可达"
            out["error"] = str(exc) or "channel not ready"
            return out

        try:
            t1 = time.perf_counter()
            hb = self.call(
                f"{SYSTEM_SERVICE}/Heartbeat",
                {"sequence": 1},
                timeout_s=timeout_s,
            )
            rtt = round((time.perf_counter() - t1) * 1000.0, 1)
            out["ok"] = True
            out["linked"] = True
            out["state"] = "online"
            out["label"] = "已连接"
            out["latency_ms"] = rtt
            out["channel_ready_ms"] = ready_ms
            out["heartbeat"] = hb
            out["error"] = ""
        except RpcClientError as exc:
            out["state"] = "error"
            out["label"] = "RPC 失败"
            out["error"] = str(exc)
            out["channel_ready_ms"] = ready_ms
        finally:
            self.close()
        return out

    def _include(self) -> Path:
        if not self._automsgs:
            raise RpcClientError(
                "automsgs proto root not found. Set AUTOMSGS_ROOT to the automsgs package."
            )
        if self._include_dir is None:
            cache = Path(tempfile.gettempdir()) / "benchmark-ota-proto-include"
            return ensure_proto_include(self._automsgs, cache)
        return ensure_proto_include(self._automsgs, self._include_dir)

    def _get_pool(self) -> Any:
        if self._pool is None:
            self._pool = load_descriptor_pool(self._include())
        return self._pool

    def _get_channel(self) -> Any:
        if self._channel is not None:
            return self._channel
        if not grpc_available():
            raise RpcClientError(
                "grpcio not installed. pip install 'autonomy-benchmark[dashboard]'"
            )
        import grpc

        if self.plaintext:
            self._channel = grpc.insecure_channel(self.target)
        else:
            creds = grpc.ssl_channel_credentials()
            self._channel = grpc.secure_channel(self.target, creds)
        return self._channel

    def close(self) -> None:
        if self._channel is not None:
            self._channel.close()
            self._channel = None

    def _resolve_method(self, method: str) -> tuple[Any, Any, Any, str]:
        service_name, method_name = _split_method(method)
        pool = self._get_pool()
        try:
            service = pool.FindServiceByName(service_name)
        except KeyError as exc:
            raise RpcClientError(f"unknown service: {service_name}") from exc
        md = service.FindMethodByName(method_name)
        if md is None:
            raise RpcClientError(f"unknown method: {service_name}/{method_name}")
        req_cls = _message_class(pool, md.input_type)
        resp_cls = _message_class(pool, md.output_type)
        path = f"/{service_name}/{method_name}"
        return md, req_cls, resp_cls, path

    def call(
        self,
        method: str,
        data: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Unary or server-streaming call; streaming returns the last message."""
        messages = self.call_stream(method, data, timeout_s=timeout_s)
        if not messages:
            return {}
        return messages[-1]

    def call_stream(
        self,
        method: str,
        data: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> list[dict[str, Any]]:
        """Call RPC; unary → ``[resp]``, server-stream → all responses."""
        import grpc

        md, req_cls, resp_cls, path = self._resolve_method(method)
        req = _from_dict(req_cls, data)
        channel = self._get_channel()
        max_time = timeout_s if timeout_s is not None else self.timeout_s

        if md.client_streaming:
            raise RpcClientError(
                f"{path} is client-streaming; use call_client_stream([...])"
            )

        try:
            if md.server_streaming:
                stub = channel.unary_stream(
                    path,
                    request_serializer=req_cls.SerializeToString,
                    response_deserializer=resp_cls.FromString,
                )
                out: list[dict[str, Any]] = []
                for msg in stub(req, timeout=max_time or None):
                    out.append(_to_dict(msg))
                return out

            stub = channel.unary_unary(
                path,
                request_serializer=req_cls.SerializeToString,
                response_deserializer=resp_cls.FromString,
            )
            resp = stub(req, timeout=max_time or None)
            return [_to_dict(resp)]
        except grpc.RpcError as exc:
            details = exc.details() if hasattr(exc, "details") else str(exc)
            code = exc.code().name if hasattr(exc, "code") else "ERROR"
            raise RpcClientError(f"{path} failed: {code}: {details}") from exc

    def call_client_stream(
        self,
        method: str,
        messages: Iterable[dict[str, Any]],
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Client-stream or bidi: send JSON messages, return last response."""
        import grpc

        md, req_cls, resp_cls, path = self._resolve_method(method)
        channel = self._get_channel()
        max_time = timeout_s if timeout_s is not None else self.timeout_s

        def _gen() -> Iterator[Any]:
            for item in messages:
                yield _from_dict(req_cls, item if isinstance(item, dict) else {})

        try:
            if md.client_streaming and md.server_streaming:
                stub = channel.stream_stream(
                    path,
                    request_serializer=req_cls.SerializeToString,
                    response_deserializer=resp_cls.FromString,
                )
            elif md.client_streaming:
                stub = channel.stream_unary(
                    path,
                    request_serializer=req_cls.SerializeToString,
                    response_deserializer=resp_cls.FromString,
                )
            else:
                raise RpcClientError(f"{path} is not client-streaming")

            call = stub(_gen(), timeout=max_time or None)
            if md.server_streaming:
                last: dict[str, Any] = {}
                for msg in call:
                    last = _to_dict(msg)
                return last
            return _to_dict(call)
        except grpc.RpcError as exc:
            details = exc.details() if hasattr(exc, "details") else str(exc)
            code = exc.code().name if hasattr(exc, "code") else "ERROR"
            raise RpcClientError(f"{path} failed: {code}: {details}") from exc


def get_rpc_client(target: str | None = None, **kwargs: Any) -> RpcClient:
    return RpcClient(target=target, **kwargs)


def _enum_default(field: Any) -> str | int:
    enum = field.enum_type
    if enum is None or not enum.values:
        return 0
    # Prefer a non-UNSPECIFIED / unknown-looking name when available.
    for v in enum.values:
        name = v.name or ""
        upper = name.upper()
        if "UNSPECIFIED" in upper or upper.endswith("_UNKNOWN") or name.endswith("0"):
            continue
        return name
    return enum.values[0].name


def _sample_field_value(field: Any, *, depth: int, max_depth: int, seen: set[str]) -> Any:
    from google.protobuf.descriptor import FieldDescriptor as FD

    if field.label == FD.LABEL_REPEATED:
        # map<K,V> is a repeated message with map_entry option.
        msg = field.message_type
        if msg is not None and msg.GetOptions().map_entry:
            return {}
        if depth >= max_depth:
            return []
        if field.type == FD.TYPE_MESSAGE:
            return [_sample_message(field.message_type, depth=depth + 1, max_depth=max_depth, seen=seen)]
        if field.type == FD.TYPE_ENUM:
            return [_enum_default(field)]
        if field.type in (FD.TYPE_STRING, FD.TYPE_BYTES):
            return [""]
        if field.type == FD.TYPE_BOOL:
            return [False]
        return [0]

    if field.type == FD.TYPE_MESSAGE:
        if depth >= max_depth:
            return {}
        return _sample_message(field.message_type, depth=depth + 1, max_depth=max_depth, seen=seen)
    if field.type == FD.TYPE_ENUM:
        return _enum_default(field)
    if field.type in (FD.TYPE_STRING, FD.TYPE_BYTES):
        return ""
    if field.type == FD.TYPE_BOOL:
        return False
    if field.type in (FD.TYPE_DOUBLE, FD.TYPE_FLOAT):
        return 0.0
    return 0


def _sample_message(
    descriptor: Any,
    *,
    depth: int = 0,
    max_depth: int = 4,
    seen: set[str] | None = None,
) -> dict[str, Any]:
    seen = seen if seen is not None else set()
    name = getattr(descriptor, "full_name", "") or ""
    if name in ("google.protobuf.Empty", "google.protobuf.Any"):
        return {}
    if name in seen:
        return {}
    seen = set(seen)
    seen.add(name)
    out: dict[str, Any] = {}
    for field in descriptor.fields:
        # Skip oneof siblings that aren't the first — still fill all for a usable template.
        out[field.name] = _sample_field_value(
            field, depth=depth, max_depth=max_depth, seen=seen
        )
    return out


def _fields_from_sample(
    sample: Any,
    *,
    prefix: str = "",
    limit: int = 16,
) -> list[dict[str, Any]]:
    """Flatten sample JSON into editable form fields (scalars + enum strings)."""
    fields: list[dict[str, Any]] = []

    def walk(node: Any, path: str) -> None:
        if len(fields) >= limit:
            return
        if isinstance(node, dict):
            for key, val in node.items():
                walk(val, f"{path}.{key}" if path else key)
            return
        if isinstance(node, list):
            if node:
                walk(node[0], f"{path}.0" if path else "0")
            return
        if not path:
            return
        if isinstance(node, bool):
            fields.append(
                {
                    "path": path,
                    "label": path.split(".")[-1],
                    "type": "select",
                    "options": ["false", "true"],
                }
            )
            return
        if isinstance(node, (int, float)) and not isinstance(node, bool):
            fields.append(
                {
                    "path": path,
                    "label": path.split(".")[-1],
                    "type": "number",
                    "step": 0.1 if isinstance(node, float) else 1,
                }
            )
            return
        fields.append({"path": path, "label": path.split(".")[-1], "type": "text"})

    walk(sample, prefix)
    return fields


def describe_rpc_request(method: str) -> dict[str, Any]:
    """Build autofill JSON + form fields from the request message descriptor.

    Safe to call without a live robot; only needs protoc + automsgs protos.
    """
    key = method if "/" in method else f"{SYSTEM_SERVICE}/{method}"
    with _request_desc_lock:
        cached = _request_desc_cache.get(key)
        if cached is not None:
            return cached

    empty = {"default_body": {}, "fields": [], "request_type": "", "ok": False, "error": ""}
    try:
        client = RpcClient()
        if not client.probe().get("ready"):
            return {**empty, "error": "rpc deps not ready"}
        md, _req_cls, _resp_cls, _path = client._resolve_method(key)
        sample = _sample_message(md.input_type)
        out = {
            "ok": True,
            "request_type": md.input_type.full_name,
            "default_body": sample,
            "fields": _fields_from_sample(sample),
            "error": "",
        }
    except Exception as exc:  # noqa: BLE001 — catalog must stay resilient
        return {**empty, "error": str(exc)}

    with _request_desc_lock:
        _request_desc_cache[key] = out
    return out
