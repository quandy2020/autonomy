# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""gRPC client for automsgs ``SystemService`` OTA RPCs (Python grpcio).

Calls map to ``automsgs/proto/rpcs/system.proto``:

* ``GetCapabilities`` / ``GetInfo`` / ``GetOtaStatus``
* ``StartOta`` / ``AbortOta``

Transport uses native Python gRPC (see ``rpc_client``).
Override target with ``AUTOMSGS_RPC_TARGET`` (default ``localhost:50051``).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from benchmark.dashboard.rpc_client import (
    SYSTEM_SERVICE,
    RpcClient,
    RpcClientError,
    default_target,
    ensure_proto_include,
    find_automsgs_root,
    find_protoc,
    get_rpc_client,
    grpc_available,
)

# Backward-compatible aliases for map_control / tests.
OtaClientError = RpcClientError
_ensure_proto_include = ensure_proto_include


def find_grpcurl() -> str | None:
    """Deprecated: grpcurl is no longer required (kept for probe compat)."""
    return None


def _rpc_proto_flags(_include_dir: Path) -> list[str]:
    """Deprecated stub for old grpcurl callers."""
    return []


OTA_STATE_LABELS: dict[str, str] = {
    "OTA_STATE_UNKNOWN": "未知",
    "OTA_STATE_IDLE": "空闲",
    "OTA_STATE_DOWNLOADING": "下载中",
    "OTA_STATE_VERIFYING": "校验中",
    "OTA_STATE_APPLYING": "应用中",
    "OTA_STATE_SWITCHING": "切换中",
    "OTA_STATE_RESTARTING": "重启中",
    "OTA_STATE_HEALTH_GATING": "健康门控",
    "OTA_STATE_COMMITTED": "已提交",
    "OTA_STATE_ROLLING_BACK": "回滚中",
    "OTA_STATE_FAILED": "失败",
    "0": "未知",
    "1": "空闲",
    "2": "下载中",
    "3": "校验中",
    "4": "应用中",
    "5": "切换中",
    "6": "重启中",
    "7": "健康门控",
    "8": "已提交",
    "9": "回滚中",
    "10": "失败",
}


class OtaClient:
    """Thin wrapper around :class:`RpcClient` for SystemService OTA methods."""

    def __init__(
        self,
        target: str | None = None,
        *,
        plaintext: bool = True,
        timeout_s: float = 30.0,
        start_timeout_s: float = 600.0,
        include_dir: Path | None = None,
    ) -> None:
        self.target = target or default_target()
        self.plaintext = plaintext
        self.timeout_s = timeout_s
        self.start_timeout_s = start_timeout_s
        self._include_dir = include_dir
        self._automsgs = find_automsgs_root()
        self._rpc = RpcClient(
            target=self.target,
            plaintext=plaintext,
            timeout_s=timeout_s,
            include_dir=include_dir,
        )

    def probe(self) -> dict[str, Any]:
        probe = self._rpc.probe()
        # Keep legacy key so existing UI/tests can adapt gradually.
        probe["grpcurl"] = False
        probe["grpcurl_path"] = ""
        probe["python_grpc"] = probe.get("grpcio", False)
        return probe

    def get_capabilities(self) -> dict[str, Any]:
        return self.call("GetCapabilities", {})

    def get_info(self) -> dict[str, Any]:
        return self.call("GetInfo", {})

    def get_ota_status(self) -> dict[str, Any]:
        return self.call("GetOtaStatus", {})

    def start_ota(
        self,
        package_uri: str,
        *,
        package_type: str = "",
        prefer_delta: bool = False,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {
            "package_uri": package_uri,
            "prefer_delta": prefer_delta,
        }
        if package_type:
            body["package_type"] = package_type
        return self.call("StartOta", body, timeout_s=self.start_timeout_s)

    def abort_ota(self, reason: str = "operator") -> dict[str, Any]:
        return self.call("AbortOta", {"reason": reason or "operator"})

    def snapshot(self) -> dict[str, Any]:
        """Capabilities + info + OTA status for the panel."""
        probe = self.probe()
        out: dict[str, Any] = {"ok": True, "probe": probe, "error": ""}
        if not probe["ready"]:
            out["ok"] = False
            missing = []
            if not probe.get("grpcio"):
                missing.append("grpcio")
            if not probe.get("protoc"):
                missing.append("protoc")
            if not probe.get("automsgs_root"):
                missing.append("automsgs/proto")
            out["error"] = "缺少依赖: " + ", ".join(missing)
            return out
        try:
            caps = self.get_capabilities()
            info = self.get_info()
            status = self.get_ota_status()
            out["capabilities"] = caps
            out["info"] = info
            out["ota"] = enrich_ota_status(status)
            out["supports_ota"] = bool(
                (caps or {}).get("supportsOta") or (caps or {}).get("supports_ota")
            )
        except RpcClientError as exc:
            out["ok"] = False
            out["error"] = str(exc)
        return out

    def call(
        self,
        method: str,
        data: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        full = method if "/" in method else f"{SYSTEM_SERVICE}/{method}"
        return self._rpc.call(full, data, timeout_s=timeout_s)

    def call_client_stream(
        self,
        method: str,
        messages: list[dict[str, Any]],
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        return self._rpc.call_client_stream(method, messages, timeout_s=timeout_s)


def enrich_ota_status(status: dict[str, Any]) -> dict[str, Any]:
    """Add human-readable state label + clamped progress."""
    raw = status.get("state")
    key = str(raw if raw is not None else "OTA_STATE_UNKNOWN")
    progress = status.get("progress")
    try:
        prog = float(progress) if progress is not None else 0.0
    except (TypeError, ValueError):
        prog = 0.0
    prog = max(0.0, min(1.0, prog))
    return {
        **status,
        "state_key": key,
        "state_label": OTA_STATE_LABELS.get(key, key),
        "progress_pct": round(prog * 100.0, 1),
    }


def get_client(target: str | None = None) -> OtaClient:
    return OtaClient(target=target)


__all__ = [
    "OTA_STATE_LABELS",
    "OtaClient",
    "OtaClientError",
    "SYSTEM_SERVICE",
    "RpcClient",
    "RpcClientError",
    "_ensure_proto_include",
    "_rpc_proto_flags",
    "default_target",
    "enrich_ota_status",
    "find_automsgs_root",
    "find_grpcurl",
    "find_protoc",
    "get_client",
    "get_rpc_client",
    "grpc_available",
]
