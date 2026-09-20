#!/usr/bin/env python3
# Copyright 2026 The Autonomy Authors
#
# @file enable_board_swap.py
# @brief Create and enable a swapfile on low-RAM boards (Firefly, etc.).
#
# Usage:
#   python3 tools/enable_board_swap.py
#   SWAP_SIZE=8G python3 tools/enable_board_swap.py
#
# Many Firefly RT kernels ship with CONFIG_SWAP=n. In that case this script
# exits non-zero — reduce build parallelism instead: cmake --build build -j1

"""@package tools.enable_board_swap
@brief Board swapfile helper for low-memory builds.
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


def run_command(cmd: list[str], *, check: bool = True) -> None:
    """@brief Run a subprocess command.

    @param cmd Argument vector passed to subprocess.run.
    @param check If True, raise CalledProcessError on non-zero exit.
    """
    subprocess.run(cmd, check=check)


def run_as_root(cmd: list[str], *, check: bool = True) -> None:
    """@brief Run @p cmd as root (via sudo when not already root).

    @param cmd Argument vector (without leading sudo).
    @param check If True, raise on failure.
    """
    if os.geteuid() == 0:
        run_command(cmd, check=check)
    else:
        run_command(["sudo", *cmd], check=check)


def kernel_release() -> str:
    """@brief Return the running kernel release string.

    @return Value of platform.release(), e.g. \"5.10.160-rt78-preempt\".
    """
    return platform.release()


def swap_config_snippet() -> str:
    """@brief Read CONFIG_SWAP-related lines from the kernel config.

    @return Multiline snippet, or empty string when unavailable.
    """
    boot_cfg = Path(f"/boot/config-{kernel_release()}")
    if boot_cfg.exists():
        try:
            text = boot_cfg.read_text(encoding="utf-8", errors="ignore")
            lines = [ln for ln in text.splitlines() if "CONFIG_SWAP" in ln]
            return "\n".join(lines)
        except OSError:
            pass
    proc_cfg = Path("/proc/config.gz")
    if proc_cfg.exists() and shutil.which("zgrep"):
        out = subprocess.run(
            ["zgrep", "CONFIG_SWAP", str(proc_cfg)],
            check=False,
            capture_output=True,
            text=True,
        )
        return out.stdout or ""
    return ""


def swap_disabled_in_kernel() -> bool:
    """@brief Detect whether the kernel was built without swap support.

    @return True when `# CONFIG_SWAP is not set` appears in kernel config.
    """
    return "# CONFIG_SWAP is not set" in swap_config_snippet()


def swap_already_active(swapfile: Path) -> bool:
    """@brief Check whether @p swapfile is already enabled.

    @param swapfile Absolute path of the swapfile.
    @return True if `swapon --show` lists the path.
    """
    out = subprocess.run(
        ["swapon", "--show"],
        check=False,
        capture_output=True,
        text=True,
    )
    return str(swapfile) in (out.stdout or "")


def parse_size_to_mib(size: str) -> int:
    """@brief Parse a human size string into mebibytes.

    @param size Size like \"4G\" or \"4096M\".
    @return Size in MiB.
    @raises ValueError If the suffix is not G/M.
    """
    raw = size.strip().upper()
    if raw.endswith("G"):
        return int(float(raw[:-1]) * 1024)
    if raw.endswith("M"):
        return int(float(raw[:-1]))
    raise ValueError(f"unsupported size={size!r} (use e.g. 4G or 4096M)")


def create_swapfile(swapfile: Path, size: str) -> None:
    """@brief Allocate a new swapfile on disk.

    Prefers `fallocate`; falls back to `dd` when fallocate fails.

    @param swapfile Destination path.
    @param size Human-readable size (e.g. \"4G\").
    """
    print(f"[run] creating {swapfile} ({size})...")
    try:
        run_as_root(["fallocate", "-l", size, str(swapfile)])
    except subprocess.CalledProcessError:
        print("[warn] fallocate failed, falling back to dd...")
        mib = parse_size_to_mib(size)
        run_as_root(
            [
                "dd",
                "if=/dev/zero",
                f"of={swapfile}",
                "bs=1M",
                f"count={mib}",
                "status=progress",
            ]
        )


def ensure_fstab(swapfile: Path) -> None:
    """@brief Persist @p swapfile in /etc/fstab if missing.

    @param swapfile Absolute swapfile path.
    """
    fstab = Path("/etc/fstab")
    line = f"{swapfile} none swap sw 0 0\n"
    existing = ""
    if fstab.exists():
        existing = fstab.read_text(encoding="utf-8", errors="ignore")
    for row in existing.splitlines():
        if row.strip().startswith("#") or not row.strip():
            continue
        if row.split()[:1] == [str(swapfile)]:
            print("[ok] already in /etc/fstab")
            return
    subprocess.run(
        ["sudo", "tee", "-a", str(fstab)],
        input=line,
        text=True,
        check=True,
        capture_output=True,
    )
    print("[ok] appended to /etc/fstab")


def show_memory_status() -> None:
    """@brief Print `free -h` and `swapon --show` when available."""
    if shutil.which("free"):
        run_command(["free", "-h"], check=False)
    if shutil.which("swapon"):
        run_command(["swapon", "--show"], check=False)


def main(argv: list[str] | None = None) -> int:
    """@brief CLI entry: create and enable a swapfile.

    @param argv Optional argument vector (defaults to sys.argv[1:]).
    @return Process exit code (0 on success).
    """
    parser = argparse.ArgumentParser(
        description="Create and enable a swapfile on low-RAM boards."
    )
    parser.add_argument(
        "--swapfile",
        default=os.environ.get("SWAPFILE", "/swapfile"),
        help="swapfile path (default: /swapfile or $SWAPFILE)",
    )
    parser.add_argument(
        "--size",
        default=os.environ.get("SWAP_SIZE", "4G"),
        help="size for fallocate (default: 4G or $SWAP_SIZE)",
    )
    args = parser.parse_args(argv)
    swapfile = Path(args.swapfile)

    if swap_disabled_in_kernel():
        print(
            "[error] kernel has CONFIG_SWAP disabled — cannot enable swapfile/zram."
        )
        print(f"        uname={kernel_release()}")
        print("        Mitigate OOM with: cmake --build build -j1   (or -j2)")
        print("        and stop unused services (ros2 demos, ffmpeg, etc.).")
        show_memory_status()
        return 1

    if swap_already_active(swapfile):
        print(f"[ok] {swapfile} already active")
        show_memory_status()
        return 0

    if not swapfile.exists():
        create_swapfile(swapfile, args.size)
    else:
        print(f"[ok] {swapfile} exists, will enable")

    run_as_root(["chmod", "600", str(swapfile)])
    run_as_root(["mkswap", str(swapfile)])
    try:
        run_as_root(["swapon", str(swapfile)])
    except subprocess.CalledProcessError:
        print("[error] swapon failed (kernel may lack CONFIG_SWAP).")
        run_as_root(["rm", "-f", str(swapfile)], check=False)
        return 1

    ensure_fstab(swapfile)
    print("=== done ===")
    show_memory_status()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
