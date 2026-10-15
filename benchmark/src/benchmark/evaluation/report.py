# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Write evaluation summaries (JSON / Markdown / HTML)."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any, Dict, Union


def write_report(summary: Dict[str, Any], output_dir: Union[str, Path], name: str = "summary") -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{name}.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    md = out / f"{name}.md"
    lines = ["# Benchmark report", ""]
    for key, value in summary.items():
        lines.append(f"- **{key}**: `{value}`")
    md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    write_html_report(summary, out / f"{name}.html", title=name)
    return path


def write_html_report(summary: Dict[str, Any], path: Union[str, Path], title: str = "report") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    def rows(obj: Any, prefix: str = "") -> str:
        if isinstance(obj, dict):
            parts = []
            for k, v in obj.items():
                key = f"{prefix}.{k}" if prefix else str(k)
                if isinstance(v, dict):
                    parts.append(rows(v, key))
                else:
                    parts.append(
                        f"<tr><td>{html.escape(key)}</td>"
                        f"<td><code>{html.escape(str(v))}</code></td></tr>"
                    )
            return "".join(parts)
        return f"<tr><td>{html.escape(prefix or 'value')}</td><td><code>{html.escape(str(obj))}</code></td></tr>"

    # Highlight common Arena Evaluation blocks
    blocks = ""
    for section in ("efficiency", "safety", "smoothness", "status"):
        if section in summary and isinstance(summary[section], dict):
            blocks += f"<h2>{html.escape(section)}</h2><table>{rows(summary[section])}</table>"

    body = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>{html.escape(title)}</title>
<style>
body{{font-family:ui-sans-serif,system-ui,sans-serif;margin:2rem;background:#0b1220;color:#e8eef7}}
table{{border-collapse:collapse;width:100%;margin:1rem 0}}
td,th{{border:1px solid #243044;padding:.4rem .6rem;text-align:left}}
h1,h2{{color:#4cc9f0}} code{{color:#b5e48c}}
</style></head><body>
<h1>Arena Evaluation — {html.escape(title)}</h1>
{blocks}
<h2>raw</h2>
<table>{rows({k: v for k, v in summary.items() if k not in ("efficiency", "safety", "smoothness", "status", "episodes")})}</table>
</body></html>
"""
    path.write_text(body, encoding="utf-8")
    return path
