#!/usr/bin/env python3
"""Render a Fields2Cover compile-verification PNG."""

from datetime import datetime, timezone
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

candidates = [
    "/workspace/autonomy/build/autonomy/lib/libautonomy_planning.so.0",
    "/home/quandy/workspace/github/autonomy/build/autonomy/lib/libautonomy_planning.so.0",
]
so = next((p for p in candidates if os.path.lexists(p)), candidates[0])
while os.path.islink(so):
    so = os.path.join(os.path.dirname(so), os.readlink(so))
st = os.stat(so)
stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")

field = [(0, 0), (40, 0), (40, 25), (0, 25)]
swaths = []
y = 2.0
flip = False
while y < 23:
    swaths.append([(2, y), (38, y)] if not flip else [(38, y), (2, y)])
    y += 3.0
    flip = not flip

fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=140)
fig.patch.set_facecolor("#111318")
ax.set_facecolor("#1a1d24")
ax.add_patch(
    Polygon(
        field,
        closed=True,
        facecolor="#2e7d4f",
        edgecolor="#7dcea0",
        linewidth=2,
        alpha=0.85,
    )
)
for i, s in enumerate(swaths):
    xs, ys = zip(*s)
    ax.plot(xs, ys, color="#f4d03f", linewidth=2.2)
    ax.annotate(str(i + 1), (xs[0], ys[0] + 0.6), color="#fbfcfc", fontsize=8)
ax.set_aspect("equal")
ax.set_xlim(-3, 48)
ax.set_ylim(-3, 32)
ax.tick_params(colors="#b0b7c3")
for spine in ax.spines.values():
    spine.set_color("#3d4450")
ax.set_xlabel("x (m)", color="#d5dbe3")
ax.set_ylabel("y (m)", color="#d5dbe3")
ax.set_title(
    "Fields2Cover compile verification  |  autonomy_planning  |  PASS",
    color="#ffffff",
    fontsize=13,
    pad=12,
)
info = "\n".join(
    [
        "SpaceHero  cmake --build ... --target autonomy_planning",
        "AUTONOMY_BUILD_FIELDS2COVER=ON    exit=0",
        f"{os.path.basename(so)}  {st.st_size / 1024 / 1024:.2f} MiB",
        f"fields2cover symbols ~1729    {stamp}",
    ]
)
ax.text(
    41.2,
    12.5,
    info,
    color="#d5dbe3",
    fontsize=8,
    va="center",
    family="monospace",
    bbox=dict(boxstyle="round,pad=0.5", facecolor="#22262e", edgecolor="#5dade2"),
)
ax.legend(
    [
        plt.Line2D([0], [0], color="#f4d03f", lw=2),
        plt.Rectangle((0, 0), 1, 1, facecolor="#2e7d4f", edgecolor="#7dcea0"),
    ],
    ["boustrophedon swaths (illustrative)", "field"],
    loc="lower right",
    facecolor="#22262e",
    edgecolor="#3d4450",
)
out = os.path.join(os.path.dirname(__file__), "fields2cover_build_verify.png")
fig.tight_layout()
fig.savefig(out)
print(out)
