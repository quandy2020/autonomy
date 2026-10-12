#!/usr/bin/env bash
# Render TikZ schematics with tectonic → PDF → PNG/SVG (pymupdf).
set -euo pipefail
cd "$(dirname "$0")"
FIGS=(double_integrator cartpole ballbot quadrotor)
# legged_robot.png: external figure (see ATTRIBUTION.md), not TikZ
ZOOM="${ZOOM:-5}"  # 5 ≈ 360 dpi relative to 72

for name in "${FIGS[@]}"; do
  echo "==> tectonic ${name}.tex"
  tectonic "${name}.tex"
  python3 - <<PY
import fitz
from pathlib import Path
name = "${name}"
zoom = float("${ZOOM}")
doc = fitz.open(f"{name}.pdf")
page = doc[0]
pix = page.get_pixmap(matrix=fitz.Matrix(zoom, zoom), alpha=False)
pix.save(f"{name}.png")
Path(f"{name}.svg").write_text(page.get_svg_image())
print(f"    png {pix.width}x{pix.height}, svg ok")
PY
done
echo "done."
