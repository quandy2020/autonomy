# BicMap vendor (built from local source)

Built by `scripts/build_bicmap.sh` from `third_party/bic-map-src`.

| File | Role |
|------|------|
| `bicMap.ext.min.js` | Self-contained IIFE used by the Flask page |
| `LICENSE` / `THIRD-PARTY-LICENSES.md` | Upstream licenses |
| `package.meta.json` | `{ version, source: third_party/bic-map-src, built: local }` |

Do **not** replace this with a CDN URL. Rebuild from source when updating BicMap.
