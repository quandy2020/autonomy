# BicMap library source (slim)

Core source extracted from [Open-X-Humanoid/BICMap](https://github.com/Open-X-Humanoid/BICMap) (`src/bicMap` + build entry).  
Examples / public assets are **not** included.

## Build into dashboard (offline vendor)

```bash
# from benchmark/
bash scripts/build_bicmap.sh
```

Output: `src/benchmark/dashboard/static/vendor/bic-map/bicMap.ext.min.js`

Runtime loads that file via Flask static — **no website / CDN**.
