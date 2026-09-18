# atlas/optimize

Stock g2o BA/pose optimizers stay here for now.

New residuals live in `estimate/`.

`graph_optimizer` canonical path is `backend/`; prefer
`backend/graph_optimizer.hpp` for new code. A thin shim remains at
`optimize/graph_optimizer.hpp` for compatibility.

Do not grow this tree with new multimodal logic.
