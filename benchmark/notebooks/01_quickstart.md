# Benchmark quickstart

```bash
cd src/autonomy/benchmark
pip install -e ".[dev]"
benchmark list
benchmark run --mode random --set task.episodes=3
benchmark eval --episodes 5 --set planner.name=dwa
```

In Python:

```python
from benchmark.common.config import load_config
from benchmark.evaluation.arena_evaluation import ArenaEvaluation

cfg = load_config("configs/default.yaml", overrides=["task.episodes=3", "planner.name=hybrid"])
summary = ArenaEvaluation(cfg).run()
print(summary["efficiency"])
```
