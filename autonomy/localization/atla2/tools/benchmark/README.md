# benchmark

Atla2 数据集 / 平台评测套件。

```text
benchmark/
├── configs/          # euroc / uma_vi / mun_frl / kitti
├── metrics/          # accuracy / efficiency / robustness / thermal
├── runners/          # run_offline.sh / run_embedded.sh
└── reports/          # 输出报告
```

## Quick start

```bash
# 离线（默认 euroc 配置 → drone_vision_only）
./runners/run_offline.sh euroc

# 机载短跑 + 温升采样
STEPS=50 ./runners/run_embedded.sh mun_frl
```

单独跑指标：

```bash
python3 metrics/accuracy.py --est est.txt --gt gt.txt --out reports/acc.json
python3 metrics/efficiency.py --log reports/.../run.log
python3 metrics/robustness.py --log reports/.../run.log
python3 metrics/thermal.py --duration 10
```
