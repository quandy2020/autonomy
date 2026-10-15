#!/usr/bin/env bash
# End-to-end smoke: list → navigate → eval → collect → slam → perception → multi-robot
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

echo "== list =="
benchmark list

echo "== navigate scenario =="
benchmark run --mode scenario --set task.episodes=1 --set task.timeout_s=8 \
  --set task.obstacles=4 --set task.pedestrians=2 --set task.moving_obstacles=0 \
  --set task.goal_x=3.0 --set humans.name=social_force

echo "== eval dwa =="
benchmark eval --episodes 2 --set planner.name=dwa --set task.timeout_s=8 \
  --set task.obstacles=4 --set task.pedestrians=1 --set task.moving_obstacles=0

echo "== collect =="
benchmark run --task collect --set task.episodes=1 --set task.episode_frames=20 \
  --set task.output=data/datasets/smoke_collect

echo "== slam =="
benchmark run --task slam --set task.episodes=1 --set task.timeout_steps=60 \
  --set task.obstacles=3 --set task.goal_x=2.5

echo "== perception =="
benchmark run --task perception --set task.episodes=2 --set task.obstacles=6

echo "== multi-robot =="
benchmark multi-robot --robots 2 --planner pure_pursuit --timeout 8

echo "== arena modes / humans =="
benchmark run --mode benchmark --set task.episodes=3 --set task.timeout_s=6 \
  --set task.obstacles=4 --set task.pedestrians=2 --set task.moving_obstacles=0 \
  --set humans.name=orca --set planner.name=cadrl
benchmark run --mode dynamic --set task.map=canteen --set task.episodes=1 \
  --set task.timeout_s=6 --set task.pedestrians=2 --set humans.name=spinny

echo "smoke_e2e OK"
