.PHONY: help install install-dev lint test run list-tasks eval train modes compare multi sim nav collect replay hil clean

PYTHON ?= python3
CONFIG ?= configs/default.yaml
TASK   ?= navigate
MODE   ?= random
EPISODES ?= 5

help:
	@echo "autonomy-benchmark (Arena-style) targets:"
	@echo "  make install|install-dev|lint|test"
	@echo "  make list-tasks|modes|compare|multi"
	@echo "  make run TASK=navigate MODE=random"
	@echo "  make eval EPISODES=10"
	@echo "  make train"
	@echo "  make nav|collect|replay|hil"

install:
	$(PYTHON) -m pip install -e .

install-dev:
	$(PYTHON) -m pip install -e ".[dev]"
	pre-commit install || true

lint:
	ruff check src tests
	ruff format --check src tests

test:
	$(PYTHON) -m pytest -q

list-tasks:
	benchmark list

modes:
	benchmark modes

run:
	benchmark run --config $(CONFIG) --task $(TASK) --mode $(MODE)

eval:
	benchmark eval --config $(CONFIG) --episodes $(EPISODES)

train:
	benchmark train --backend simple --episodes 30 --curriculum --export data/checkpoints/policy

tune:
	benchmark tune --trials 5 --episodes 3

compare:
	$(PYTHON) scripts/benchmark_planners.py

multi:
	benchmark multi-robot --robots 3 --planner hybrid --timeout 15

sim:
	benchmark run --config $(CONFIG) --task navigate --mode $(MODE)

nav:
	benchmark run --config $(CONFIG) --task navigate --mode scenario

collect:
	benchmark run --config configs/default.yaml --task collect

replay:
	benchmark run --config configs/default.yaml --task replay

hil:
	benchmark run --config configs/default.yaml --task hil

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	rm -rf .pytest_cache .ruff_cache *.egg-info
