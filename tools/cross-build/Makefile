.DEFAULT_GOAL := help
PYTHON ?= python3
SOURCE ?= $(abspath ../..)
DOCKER_PLATFORM ?= linux/amd64
export PROFILE HOST USER SOURCE IMAGE DOCKER_PLATFORM
.PHONY: help list check sysroot verify build install test
help:
	@echo 'make list | check | test'
	@echo 'make sysroot PROFILE=<id> HOST=<board> USER=<ssh-user>'
	@echo 'make verify | build | install PROFILE=<id> [SOURCE=<cmake-project>]'
	@echo 'For extra SDK paths/CMake arguments use scripts/cross_build.py (see README).'
list check sysroot verify build install:
	$(PYTHON) scripts/cross_build.py $@
test:
	$(PYTHON) -m unittest discover -s tests -v
