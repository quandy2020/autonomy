# syntax=docker/dockerfile:1
# Autoviz developer/build image. Context: Autonomy CMake root (src/autonomy/).
#
#   docker build -f autoviz/deploy/docker/Dockerfile --target dev -t autoviz-dev .
#   ./autoviz/deploy/docker/run-docker.sh ubuntu Release
#
# For a full Autonomy thirdparty stack, pass a pre-built dev image:
#   docker build --build-arg AUTONOMY_BASE_IMAGE=spacehero -f autoviz/deploy/docker/Dockerfile .

ARG AUTONOMY_BASE_IMAGE=""
ARG BASE_REF=ubuntu:22.04

FROM ${BASE_REF} AS bootstrap

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

COPY autoviz/deploy/docker/lib/ /usr/local/lib/autoviz/

ARG APT_EXTRA=""
RUN APT_EXTRA="${APT_EXTRA}" sh /usr/local/lib/autoviz/setup-base.sh

RUN git config --system --add safe.directory /project/source && \
    mkdir -p /project/source /project/install

WORKDIR /project/source

# ---------- dev: Autoviz build entrypoint ----------
FROM bootstrap AS dev

COPY autoviz/deploy/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["Release"]

# Optional layer when AUTONOMY_BASE_IMAGE is provided at build time (multi-stage skip).
# Consumers typically run with AUTONOMY_BASE_IMAGE set to SpaceHero instead of bootstrap.
