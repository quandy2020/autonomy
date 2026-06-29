#!/bin/bash
set -e

# Download and install ccache
cd /tmp && \
    wget https://github.com/ccache/ccache/releases/download/v4.10.2/ccache-4.10.2-linux-x86_64.tar.xz && \
    tar -xvf ccache-4.10.2-linux-x86_64.tar.xz && \
    cd ccache-4.10.2-linux-x86_64 && \
    make install && \
    ln -sf /usr/local/bin/ccache /usr/local/bin/cc && \
    ln -sf /usr/local/bin/ccache /usr/local/bin/c++ && \
    if [ -x /usr/local/cuda/bin/nvcc ] && ! readlink -f /usr/local/cuda/bin/nvcc | grep -q '/ccache$'; then \
        ln -sf /usr/local/bin/ccache /usr/local/cuda/bin/nvcc; \
    fi && \
    cd / && \
    rm -rf /tmp/ccache-4.10.2-linux-x86_64* && \
    echo 'ccache --max-size 10GB' | tee --append /etc/bash.bashrc

