# Copyright 2025 The Autonomy Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM nvcr.io/nvidia/isaac-lab:2.2.0

# TZData goes first.
RUN apt-get update
ENV TZ=Europe/Berlin
ENV DEBIAN_FRONTEND=noninteractive
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update && apt-get install -y tzdata

# install packages
RUN apt clean

# Fix libbrotli version conflict: base image has 1.1.0, but libbrotli-dev needs 1.0.9
# We need to downgrade libbrotli1 before installing packages that depend on libbrotli-dev
RUN apt-get update && \
    apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get --no-install-recommends install -y \
    # Source development \
    git jq build-essential \
    # System packages \
    sudo ssh gnupg wget curl unzip \
    # Package management tools \
    apt-utils software-properties-common \
    # Python bootstrap packages \
    python3-pip python3-venv python3-dev \
    libegl1 libgl1 libgomp1 \
    # Tools used in tests \
    parallel \
    pkg-config \
    autoconf \
    automake \
    cmake \
    vim \
    bc \
    gdb \
    libsdl2-dev \
    libblas-dev \
    liblapack-dev \
    libtinyxml2-dev \
    liblua5.3-dev \
    ninja-build \
    sphinx \
    python3-sphinx \
    uuid-dev \
    libcivetweb-dev \
    libsuitesparse-dev \
    lsb-release \
    libompl-dev \
    libfontconfig1-dev \
    libfreetype6-dev \
    libcairo2-dev \
    libboost-all-dev \
    libasio-dev \
    libncurses5-dev \
    libavcodec-dev \
    libswscale-dev \
    libpoco-dev \
    libflann-dev \
    libqhull-dev \
    libpcap0.8 \
    libpcap0.8-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libmetis-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    liboctomap-dev \
    libgraphicsmagick++-dev \
    libfltk1.3-dev \
    libtool \
    libtiff-dev \
    libcurl4-openssl-dev \
    libwebsocketpp-dev \
    libeigen3-dev \
    libsqlite3-dev \
    libzmq3-dev \
    liburdfdom-dev \
    libgtk2.0-dev \
    clang-format \
    uuid-dev \
    sqlite3 \
    stow && \
    rm -rf /var/lib/apt/lists/*


# # Setup entrypoint
# COPY scripts/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
# RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
# ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]

# Install pytorch.
# Note that all python deps are installed inside a venv. This is required by ubuntu >= 24.
RUN python3 -m venv /opt/venv
RUN . /opt/venv/bin/activate && \
    python3 -m pip install --ignore-installed --upgrade pip && \
    python3 -m pip install \
    # Pytorch. Note that this version is only compatible with pre-cxx11 ABI binaries. \
    torch==2.7.0

# Install python deps.
RUN . /opt/venv/bin/activate && \
    python3 -m pip install --upgrade \
    # Deployment tools \
    wheel requests setuptools \
    # Testing \
    pytest pytest-rerunfailures \
    # Profiling \
    nvtx

# Copy all install scripts
COPY install /tmp/install

# Install cmake and ccache.
RUN chmod +x /tmp/install/install_cmake.sh && bash /tmp/install/install_cmake.sh
RUN chmod +x /tmp/install/install_ccache.sh && bash /tmp/install/install_ccache.sh

# Make sure we're using text-based interactive terminal.
ENV DEBIAN_FRONTEND=teletype

# Setup env variables by adding them to the global bashrc.
RUN . /opt/venv/bin/activate && python3 -c "import site; print(site.getsitepackages()[0])" > /site-packages-dir
RUN echo '# CUDA and PyTorch environment variables' | tee --append /etc/bash.bashrc && \
    echo 'PATH=$PATH:/usr/local/cuda/bin' | tee --append /etc/bash.bashrc && \
    # Make sure we have the torch cmake path
    echo "export CMAKE_PREFIX_PATH=$(cat /site-packages-dir)/torch" | tee --append /etc/bash.bashrc && \
    # Explicit CUDA_PATH is needed here due to the non-standard location of the nvcc compiler (see section on ccache)
    echo 'export CUDA_PATH=/usr/local/cuda' | tee --append /etc/bash.bashrc && \
    # Always activate the venv upon login
    echo 'source /opt/venv/bin/activate' | tee --append /etc/bash.bashrc

# Setup GLOG environment variables in user home directory .bashrc
RUN touch /root/.bashrc && \
    echo '# GLOG environment variables' >> /root/.bashrc && \
    echo 'export GLOG_logtostderr=1' >> /root/.bashrc && \
    echo 'export GLOG_alsologtostderr=0' >> /root/.bashrc && \
    echo 'export GLOG_colorlogtostderr=1' >> /root/.bashrc && \
    echo 'export GLOG_minloglevel=0' >> /root/.bashrc

# Ensure that a local user can access the python venv
RUN chmod -R a+rw /opt/venv

# Install autonomy dependencies

# thirdparty
RUN mkdir /thirdparty
RUN bash /tmp/install/install_gtest.sh
RUN bash /tmp/install/install_glog.sh
RUN bash /tmp/install/install_gflags.sh
RUN bash /tmp/install/install_grpc.sh
RUN bash /tmp/install/install_gperftools.sh
RUN bash /tmp/install/install_opencv.sh
RUN bash /tmp/install/install_ceres_solver.sh
RUN bash /tmp/install/install_nlohmann.sh
RUN bash /tmp/install/install_osqp.sh
RUN bash /tmp/install/install_behaviortree_cpp.sh
RUN bash /tmp/install/install_ccache.sh
RUN bash /tmp/install/install_ros2.sh
RUN bash /tmp/install/install_assimp.sh
RUN bash /tmp/install/install_ogre.sh
RUN bash /tmp/install/install_mcap.sh
RUN bash /tmp/install/install_adolc.sh
RUN bash /tmp/install/install_ipopt.sh
RUN bash /tmp/install/install_gperftools.sh
RUN bash /tmp/install/install_python_modules.sh

# autonomy workspace
ENV AUTONOMY_WS=/workspace/autonomy

RUN mkdir -p $AUTONOMY_WS
WORKDIR $AUTONOMY_WS

