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

FROM ubuntu:24.04 

ENV DEBIAN_FRONTEND=noninteractive

# install packages
RUN apt clean
RUN apt-get update && apt-get install -y sudo \
    software-properties-common \
    pkg-config \
    autoconf \
    automake \
    cmake \
    curl \
    git \
    unzip \
    vim \
    wget \
    bc \
    gdb \
    libsdl2-dev \
    libblas-dev \
    liblapack-dev \
    libtinyxml2-dev \
    liblua5.3-dev \
    ninja-build \
    sphinx \
    python3-pip \
    python3-dev \
    python3-sphinx \
    uuid-dev \
    libcivetweb-dev \
    libsuitesparse-dev \
    lsb-release \
    libompl-dev \
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
    sqlite3 \
    stow && \
    rm -rf /var/lib/apt/lists/*


# # Setup entrypoint
# COPY scripts/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
# RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
# ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]

RUN current_dir="dirname $(dirname "$(realpath "$0")")"

# Install autonomy dependencies
COPY $current_dir/install /tmp/install

# thirdparty
RUN mkdir /thirdparty
RUN bash /tmp/install/install_gflags_glog.sh
RUN bash /tmp/install/install_gtest.sh
RUN bash /tmp/install/install_grpc.sh
RUN bash /tmp/install/install_gperftools.sh
RUN bash /tmp/install/install_opencv.sh
RUN bash /tmp/install/install_foonathan_memory.sh
RUN bash /tmp/install/install_fastcdr.sh
RUN bash /tmp/install/install_fastdds.sh
RUN bash /tmp/install/install_behaviortree_cpp.sh
RUN bash /tmp/install/install_ceres_solver.sh
RUN bash /tmp/install/install_assimp.sh
RUN bash /tmp/install/install_ogre.sh
RUN bash /tmp/install/install_nlohmann.sh


# autonomy workspace
ENV AUTONOMY_WS /workspace/autonomy

RUN mkdir -p $AUTONOMY_WS
WORKDIR $AUTONOMY_WS
