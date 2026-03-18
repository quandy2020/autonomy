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

FROM --platform=linux/arm64 osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

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
    libcivetweb-dev \
    libceres-dev \
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
    uuid-dev \
    liburdfdom-dev \
    libgtk2.0-dev \
    clang-format \
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

# Setup GLOG environment variables in user home directory .bashrc
RUN touch /root/.bashrc && \
    echo '# GLOG environment variables' >> /root/.bashrc && \
    echo 'export GLOG_logtostderr=1' >> /root/.bashrc && \
    echo 'export GLOG_alsologtostderr=0' >> /root/.bashrc && \
    echo 'export GLOG_colorlogtostderr=1' >> /root/.bashrc && \
    echo 'export GLOG_minloglevel=0' >> /root/.bashrc

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
RUN bash /tmp/install/install_python_modules.sh
RUN bash /tmp/install/install_mcap.sh
RUN bash /tmp/install/install_adolc.sh
RUN bash /tmp/install/install_ipopt.sh
RUN bash /tmp/install/install_gperftools.sh

# autonomy workspace
ENV AUTONOMY_WS /workspace/autonomy

RUN mkdir -p $AUTONOMY_WS
WORKDIR $AUTONOMY_WS
