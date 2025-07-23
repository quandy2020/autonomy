FROM osrf/ros:humble-desktop

# install packages
RUN apt clean
RUN apt update && \
    apt install -y \
    build-essential \
    cmake \
    curl \
    git \
    unzip \
    vim \
    wget \
    bc \
    gdb \
    libceres-dev \
    libblas-dev liblapack-dev \
    libtinyxml2-dev \
    liblua5.3-dev \
    ninja-build \
    sphinx \
    python3-pip \
    python3-sphinx \
    uuid-dev \
    libsuitesparse-dev \
    lsb-release \
    libompl-dev \
    libcairo2-dev \
    libboost-all-dev \
    libasio-dev \
    libtinyxml2-dev \
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
    libzmq3-dev \
    software-properties-common \
    stow


# # Setup entrypoint
# COPY scripts/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
# RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
# ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]

RUN current_dir="dirname $(dirname "$(realpath "$0")")"

# Install openbot dependencies
COPY $current_dir/install /tmp/install

# thirdparty
RUN mkdir /thirdparty
# RUN bash /tmp/install/install_bazel.sh
# RUN bash /tmp/install/install_bazel_packages.sh
RUN bash /tmp/install/install_python_modules.sh
RUN bash /tmp/install/install_cyberrt.sh
RUN bash /tmp/install/install_osqp.sh
RUN bash /tmp/install/install_abseil-cpp.sh
RUN bash /tmp/install/install_g2o.sh
RUN bash /tmp/install/install_ceres_solver.sh
RUN bash /tmp/install/install_behaviortree_cpp.sh

# openbot workspace
ENV OPENBOT_WS /workspace

RUN mkdir -p $OPENBOT_WS
WORKDIR $OPENBOT_WS
