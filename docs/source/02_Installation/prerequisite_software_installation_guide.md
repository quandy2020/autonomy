# 2 程序运行

程序运行的方式有2种：

* 方式1：使用源码方式进行安装运行（不推荐，效率较低，安装依赖繁琐）
* :shamrock: **<font color='green'> 方式2：使用docker方式进行安装运行 </font>** **<font color='red'>(推荐指数:star2::star2::star2::star2::star2:)</font>** 

## 1 源码方式

* apt安装

```bash
sudo apt update && \
    apt install -y sudo \
    build-essential \
    cmake \
    curl \
    git \
    unzip \
    vim \
    wget \
    bc \
    gdb \
    libcivetweb-dev \
    libceres-dev \
    libblas-dev \
    liblapack-dev \
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
	  libyaml-cpp-dev \
	  libwebsocketpp-dev \
	  libpcl-dev \
	  liboctomap-dev \
	  libopencv-dev \
	  libgraphicsmagick++-dev \
    software-properties-common \
    stow
```

*  behaviortree_cpp

```bash
git clone -b 4.7.2 https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP 
mkdir build && cmake ..
make -j8
sudo make install
```

* cyberRT

```bash
git clone --single-branch --branch v10.0.0 https://github.com/minhanghuang/CyberRT.git

# 安装third_party
cd CyberRT 
sudo python3 install.py --install_prefix /usr/local
source /usr/local/setup.zsh or source /usr/local/setup.bash

# 安装cyber
cd CyberRT && cmake -B build
cd build && cmake ..
make -j8
sudo make install
```

*  benchmark

```bash
git clone -b v1.9.0 https://github.com/google/benchmark.git
cd benchmark
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_ENABLE_TESTING=OFF
cmake --install build
```

* gRPC

```bash
git clone https://github.com/grpc/grpc.git
cd grpc 
git checkout v1.48.0
mkdir build && cd build
cmake  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DgRPC_INSTALL=ON \
  -DgRPC_BUILD_TESTS=OFF \
  -DgRPC_SSL_PROVIDER=package \
  -DgRPC_ZLIB_PROVIDER=package \
  -DgRPC_PROTOBUF_PROVIDER=package \
  -DABSL_PROPAGATE_CXX_STD=ON \
  -DBUILD_SHARED_LIBS=ON   \
  ..  
make -j8
sudo make install
```

## <font color='green'> 2 Docker（ 推荐指数:star2::star2::star2::star2:) </font>

* :rocket: docker安装

```bash
cd autonomy/docker/scripts
sudo ./install_docker.sh
```

* :fire: 构造镜像(x86-64)平台

```bash
cd autonomy/docker
./build_docker.sh -f dockerfile/autonomy.x86_64.dockerfile 
```

* :triangular_flag_on_post: 构造镜像(aarch64)平台

```bash
cd autonomy/docker
./build_docker.sh -f dockerfile/autonomy.aarch64.dockerfile 
```

* :leaves: 设置环境变量

在autonomy的docker容器中配置.bashrc环境变量如下

```bash
### Autonomy ###
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
export GLOG_log_dir=${HOME}/.autonomy/log
source /usr/local/setup.bash

# /home/quandy/workspace/github/autonomy: autonomy源码目录
# AUTONOMY_ENV环境变量在./run_autonomy.sh 脚本中使用
### autonomy ###
export AUTONOMY_ENV=/home/quandy/workspace/github/autonomy 
```

* :house: 运行docker

```bash
./run_autonomy.sh 
```

* :heart: 运行docker(容器SpaceHero)

```bash
docker exec -it SpaceHero  /bin/bash
```

## 3 代码编译&安装

```bash
cd autonomy
mkdir build
cd build
cmake .. -G Ninja
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
```

