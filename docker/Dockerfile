FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.2 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libboost-all-dev \
        libeigen3-dev \
        libflann-dev \
        libgtsam-dev \
        libomp-dev \
        libpcl-dev \
        libspdlog-dev \
        ninja-build \
        pkg-config \
        python3-colcon-common-extensions \
        ros-humble-pcl-conversions \
        ros-humble-rosbag2 \
        ros-humble-rviz2 \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v1.0.1 \
        https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/small_gicp/build --parallel "$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp

WORKDIR /workspace
CMD ["bash"]
