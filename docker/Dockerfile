# Sapphire — LiDAR-Inertial SLAM
# Dockerfile for development and deployment
#
# Build from the sapphire_ros2 repository root:
#   docker build -f docker/Dockerfile \
#     --build-arg ROS_DISTRO=humble \
#     -t sapphire:humble .
#
# Run:
#   docker run -it --rm --network host \
#     -v /path/to/sapphire_ws:/workspace sapphire:humble
#
# ROS_DISTRO may be changed to any distribution for which the selected
# osrf/ros image and all dependencies below are available (for example,
# humble or jazzy).

ARG ROS_DISTRO=humble
ARG ROS_IMAGE_VARIANT=desktop-full
FROM osrf/ros:${ROS_DISTRO}-${ROS_IMAGE_VARIANT}

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Sapphire core
    libeigen3-dev \
    libtbb-dev \
    # Build tools
    cmake \
    build-essential \
    # small_gicp deps
    libnanoflann-dev \
    libomp-dev \
    # Logging
    libspdlog-dev \
    # misc
    git \
    && rm -rf /var/lib/apt/lists/*

# ── GTSAM via BorgLab PPA ─────────────────────────────────────────
# The PPA must support the Ubuntu release used by the selected ROS image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-develop \
    && apt-get update && apt-get install -y --no-install-recommends \
    libgtsam-dev \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 extra deps ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-pcl-ros" \
    "ros-${ROS_DISTRO}-tf2-eigen" \
    && rm -rf /var/lib/apt/lists/*

# ── Vendor: small_gicp ────────────────────────────────────────────
RUN cd /tmp && \
    git clone --depth 1 https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) install && \
    rm -rf /tmp/small_gicp

# ── Vendor: tomlplusplus (header-only) ────────────────────────────
# Build from source so this works on ROS distributions whose Ubuntu base
# does not package a sufficiently recent tomlplusplus.
RUN cd /tmp && \
    git clone --depth 1 https://github.com/marzer/tomlplusplus.git && \
    cd tomlplusplus && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make install && \
    rm -rf /tmp/tomlplusplus

# ── Workspace skeleton ────────────────────────────────────────────
# Source is expected to be mounted at runtime.
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
