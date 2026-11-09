FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.3 \
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

RUN set -eux; \
    for attempt in 1 2 3 4 5; do \
        rm -rf /tmp/small_gicp; \
        if git clone --depth 1 --branch v1.0.1 \
            https://github.com/koide3/small_gicp.git /tmp/small_gicp; then \
            break; \
        fi; \
        if [ "${attempt}" -eq 5 ]; then exit 1; fi; \
        sleep "$((attempt * 5))"; \
    done; \
    test "$(git -C /tmp/small_gicp rev-parse HEAD)" = \
        "57c1106daf83c2c79ee0c58a9c7ed0032298ff4e" \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/small_gicp/build --parallel "$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp

RUN set -eux; \
    for attempt in 1 2 3 4 5; do \
        rm -rf /tmp/tomlplusplus; \
        if git clone --depth 1 --branch v3.4.0 \
            https://github.com/marzer/tomlplusplus.git /tmp/tomlplusplus; then \
            break; \
        fi; \
        if [ "${attempt}" -eq 5 ]; then exit 1; fi; \
        sleep "$((attempt * 5))"; \
    done; \
    test "$(git -C /tmp/tomlplusplus rev-parse HEAD)" = \
        "30172438cee64926dc41fdd9c11fb3ba5b2ba9de" \
    && cmake -S /tmp/tomlplusplus -B /tmp/tomlplusplus/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DTOMLPLUSPLUS_BUILD_TESTS=OFF \
        -DTOMLPLUSPLUS_BUILD_EXAMPLES=OFF \
    && cmake --build /tmp/tomlplusplus/build --parallel "$(nproc)" \
    && cmake --install /tmp/tomlplusplus/build \
    && rm -rf /tmp/tomlplusplus

# GTSAM 4.3's exported target names libmetis.so, but the PPA package does not
# pull in the development symlink package transitively.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libmetis-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]
