# KILVO + Gazebo 11 trên ROS 1 Noetic
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-velodyne-gazebo-plugins \
      ros-noetic-teleop-twist-keyboard \
      ros-noetic-rqt-image-view \
      libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev \
      python3-catkin-tools python3-pip python3-yaml python3-numpy python3-matplotlib \
      mesa-utils git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Sophus không-template (commit a621ff) — bản FAST-LIVO2/KILVO yêu cầu.
# so2.cpp đã vá cho Eigen mới (gán unit_complex_ thay vì .real()=).
COPY sophus /tmp/Sophus
RUN cd /tmp/Sophus && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
    && make -j"$(nproc)" install && ldconfig && rm -rf /tmp/Sophus \
    && mkdir -p /usr/local/lib/cmake/Sophus \
    && printf 'set(Sophus_INCLUDE_DIR /usr/local/include)\nset(Sophus_INCLUDE_DIRS /usr/local/include)\nset(Sophus_LIBRARY /usr/local/lib/libSophus.so)\nset(Sophus_LIBRARIES /usr/local/lib/libSophus.so)\nset(Sophus_FOUND TRUE)\n' > /usr/local/lib/cmake/Sophus/SophusConfig.cmake

ENV QT_X11_NO_MITSHM=1
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc \
 && echo "[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash" >> /etc/bash.bashrc
WORKDIR /ws
