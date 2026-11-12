# ============================================================================
# Image kilvo-sim:noetic — ROS 1 Noetic + Gazebo 11 + thư viện để build KILVO.
# Build:  docker build -t kilvo-sim:noetic docker/
# Image chỉ chứa thư viện; mã nguồn và kết quả build nằm ở ~/kilvo_ros1_ws (gắn vào /ws khi chạy).
# ============================================================================

# ===== 1. NỀN: ROS Noetic bản đầy đủ (có sẵn Gazebo 11, RViz, PCL, OpenCV, cv_bridge) =====
FROM osrf/ros:noetic-desktop-full

# ===== 2. GÓI APT BỔ SUNG =====
#   velodyne-gazebo-plugins : plugin LiDAR 3D cho Gazebo (xuất PointCloud2 có trường ring)
#   teleop-twist-keyboard   : lái robot bằng bàn phím
#   rqt-image-view          : xem ảnh camera
#   lib*-dev, catkin-tools  : thư viện + công cụ build KILVO
#   python3-*               : cho các node Python và script đánh giá
#   mesa-utils              : glxinfo, kiểm tra đồ họa
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-velodyne-gazebo-plugins \
      ros-noetic-teleop-twist-keyboard \
      ros-noetic-rqt-image-view \
      libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev \
      python3-catkin-tools python3-pip python3-yaml python3-numpy python3-matplotlib \
      mesa-utils git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# ===== 3. SOPHUS (bản không-template, commit a621ff) — KILVO/vikit yêu cầu đúng bản này =====
# Mã nguồn chép sẵn trong docker/sophus (xem docker/sophus/VENDORED.md).
# so2.cpp đã vá cho Eigen mới (gán unit_complex_ thay vì .real()=).
# Bản Sophus cũ không cài SophusConfig.cmake -> tự tạo để find_package(Sophus) tìm thấy.
COPY sophus /tmp/Sophus
RUN cd /tmp/Sophus && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
    && make -j"$(nproc)" install && ldconfig && rm -rf /tmp/Sophus \
    && mkdir -p /usr/local/lib/cmake/Sophus \
    && printf 'set(Sophus_INCLUDE_DIR /usr/local/include)\nset(Sophus_INCLUDE_DIRS /usr/local/include)\nset(Sophus_LIBRARY /usr/local/lib/libSophus.so)\nset(Sophus_LIBRARIES /usr/local/lib/libSophus.so)\nset(Sophus_FOUND TRUE)\n' > /usr/local/lib/cmake/Sophus/SophusConfig.cmake

# ===== 4. MÔI TRƯỜNG =====
# QT_X11_NO_MITSHM: tránh lỗi hiển thị Qt (RViz) qua X11 từ container
ENV QT_X11_NO_MITSHM=1
# Mở shell bash trong container là có sẵn môi trường ROS + workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc \
 && echo "[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash" >> /etc/bash.bashrc
WORKDIR /ws
