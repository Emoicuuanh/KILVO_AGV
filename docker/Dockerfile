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

# ===== 5. CÔNG CỤ XEM / XỬ LÝ BẢN ĐỒ .pcd =====
#   pcl_viewer     : xem bản đồ (kros pcl_viewer file.pcd)
#   pcl_voxel_grid : lọc thưa bản đồ
RUN apt-get update && apt-get install -y --no-install-recommends pcl-tools \
    && rm -rf /var/lib/apt/lists/*

# ===== 6. DẪN ĐƯỜNG (move_base) =====
#   map_server: nạp bản đồ .pgm/.yaml; move_base + costmap + navfn + dwa: lập đường và bám đường
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-move-base ros-noetic-map-server ros-noetic-costmap-2d \
      ros-noetic-navfn ros-noetic-global-planner ros-noetic-dwa-local-planner \
      ros-noetic-base-local-planner ros-noetic-clear-costmap-recovery ros-noetic-rotate-recovery \
    && rm -rf /var/lib/apt/lists/*

# ===== 7. MESA MỚI (build từ mã nguồn) để dùng GPU Intel đời mới =====
# Mesa 21.x của Ubuntu 20.04 không có driver cho GPU Intel Raptor Lake (8086:a7a8) -> Gazebo/RViz
# phải vẽ bằng CPU. Build libdrm 2.4.120 + Mesa 23.3.6, chỉ driver iris (Intel Gen12+) và swrast (dự phòng).
# Cài vào /usr/local (ưu tiên hơn /usr nhờ ld.so.conf), dùng chung libglvnd của hệ thống.
ARG LIBDRM_VER=2.4.120
ARG MESA_VER=23.3.6
RUN apt-get update && apt-get install -y --no-install-recommends \
      wget xz-utils ninja-build pkg-config bison flex python3-pip python3-setuptools \
      libglvnd-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxrandr-dev libxxf86vm-dev \
      libxcb1-dev libxcb-dri2-0-dev libxcb-dri3-dev libxcb-present-dev libxcb-sync-dev libxcb-glx0-dev \
      libxcb-shm0-dev libxcb-xfixes0-dev libxcb-randr0-dev libxshmfence-dev x11proto-dev \
      libexpat1-dev zlib1g-dev libzstd-dev libpciaccess-dev \
    && pip3 install --no-cache-dir "meson==1.3.2" "mako==1.3.2" \
    && cd /tmp && wget -q https://dri.freedesktop.org/libdrm/libdrm-${LIBDRM_VER}.tar.xz \
    && tar xf libdrm-${LIBDRM_VER}.tar.xz && cd libdrm-${LIBDRM_VER} \
    && meson setup build --prefix=/usr/local --buildtype=release \
         -Damdgpu=disabled -Dradeon=disabled -Dnouveau=disabled -Dvmwgfx=disabled \
         -Dintel=enabled -Dvalgrind=disabled -Dman-pages=disabled -Dtests=false -Dcairo-tests=disabled \
    && ninja -C build install \
    && cd /tmp && wget -q https://archive.mesa3d.org/mesa-${MESA_VER}.tar.xz \
    && tar xf mesa-${MESA_VER}.tar.xz && cd mesa-${MESA_VER} \
    && meson setup build --prefix=/usr/local --buildtype=release \
         -Dplatforms=x11 -Dgallium-drivers=iris,swrast -Dvulkan-drivers= -Dllvm=disabled \
         -Dglx=dri -Degl=enabled -Dgbm=enabled -Dglvnd=true -Dshared-glapi=enabled \
         -Dgles1=disabled -Dgles2=enabled -Dvalgrind=disabled -Dlibunwind=disabled \
         -Dgallium-va=disabled -Dgallium-vdpau=disabled -Dgallium-xa=disabled -Dgallium-nine=false \
         -Dgallium-opencl=disabled -Dosmesa=false -Dbuild-tests=false -Dvideo-codecs= \
    && ninja -C build install && ldconfig \
    && rm -rf /tmp/libdrm-* /tmp/mesa-* /var/lib/apt/lists/*
