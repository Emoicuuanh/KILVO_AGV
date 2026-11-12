#!/usr/bin/env bash
# ============================================================================
# run.sh — mở container ROS 1 Noetic + Gazebo 11, gắn workspace ~/kilvo_ros1_ws vào /ws.
#   docker/run.sh                    -> shell tương tác
#   docker/run.sh <lệnh...>          -> chạy lệnh rồi thoát, vd: docker/run.sh catkin_make
# Thường không gọi trực tiếp mà dùng lệnh tắt `kros` (docker/kros).
# GUI (Gazebo/RViz) qua X11: cần `xhost +SI:localuser:$USER` (kros tự làm).
# Nếu lỗi đồ họa: SOFTWARE_GL=1 docker/run.sh ...
# ============================================================================
set -e

# ===== 1. ĐƯỜNG DẪN: workspace = thư mục cha của docker/ =====
WS="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$WS/.home" "$WS/results"   # .home: HOME trong container (log ROS, cache Gazebo)

# ===== 2. TÙY CHỌN THEO MÁY =====
GL_ENV=()                              # ép vẽ bằng CPU nếu SOFTWARE_GL=1
[ "${SOFTWARE_GL:-0}" = "1" ] && GL_ENV=(-e LIBGL_ALWAYS_SOFTWARE=1)
DRI=()                                 # cho container thấy GPU (nếu có)
[ -e /dev/dri ] && DRI=(--device /dev/dri)
TTY=()                                 # chế độ tương tác chỉ khi chạy từ terminal
[ -t 0 ] && TTY=(-it)
NAME=${CONTAINER_NAME:-kilvo_sim}      # tên container (kros dùng tên này để tìm lại)

# ===== 3. CHẠY CONTAINER =====
#   --rm               : tự xóa container khi thoát (dữ liệu nằm ở /ws trên máy host nên không mất)
#   --net/--ipc host   : dùng chung mạng với máy -> ROS master/topic thấy được từ ngoài
#   --user + group-add : chạy bằng user của bạn -> file tạo ra không thuộc root
#   DISPLAY + X11      : hiện cửa sổ Gazebo/RViz lên màn hình
#   -v "$WS":/ws       : gắn workspace
#   bash -c ...        : nạp môi trường ROS + workspace rồi chạy lệnh được truyền vào
exec docker run --rm "${TTY[@]}" --name "$NAME" --net host --ipc host \
  --user "$(id -u):$(id -g)" $(for g in $(id -G); do echo --group-add "$g"; done) \
  -e HOME=/ws/.home -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 "${GL_ENV[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw "${DRI[@]}" \
  -v "$WS":/ws -w /ws \
  kilvo-sim:noetic bash -c 'source /opt/ros/noetic/setup.bash; [ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash; if [ $# -gt 0 ]; then exec "$@"; else exec bash; fi' _ "$@"
