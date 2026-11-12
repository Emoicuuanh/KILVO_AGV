#!/usr/bin/env bash
# Mở container ROS 1 Noetic + Gazebo 11 với workspace ~/kilvo_ros1_ws gắn vào /ws.
#   docker/run.sh                    -> shell tương tác
#   docker/run.sh <lệnh...>          -> chạy lệnh rồi thoát, vd: docker/run.sh catkin_make
# GUI (Gazebo/RViz) qua X11. Nếu không hiện cửa sổ: chạy `xhost +local:docker` trên máy host.
# Mặc định dùng GPU tích hợp qua /dev/dri; nếu lỗi đồ họa đặt SOFTWARE_GL=1.
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$WS/.home" "$WS/results"
GL_ENV=()
[ "${SOFTWARE_GL:-0}" = "1" ] && GL_ENV=(-e LIBGL_ALWAYS_SOFTWARE=1)
DRI=()
[ -e /dev/dri ] && DRI=(--device /dev/dri)
TTY=()
[ -t 0 ] && TTY=(-it)
NAME=${CONTAINER_NAME:-kilvo_sim}
exec docker run --rm "${TTY[@]}" --name "$NAME" --net host --ipc host \
  --user "$(id -u):$(id -g)" $(for g in $(id -G); do echo --group-add "$g"; done) \
  -e HOME=/ws/.home -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 "${GL_ENV[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw "${DRI[@]}" \
  -v "$WS":/ws -w /ws \
  kilvo-sim:noetic bash -c 'source /opt/ros/noetic/setup.bash; [ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash; if [ $# -gt 0 ]; then exec "$@"; else exec bash; fi' _ "$@"
