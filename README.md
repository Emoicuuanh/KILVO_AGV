# KILVO trên AGV mô phỏng — ROS 1 Noetic + Gazebo 11 (Docker)

Máy host chạy Ubuntu 24.04 nên ROS 1 chạy trong image `kilvo-sim:noetic`. Workspace này được gắn vào container tại `/ws`.

```
src/kilvo             KILVO gốc (github.com/JixinGao/KILVO), KHÔNG sửa
src/rpg_vikit         vikit gốc (phụ thuộc của KILVO)
src/livox_ros_driver  chỉ có message CustomMsg/CustomPoint (cùng MD5 với driver thật)
src/kilvo_agv_gazebo  nhà xưởng + AGV + các node mô phỏng
docker/               Dockerfile, run.sh
results/              kết quả từng lần chạy
```

## Lấy mã nguồn
```bash
git clone --recursive git@github.com:Emoicuuanh/KILVO_AGV.git ~/kilvo_ros1_ws
# nếu đã clone mà thiếu --recursive:
git submodule update --init --recursive
```
`src/kilvo` và `src/rpg_vikit` là submodule, trỏ về repo gốc của tác giả ở đúng commit đang dùng.

## Một lần duy nhất
```bash
cd ~/kilvo_ros1_ws
docker build -t kilvo-sim:noetic docker/
docker/run.sh catkin_make -DCMAKE_BUILD_TYPE=Release
xhost +SI:localuser:$USER                    # cho container vẽ lên màn hình (cần cả khi chạy headless, vì camera phải render)
```
Lệnh `xhost` mất tác dụng sau mỗi lần đăng nhập lại, nên phải chạy lại.

## Chạy
```bash
# Gazebo có giao diện, lái tay
docker/run.sh roslaunch kilvo_agv_gazebo sim.launch rviz:=true
docker exec -it kilvo_sim bash -ic 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py'

# Thực nghiệm đầy đủ: KILVO + kịch bản tự động + ghi quỹ đạo (headless)
docker/run.sh roslaunch kilvo_agv_gazebo sim.launch gui:=false kilvo:=true scenario:=true record:=true output_dir:=/ws/results/run1

# Đánh giá: ATE, sai số tại mã QR, sai số theo vùng, hình quỹ đạo
docker/run.sh rosrun kilvo_agv_gazebo eval_ate.py /ws/results/run1
```
Có thể thêm `rviz:=true` vào `kilvo_agv.launch` để xem bằng RViz của tác giả.

## Topic
| Topic | Nội dung |
|---|---|
| `/livox/lidar` | `livox_ros_driver/CustomMsg`, 10 Hz, vùng nhìn kiểu Mid360 (360° × −7°…52°) |
| `/lidar/points` | PointCloud2 gốc từ Gazebo |
| `/imu/data` | 200 Hz, có nhiễu và bias |
| `/camera/image_raw`, `/camera/camera_info` | 640×480 |
| `/joint_states` | góc bánh |
| `/wheel/encoder`, `/wheel/twist`, `/wheel/odom` | encoder 4096 xung/vòng, odometry bánh |
| `/sim/slip` | `[v_enc, w_enc, v_gt, w_gt, slip_L, slip_R, flag]`: nhãn trượt thật |
| `/ground_truth/odom` | tư thế thật, 100 Hz |
| `/sim/zone`, `/sim/step` | vùng hiện tại, bước kịch bản |
| `/aft_mapped_to_init` | odometry của KILVO |

## Ghi chú kỹ thuật
- **offset_time = 0:** Gazebo chụp cả vòng quét tức thời, nên `lidar_to_livox` đặt `offset_time = 0`. Nhờ vậy KILVO không khử méo nhầm.
  - Không dùng `lidar_type` Velodyne, vì bộ xử lý Velodyne của KILVO luôn tự suy thời gian từ góc quét.
- **Chế độ LIVO:** config `config/kilvo_agv_sim.yaml` để `leg_en: 0`, vì AGV không có chân. Thay động học chân bằng động học vi sai là phần việc của bài báo.
- **Ngoại tham số** khớp với `urdf/agv.urdf.xacro`. Nếu dời cảm biến thì phải sửa cả hai nơi.
- **Tốc độ mô phỏng** khoảng 0.4× thời gian thực. Muốn nhanh hơn có hai cách:
  - Giảm số tia LiDAR: `lidar_h_samples:=480 lidar_v_samples:=16`.
  - Thử `gpu_lidar:=true`.
- **Đổi bố trí xưởng:** sửa `config/layout.yaml`, rồi chạy `python3 tools/generate_world.py --classic`.
