cmake_minimum_required(VERSION 3.0.2)
project(kilvo_agv_gazebo)

# ===== 1. CẤU HÌNH BUILD =====
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

# ===== 2. PHỤ THUỘC (livox_ros_driver: message CustomMsg trong cùng workspace) =====
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs livox_ros_driver)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

# ===== 3. NODE C++: lidar_to_livox (PointCloud2 -> Livox CustomMsg) =====
add_executable(lidar_to_livox src/lidar_to_livox.cpp)
target_link_libraries(lidar_to_livox ${catkin_LIBRARIES})
add_dependencies(lidar_to_livox ${catkin_EXPORTED_TARGETS})   # đợi sinh header message trước

# ===== 4. NODE PYTHON: cho phép chạy bằng rosrun =====
catkin_install_python(PROGRAMS
  scripts/wheel_encoder.py scripts/scenario_runner.py scripts/traj_recorder.py scripts/eval_ate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
