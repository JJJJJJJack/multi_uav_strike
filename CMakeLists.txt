cmake_minimum_required(VERSION 3.0.2)
# 包名必须和package.xml一致
project(multi_uav_strike)

# 设置编译选项（C++11兼容ROS1）
add_compile_options(-std=c++11)

# 查找依赖包：必须包含package.xml里声明的核心依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  visualization_msgs
)

# 声明catkin包（供其他包依赖）
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf visualization_msgs
)

# 包含ROS头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ========== 逐个编译四个节点 ==========
# 1. 目标运动模拟节点
add_executable(target_motion_simulator_node
  src/target_motion_simulator_node.cpp  
)
target_link_libraries(target_motion_simulator_node
  ${catkin_LIBRARIES}
)

# 2. 云台模拟节点
add_executable(gimbal_simulator_node
  src/gimbal_simulator_node.cpp
)
target_link_libraries(gimbal_simulator_node
  ${catkin_LIBRARIES}
)

# 3. 目标估计节点
add_executable(target_estimator_node
  src/target_estimator_node.cpp
)
target_link_libraries(target_estimator_node
  ${catkin_LIBRARIES}
)

# # 4. 制导控制节点
# add_executable(guidance_control_node
#   src/guidance_control_node.cpp
# )
# target_link_libraries(guidance_control_node
#   ${catkin_LIBRARIES}
# )

# ========== 安装可执行文件 ==========
# install(TARGETS
#   target_motion_simulator_node
#   gimbal_simulator_node
#   target_estimator_node
#   guidance_control_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )