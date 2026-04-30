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
  eigen_conversions
)

# 声明catkin包（供其他包依赖）
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf visualization_msgs
)

# 包含ROS头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 制导策略库 ==========
add_library(guidance_strategies
  src/guidance_strategies.cpp
)
target_link_libraries(guidance_strategies
  ${catkin_LIBRARIES}
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

# 4. 制导控制节点
add_executable(guidance_control_node
  src/guidance_control_node.cpp
)
target_link_libraries(guidance_control_node
  guidance_strategies
  ${catkin_LIBRARIES}
)

# ========== P0: 核心框架 ==========
# 5. 通信节点（统一机间+地面站通信）
add_executable(comm_node
  src/comm_node.cpp
)
target_link_libraries(comm_node
  ${catkin_LIBRARIES}
)

# 6. 航点执行器节点
add_executable(waypoint_executor_node
  src/waypoint_executor_node.cpp
)
target_link_libraries(waypoint_executor_node
  ${catkin_LIBRARIES}
)

# 7. 任务管理器节点（核心状态机）
add_executable(mission_manager_node
  src/mission_manager_node.cpp
)
target_link_libraries(mission_manager_node
  ${catkin_LIBRARIES}
)

# 8. 航点分发器节点（双机航点分配）
add_executable(waypoint_distributor_node
  src/waypoint_distributor_node.cpp
)
target_link_libraries(waypoint_distributor_node
  ${catkin_LIBRARIES}
)

# ========== 安装可执行文件和脚本 ==========
install(TARGETS
  target_motion_simulator_node
  gimbal_simulator_node
  target_estimator_node
  guidance_control_node
  comm_node
  waypoint_executor_node
  mission_manager_node
  waypoint_distributor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/gs_simulator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)