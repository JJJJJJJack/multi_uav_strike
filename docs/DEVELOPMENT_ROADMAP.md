# 多无人机打击系统 - 开发路线图

> 本文档整理后续开发步骤，按优先级排序。

---

## 阶段一：核心框架搭建（P0）

### 1.1 comm_node（通信节点）✅ 已完成

**目标**：统一处理机间通信和地面站通信，从同一硬件接口获取所有信息。

**实现文件**：
- `src/comm_node.cpp` - 通信节点实现

**已实现功能**：
- 订阅地面站下发的工作模式 `/gs/mode_cmd`
- 订阅地面站下发的航点列表 `/gs/waypoint_upload`
- 发布无人机状态 `/gs/telemetry`
- 发布目标报告 `/gs/target_report`
- 发布本机位置 `/inter_uav/self_pose`
- 汇总邻居无人机位置 `/inter_uav/other_uav_poses`
- 共享目标信息 `/inter_uav/target_info`

**里程碑**：`comm_node` 已完成，可收发地面站指令，可转发机间信息。

---

### 1.2 mission_manager_node（任务管理器）✅ 已完成

**目标**：接收地面站指令，根据工作模式执行对应行为，协调已有模块。

**重要说明**：
- 工作模式是地面站预设的，不是自动切换
- 无人机上电后预加载航线和模态，筒射完毕后才执行

**实现文件**：
- `src/mission_manager_node.cpp` - 任务管理器实现

**已实现功能**：
- 定义工作模式响应：`SEARCH_ONLY`、`SEARCH_TRACK`、`SEARCH_STRIKE`
- 实现 SEARCH_ONLY 行为：航点飞行 + 云台扫描 + 回传目标信息
- 实现 SEARCH_TRACK 行为：锁定 + 螺旋定位
- 实现 SEARCH_STRIKE 行为：定位完成 → 制导打击
- 机间避障（搜索全程）
- 毫米波雷达急停逻辑（低速时）

**里程碑**：`mission_manager_node` 已完成，串联已有模块。

---

### 1.3 waypoint_executor_node（航点执行器）✅ 已完成

**实现文件**：
- `src/waypoint_executor_node.cpp` - 航点执行器实现

**已实现功能**：
- 航点队列管理
- GPS 经纬高格式转换
- 螺旋接近轨迹生成
- 机间避障向量叠加

---

### 1.4 整合测试 - 单机打击流程 ✅

**测试文件**：
- `launch/single_uav_strike_test_v2.launch` - 单机完整测试 launch
- `scripts/gs_simulator_node.py` - 地面站模拟器（测试用）

**使用方法**：
```bash
# 启动完整测试（默认 SEARCH_ONLY 模式）
roslaunch multi_uav_strike single_uav_strike_test_v2.launch

# 指定模式启动
roslaunch multi_uav_strike single_uav_strike_test_v2.launch mode:=SEARCH_STRIKE
```

---

## 阶段二：核心框架搭建 - 代码文件清单
   - 航点飞行（调用 waypoint_executor）
   - 云台扫描模式
   - 检测到目标 → 回传目标信息（不跟踪、不打击）
   - **嵌入机间避障**（搜索全程）

4. **实现 SEARCH_TRACK 行为**
   - 检测到目标 → 锁定
   - 启动 `target_estimator_node`（粒子滤波）
   - 边螺旋接近边观测定位
   - **嵌入机间避障**

5. **实现 SEARCH_STRIKE 行为**
   - 检测到目标 → 锁定 + 定位
   - 调用 `guidance_control_node` 制导接近
   - 满足打击条件 → 执行打击
   - **嵌入机间避障**

6. **实现避障触发**
   - 低速（< 12m/s）时请求毫米波雷达数据
   - 检测到障碍物 → 急停
   - 机间避障：计算势场方向，叠加到航点执行

**里程碑**：无人机可响应地面站三种工作模式，串联已有模块完成全流程。

---

### 1.3 waypoint_executor_node（航点执行器）

**目标**：将航点列表转换为飞控可执行的指令，支持 GPS 经纬高格式。

**实现步骤**：

1. **创建节点框架**
   ```
   src/planning/waypoint_executor/waypoint_executor_node.cpp
   include/multi_uav_strike/waypoint_executor.h
   ```

2. **实现基础航点飞行**
   - 订阅 `/gs/waypoint_upload` 或 `/mission/waypoint_cmd`
   - 发布 `mavros/setpoint_position/global`（GPS 经纬高）
   - 支持航点队列管理

3. **实现避障叠加**
   - 接收机间避障向量
   - 在执行航点时叠加避障方向

4. **与 mission_manager 集成**
   - `mission_manager` 发布航点命令
   - `waypoint_executor` 执行并反馈状态

**里程碑**：`waypoint_executor` 可按航点列表飞行，并响应避障指令。

---

### 1.4 整合测试 - 单机打击流程

**测试场景**：
1. 启动 `single_uav_strike_test.launch`
2. 地面站下发 `MODE_SEARCH_STRIKE` + 航点列表
3. 目标运动模拟器生成移动目标
4. 无人机按航线搜索（YOLO 检测用仿真替代）
5. 检测到目标，粒子滤波开始定位
6. 定位完成，无人机螺旋接近
7. 满足条件，打击评估成功

---

## 阶段二：避障逻辑实现（P1）

> **重要调整**：避障逻辑不独立成节点，嵌入在搜索阶段实现

### 2.1 机间避障实现

**实现位置**：`waypoint_executor_node` 或 `mission_manager_node` 搜索逻辑内

**实现步骤**：

1. **人工势场法**
   - 接收 `/inter_uav/other_uav_poses`
   - 计算邻居无人机的斥力势场
   - 叠加到目标航点方向

2. **避障触发**
   - 搜索全程开启
   - 距离 < 安全距离时激活

---

### 2.2 毫米波雷达避障实现

**实现位置**：`radar_driver_node` 内

**实现步骤**：

1. **驱动开发**
   - 接收毫米波雷达原始数据
   - 解析障碍物距离/方位

2. **急停逻辑**
   - 检测到障碍物（距离 < 阈值）
   - 速度 < 12m/s 时触发
   - 直接发布急停指令给飞控

---

## 阶段三：激光雷达型支持（P2）

> 用户表示激光雷达和 A* 会自己快速集成，此处仅提供集成指导。

### 3.1 FastLio2 集成

1. 安装 `fast_lio` 包
2. 配置 Robosense Airy 激光雷达参数
3. 发布 odom 到 `/fastlio/odom`

### 3.2 3D A* 路径规划

1. 安装 `octomap` 和 `octomap_server`
2. 实现 A* 搜索
3. 与 `waypoint_executor_node` 接口对接

### 3.3 激光雷达驱动

1. **创建节点框架**
   ```
   src/sensors/lidar_driver/lidar_driver_node.cpp
   ```
2. 适配 Robosense Airy SDK
3. 发布点云数据

---

## 阶段四：其他功能（P3）

### 4.1 gimbal_driver_node（真实云台驱动）

**目标**：替代 `gimbal_simulator_node`，驱动真实云台。

### 4.2 video_stream_node（RTSP 推流）

**目标**：视频流回传地面站。

### 4.3 boot_detect.sh（开机检测脚本）

```bash
#!/bin/bash
UAV_TYPE=$(cat /etc/uav_type)  # 或其他硬件检测方式

case $UAV_TYPE in
    "lidar")
        roslaunch multi_uav_strike lidar_main.launch
        ;;
    "strike")
        roslaunch multi_uav_strike strike_main.launch
        ;;
    "comlink")
        roslaunch multi_uav_strike comlink_main.launch
        ;;
esac
```

---

## 开发检查清单

### 已完成 ✅
- [x] 目标运动模拟器
- [x] 云台模拟器
- [x] 粒子滤波目标定位
- [x] 制导控制节点（Intercept / MinSnap / LOS）
- [x] 坐标转换（NWU/NED）

### 进行中 🔄
- [ ] LOS 制导调试（用户提到 LOS 有问题待调试）

### 待开发 📋

**P0（核心框架）**
- [ ] comm_node（通信节点，统一机间+地面站通信）
- [ ] mission_manager_node（任务管理器，响应三种工作模式）
- [ ] waypoint_executor_node（航点执行器，GPS经纬高）

**P1（避障逻辑）**
- [ ] 机间避障（嵌入在搜索阶段实现）
- [ ] radar_driver_node（毫米波雷达驱动 + 急停逻辑）

**P2（激光雷达型）**
- [ ] lidar_driver_node（Robosense Airy 驱动）
- [ ] FastLio2 集成
- [ ] 3D A* 规划

**P3（其他）**
- [ ] gimbal_driver_node（云台驱动）
- [ ] video_stream_node（RTSP 推流）
- [ ] boot_detect.sh（开机检测）

### 明确不在本包内 ❌
- YOLO 目标检测（由其他模块提供，订阅 `/detection/yolo_result`）

---

## 关键设计决策记录

| 日期 | 决策 | 原因 |
|------|------|------|
| 2026-04-25 | YOLO 不在本 package 内 | 减少计算开销，部署在 RK3588J 上 |
| 2026-04-25 | 工作模式由地面站预设 | 无人机上电预加载，筒射后执行 |
| 2026-04-25 | 避障不独立成节点 | 嵌入搜索阶段实现，减少节点数量 |
| 2026-04-25 | 机间通信和地面站通信共用 comm_node | 减少接口，从同一硬件接口获取 |
| 2026-04-25 | RTK 模拟用于衔接实物/飞控 | 仿真支持，后续好衔接真实 RTK |

---

*文档版本：v1.1*
*最后更新：2026-04-25*
