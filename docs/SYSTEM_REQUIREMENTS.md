# 多无人机打击系统 - 系统需求文档

> 本文件记录系统目标、架构设计、模块划分及开发路线图。

---

## 1. 系统概述

### 1.1 项目背景
多无人机协同打击系统，支持三型无人机（激光雷达型、战斗部型、通信链路型）共用同一套机载软件，通过开机检测脚本自动识别机型并加载对应配置。

**部署目标**：RK3588J 机载计算机（算力有限，需控制计算开销）

### 1.2 三型无人机定义

| 机型 | 硬件特征 | 核心功能 |
|------|----------|----------|
| **激光雷达型** | Robosense Airy 激光雷达 | FastLio2 定位、3D A* 路径规划、点云/网格图建图 |
| **战斗部型** | 携带战斗部，无激光雷达 | 自主制导打击、粒子滤波定位、三种工作模式 |
| **通信链路型** | 最简单配置 | 航点飞行、视频推流、避障、急停 |

### 1.3 通用功能（所有机型必备）

- 两轴云台（俯仰 + 滚转）
- 光电吊舱（广角 + 长焦 + 可见光 + 红外）
- YOLO 目标识别（**不在本 package 内**，由其他模块提供检测结果）
- 云台自动对准 / 跟踪目标
- 前向毫米波雷达（低速巡航时障碍检测 → 急停）
- 航点飞行能力
- RTSP 视频推流（回传地面站）
- 机间通信（接收其他无人机位置）
- 机间避障（距离近 → 人工势场法）

---

## 2. 战斗部型 - 工作模式详细定义

### 2.1 三种工作模式

**重要说明**：工作模式由地面站预设并下发，无人机上电后预加载航线列表和任务模态。飞控板收到任务数据后等待筒射完毕再执行。

| 模式 | 代码 | 行为描述 |
|------|------|----------|
| **全图搜索** | `MODE_SEARCH_ONLY` | 按预设航线飞，云台扫描；识别到目标仅回传信息（经纬高+图像），不跟踪、不接近、不打击 |
| **搜索即跟踪** | `MODE_SEARCH_TRACK` | 识别到目标 → 锁定 → 启动粒子滤波定位；无人机边接近边螺旋观测，不打击 |
| **搜索即打击** | `MODE_SEARCH_STRIKE` | 识别 → 锁定 → 定位 → 制导接近 → 满足条件直接打击 |

### 2.2 工作流程

```
地面站
   │
   ├── 预加载航点列表 ──→ 飞控板（等待筒射完毕）
   │
   └── 下发工作模态（MODE_SEARCH_ONLY / MODE_SEARCH_TRACK / MODE_SEARCH_STRIKE）
             │
             ▼
        无人机执行
             │
             ├── 搜索阶段：航点飞行 + 云台扫描 + 机间避障
             │                   │
             │                   └── 低速（<12m/s）时开启毫米波雷达避障
             │
             └── 目标响应：
                     ├── 搜索即跟踪：锁定 + 螺旋定位
                     └── 搜索即打击：定位完成 → 制导打击
```

### 2.3 模式响应规则

| 收到的模式 | 无人机行为 |
|------------|-----------|
| `MODE_SEARCH_ONLY` | 执行预设航线，云台扫描；检测到目标 → 回传目标信息（经纬高+图像）；不跟踪、不打击 |
| `MODE_SEARCH_TRACK` | 检测到目标 → 锁定 + 启动粒子滤波；边螺旋接近边观测定位；不打击 |
| `MODE_SEARCH_STRIKE` | 检测到目标 → 锁定 + 定位 → 制导接近 → 满足条件打击 |

---

## 3. 核心模块架构

### 3.1 模块划分

```
multi_uav_strike/
├── core/                       # 核心节点
│   ├── mission_manager_node/      # 任务管理器（接收地面站指令，执行工作模态）
│   ├── comm_node/                 # 通信节点（机间通信 + 地面站通信共用，从同一硬件接口获取）
│   └── avoidance_handler/          # 避障处理（嵌入在搜索阶段，非独立节点）
│
├── sensors/                    # 传感器驱动
│   ├── gimbal_driver_node/        # 云台驱动（待开发）
│   ├── gimbal_simulator_node/     # 云台模拟器（已实现，用于仿真）
│   ├── radar_driver_node/          # 毫米波雷达驱动（待开发）
│   └── lidar_driver_node/         # 激光雷达驱动（Robosense Airy，待开发）
│
├── perception/                 # 感知层
│   └── target_estimator_node/     # 粒子滤波目标定位（已实现）
│       ⚠️ 注意：YOLO 不在本 package 内
│
├── planning/                   # 规划层
│   ├── path_planner_node/         # 3D A* 路径规划（激光雷达型专属）
│   └── waypoint_executor_node/    # 航点执行器（将航点转换为飞控可执行的GPS经纬高）
│
├── guidance/                   # 制导控制
│   ├── guidance_control_node/     # 制导控制节点（已实现）
│   └── guidance_strategies/       # 制导策略库（已实现）
│
├── localization/               # 定位（激光雷达型专属）
│   └── fastlio2_node/             # FastLio2 定位（待集成）
│
├── simulation/                 # 仿真支持
│   ├── target_motion_simulator_node/  # 目标运动模拟（已实现）
│   └── rtk_simulator_node/           # RTK模拟（衔接实物/飞控，仿真用）
│
└── utils/                      # 工具
    └── boot_detect.sh/             # 开机检测脚本（待开发）
```

### 3.2 模块设计原则

1. **YOLO 不在本 package 内** - 目标检测由独立模块提供，本 package 只订阅 `/detection/yolo_result`
2. **通信节点统一** - 机间通信和地面站通信共用一个 `comm_node`，减少接口和计算
3. **避障逻辑嵌入** - 不独立出避障节点，在搜索阶段的航点执行逻辑内实现
4. **RTK 模拟目的** - 为仿真提供定位真值，衔接实物（真实 RTK）和飞控（期望输入）

### 3.3 已有节点说明

| 节点 | 文件 | 功能 | 状态 |
|------|------|------|------|
| `target_motion_simulator_node` | [target_motion_simulator_node.cpp](src/target_motion_simulator_node.cpp) | 目标运动模拟（车辆二维运动模型） | 已实现 |
| `gimbal_simulator_node` | [gimbal_simulator_node.cpp](src/gimbal_simulator_node.cpp) | 云台 LOS 角度计算、跟踪控制 | 已实现 |
| `target_estimator_node` | [target_estimator_node.cpp](src/target_estimator_node.cpp) | 粒子滤波目标三维定位 | 已实现 |
| `guidance_control_node` | [guidance_control_node.cpp](src/guidance_control_node.cpp) | 制导控制（支持 Intercept/MinSnap/LOS） | 已实现 |

### 3.4 已有制导策略

| 策略 | 类 | 说明 |
|------|-----|------|
| Intercept | [InterceptGuidance](include/multi_uav_strike/guidance_strategies.h#L56-L76) | 预测拦截点，速度指令 |
| MinSnap | [MinSnapGuidance](include/multi_uav_strike/guidance_strategies.h#L78-L128) | 最小snap轨迹生成（未完成） |
| LOS | [LosGuidance](include/multi_uav_strike/guidance_strategies.h#L130-L159) | 视线角误差制导，姿态控制 |

---

## 4. 避障逻辑详细说明

### 4.1 避障策略（嵌入搜索阶段）

| 场景 | 触发条件 | 行为 |
|------|----------|------|
| **机间避障** | 搜索全程 | 人工势场法，计算邻居无人机势场方向，叠加到航点执行 |
| **毫米波避障** | 速度 < 12m/s | 毫米波检测到障碍物 → 立即停机 |

### 4.2 实现位置

- 机间避障：在 `waypoint_executor_node` 或 `mission_manager_node` 的搜索逻辑中实现
- 毫米波避障：在 `radar_driver_node` 中实现急停逻辑，直接干预飞控

---

## 5. 通信架构

### 5.1 comm_node（通信节点）

**职责**：统一处理机间通信和地面站通信，从同一硬件接口获取所有信息

**输入**（来自硬件接口）：
- 地面站指令（工作模式、航点列表）
- 其他无人机位置

**输出**：
- 本机状态回传（位置、目标信息）
- 共享给其他无人机的目标信息

### 5.2 接口协议

**地面站通信**：

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/gs/mode_cmd` | std_msgs/String | GS → UAV | 工作模式预设 |
| `/gs/waypoint_upload` | nav_msgs/Path | GS → UAV | 航点列表 |
| `/gs/telemetry` | custom_msgs/Telemetry | UAV → GS | 无人机状态回传 |
| `/gs/target_report` | custom_msgs/TargetReport | UAV → GS | 目标检测/定位结果 |

**机间通信**：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/inter_uav/self_pose` | geometry_msgs/PoseStamped | 本机当前位置 |
| `/inter_uav/other_uav_poses` | geometry_msgs/PoseArray | 邻居无人机位置列表 |
| `/inter_uav/target_info` | custom_msgs/TargetInfo | 共享发现的目标信息 |

---

## 6. 部署说明

### 6.1 目标平台

- **机载计算机**：RK3588J（算力有限）
- **优化要求**：
  - YOLO 不在本 package 内运行
  - 避免不必要的计算节点
  - 粒子滤波等算法需考虑实时性

### 6.2 开机流程

```
1. 开机检测脚本判断机型（激光雷达型/战斗部型/通信链路型）
2. 加载对应的环境变量和 launch 文件
3. 通信节点连接地面站
4. 预加载航点列表和工作模态（但不执行）
5. 筒射完毕，飞控解锁
6. 开始执行预设任务
```

---

## 7. 坐标系统约定

本系统使用 **NED → NWU** 坐标转换约定：

| 坐标系 | X | Y | Z |
|--------|---|---|---|
| **NED** | North（北） | East（东） | Down（下） |
| **NWU** | North（北） | West（西） | Up（上） |

**转换规则**（在 `guidance_control_node.cpp` 和 `gimbal_simulator_node.cpp` 中实现）：
- 位置：`x_nwu = x_ned, y_nwu = -y_ned, z_nwu = -z_ned`
- 四元数：`w,x 不变，y,z 取反`

---

*文档版本：v1.1*
*最后更新：2026-04-25*
