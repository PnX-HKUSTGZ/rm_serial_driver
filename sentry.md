### 架构梳理

- 将每个 IMU 的零点都定义为一个 **odom 系**。
- 共有三台 IMU：
    - 大 yaw IMU → 对应坐标系：`odom_omni`
    - pitch IMU → 对应坐标系：`odom_aim`
    - 雷达 IMU → 对应坐标系：`odom`
- 需要发送给下位机的数据：
    1. 大 yaw IMU 期望姿态（大yaw角度）
    2. pitch IMU 期望姿态（小yaw和pitch角度）
    3. 基于 `odom_omni` 的速度 `vx, vy`
- 下位机使用上述期望量 + IMU 的反馈做闭环控制。

### 模块职责设想：serial_driver

#### 1. 接收（receive）模块

**Input：**

- 三个 IMU 的四元数
- 两台电机的反馈（位置 / 速度等）

**需要解算的量：**

- `odom_omni → odom_aim` 的变换
- `odom → odom_omni` 的变换
- omni 系、aim 系下 odom 到云台的变换（`odom_* → gimble`）

**含义说明：**

- 各 odom 系之间的变换，本质上描述了几台 IMU 的零点（零点 + 零漂）之间的相对位姿关系。
- 大 yaw 与 pitch 之间的相对关系由两台电机反馈约束。
- pitch 与雷达之间为固定安装关系，可视为刚体变换。

#### 算法实现细节

1. **多传感器数据对齐**
    - **时间同步**：对齐三个 IMU 与电机反馈的时间戳（软同步或硬件触发）。
    - **空间对齐**：将各传感器原始数据转换到统一的结构坐标系定义下。
2. **运动学链推算 (TF Tree Update)**
    - 根据电机编码器（Encoder）反馈的角度，实时计算云台各连杆之间的相对变换矩阵（如 `base_link` $\rightarrow$ `yaw_link` $\rightarrow$ `pitch_link`）。
    - 引入机械安装参数，修正刚体变换矩阵。
3. **多 Odom 系融合与漂移校正**
    - **核心思想**：利用电机编码器提供的相对姿态约束，观测不同 IMU `odom` 系之间的相对漂移。
    - **一阶互补滤波 (Complementary Filter)**：
        - 利用高频的运动学推算姿态作为观测值，低频修正 IMU 的积分漂移。
        - **Bias 更新公式**：
            
            $$
            bias \leftarrow Exp(\alpha \cdot e) \cdot bias
            $$
            
            - 其中 $e$ 为误差项（如：$q_{kinematics}^{-1} \cdot q_{imu}$ 的对数映射误差）。
            - $\alpha$ 为滤波增益。
    - **输出**：实时修正 `odom_omni` 相对于 `odom_aim` 和 `odom` 的变换关系，确保全车坐标系的一致性。
4. 导航未启动时的特殊处理
    - 如果odom系不存在，则忽略和odom系相关的内容

#### 2. 发送（send）模块

**Input：**

- 基于 `odom_aim` 给出的目标：
    - 一个 pitch 角
    - 两个 yaw 角（大和小yaw）
- 基于`odom` 给出的目标：
    - `vx, vy`

**处理：**

- 将上述量统一转换到 `odom_omni` 坐标系下。

**Output：**

- 在 `odom_omni` 下的：
    - pitch / yaw 期望
        - `vx, vy` 期望
- 发送给下位机用于控制。

> 以上所有运算**计划都在 `serial_driver` 中完成**。
> 

### 接口定义

| 交互模块 | 方向 | 数据内容 | 说明 |
| --- | --- | --- | --- |
| **导航 (Navigation)** | 📥 接收 | `vx`, `vy` | 导航规划的期望速度 |
|  |  | `odom` $\to$ `base_link` | 导航系下的位姿变换 |
|  | 📤 发送 | TF Tree | 全车坐标系变换树 |
| **下位机 (Embedded)** | 📥 接收 | IMU 四元数 (x2) | 对应 IMU 的姿态反馈 |
|  |  | 电机反馈 (x2) | 小 Yaw 和 Pitch 电机的读数 |
|  | 📤 发送 | `vx`, `vy` | 基于 `odom_omni` 的期望速度 |
|  |  | 期望姿态 (Yaw, Pitch) | 大 Yaw, 小 Yaw, Pitch 的控制目标 |
| **自瞄 (Auto-aim)** | 📥 接收 | 期望姿态 (Yaw, Pitch) | 视觉解算的打击目标角度 |
|  | 📤 发送 | TF Tree | 用于预测和解算的位姿信息 |
| **全向感知 (Perception)** | 📤 发送 | TF Tree |  |

坐标系定义：

右手系，**外蕴旋转表示法（Extrinsic Rotations）**