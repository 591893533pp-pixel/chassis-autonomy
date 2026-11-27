# 研发说明与代码导航

## 1. 控制层

机体指令是 `(vx, vy, wz)`，轮组运动学把平移和绕竖轴转速映射到四轮目标。CAN 协议负责编码、反馈、使能和失能，不能只发送非零速度而忽略驱动状态。

- `control/program/include/poke/chassis_kinematics.hpp`：底盘运动学。
- `control/program/include/poke/chassis_controller.hpp`：轮组控制。
- `control/program/third_party/elmue_can/`：SocketCAN 和电机协议。
- `control/program/remote/joystick_layout.py`：XInput / DInput 轴键映射。
- `control/program/remote/vendor/navslam_core.py`：手柄状态逻辑。
- `mapping_trial/guarded_remote.py`：受建图/导航监督的低速遥控入口。

本项目保留控制库的升降兼容源码以满足依赖，不代表本底盘装有或应启用升降。改轮径、轮距、符号或电机 ID 后，先离线验证运动学，再逐轴低速复核。

## 2. 双雷达标定

本次主要估计**外参**，不是重新估计雷达厂家内部光学参数。目标变换为 `p_rear = R × p_front + t`，包含 3 个平移和 3 个旋转自由度。雷达到车体的绝对安装位置属于另一个锚定问题；只靠两台点云互相对齐，不能唯一恢复车体机械原点。

墙角提供两面不平行的墙，地面补充第三个平面方向，减少某个方向平移或转角不可观。先保留驱动原始坐标，再用初始安装估计建立匹配；点到面残差为 `r = nᵀ(Rp+t-q)`，优化使有效对应的残差平方和变小。RMSE 是这些几何残差的统计，**不是 NUC 与雷达的时间同步误差**。

- `calibration/capture.py`：身份/时间检查、静止点云和 IMU 采集。
- `calibration/static_policy.py`：历史 HMS 告警在静止分析中的资格判定。
- `calibration/registration.py`：降采样、法线、配准、变换及可观性相关计算。
- `calibration/calibrate.py`：采集、拟合与验证流程。
- `research/calibration_validation/fit_common_planes_candidate.py`：共同平面候选研发过程。
- `research/calibration_validation/validate_frozen_candidate.py`：冻结候选的独立复核。

A/B 用于拟合，已看过的 C 只能诊断；看 D 前冻结算法、候选和配置哈希，D 作留出验证。D 的 RMSE 20.136 mm、P95 39.438 mm 通过相对检查，不等价于地面和车体绝对外参全部合格。两雷达地面仍相差约 15.6–23.1 mm，需继续定位来源。

## 3. 时间同步与融合

PTP 通过主从时钟交换 Sync/Follow_Up、Delay_Req/Delay_Resp 估计偏移与链路延迟。网卡、交换机、调度和设备固件都可能影响观测。当前软件主钟、UTC 偏移约定必须与驱动一致；不能把设备诊断里的 ±500 ms 直接加到点云时间戳。

`timing/` 解析抓包；`mapping_trial/raw_time_monitor.py` 监测原始四路数据（两台点云 + 两台 IMU）；`live_gate.py` 与 `timing_policy.py` 为业务提供新鲜度和故障门禁。原始包接收时间减设备时间包含网络与调度延迟，不能直接称为真实同步精度。

`fusion_core.py`/`fusion_node.py` 按采样时间匹配、转换到统一雷达坐标并合并点；`cloud_adapter.py` 接入 FAST-LIO 所需消息。去畸变依赖每点时间和 IMU，在运动中尤为重要。静止标定可接受的历史告警策略不能直接套用于运动。

## 4. FAST-LIO 前端

`mapping_ws/src/fast_lio_sam/src/IMU_Processing.hpp`：IMU 传播和扫描去畸变；`preprocess.cpp`：点云预处理；`laserMapping.cpp`：里程计与局部地图；`include/ikd-Tree`：增量空间索引（以实际源码目录为准）。

前端持续估计运动，累积漂移不可避免。回到曾经经过的地方，前端轨迹也未必自动闭合。后端对离散关键帧施加约束，并不意味着直接重新估计每一帧原始测量。

## 5. 回环与因子图

`src/mapOptimization.cpp` 是主要修改入口。关键帧条件满足后，`saveKeyFramesAndFactor()` 的工作可拆成：

1. 用关键帧索引作为 GTSAM 变量键，取前端当前位姿。
2. `addOdomFactor()` 加相邻关键帧相对运动因子；场景、回环和待提交约束加入本轮图。
3. `isam->update(graph, initialEstimate)` 把新变量和因子交给增量优化器；历史图由 iSAM2 保存。
4. 清空的是本轮待提交容器，不是把历史图删除；再读取最新优化估计。
5. 保存优化位姿；将单帧雷达点变换到 IMU/body 系，在同一锁内追加索引相关容器。
6. 更新路径与边缘协方差；导出的单帧 PCD 仍为雷达系，必须与内存 IMU 系区分。

因子图节点是位姿 `X_i`，里程计/回环边是相对观测 `Z_ij`，典型误差为 `Log(Z_ij⁻¹ X_i⁻¹ X_j)`，由噪声模型加权。回环识别“重访旧位置”，优化负责把这一约束传播给整条关键帧轨迹。

当前回环采用空间候选 + 时间隔离 + ICP，不含 ScanContext。ICP 需通过收敛、MSE、重叠率、修正幅度等门限；几何重复场景仍可能误闭环。默认候选搜索 2 m、时间隔离 30 s、ICP 最大对应距离 0.5 m、MSE ≤0.04 m²、0.2 m 内重叠率 ≥0.6，位姿修正 ≤0.5 m / 0.35 rad。`mapping_trial/config/lio.yaml` 为实际配置依据。

日志 `loop_candidates.csv` 记录候选，`loop_factors.csv` 记录真正入图边；开启开关、发现候选或跑完优化线程都不能证明发生了回环。最终地图必须用优化后位姿重新拼接关键帧，不能只把修正后的轨迹画在旧累计点云上。

## 6. URDF 与占据空间

URDF 描述 link/joint 的刚体关系。处理网格顶点时应依次应用关节变换、visual origin 和 mesh scale；CAD link 原点通常不是雷达光学中心。连续转动轮子用解析扫掠范围纳入包络。

`scripts/build_body_model.py` 生成轻量 URDF 和 footprint；`footprint_check.py` 检查多边形与障碍、未知区、越界，以及转动和平移间的扫掠。当前包络为每部件 AABB 的保守凸包，不是任意机械变形模型。

原始 CAD 没有 collision 几何；只显示一个漂亮模型不会让 Nav2 自动避开车身碰撞。`make_nav_config.py` 将 footprint 送入代价地图与碰撞相关配置，`nav_sensor_bridge.py` 提供独立近场障碍数据，避免沿用建图 1 m 裁剪导致车边盲区。

## 7. 定位和到点

导航模式使用 AMCL：以粒子分布表示地图上的位姿假设，经运动更新与观测权重、重采样得到估计。它与 iSAM2 因子图是不同方法，本项目不是在导航中用两个全局定位器同时发布同一 TF。

Smac Hybrid 在配置的运动模型下做全局路径搜索，DWB 局部控制评估短期运动轨迹。全局配置尚非全向 lattice，因此可能拒绝某些物理上可横移通过的路线。`send_goal.py`/网页发送 NavigateToPose 目标；发送成功只代表动作请求受理，不代表已到达。

## 8. 开发约定

- 改算法在 canonical 源码中改，不直接改 `research/` 的历史副本。
- 固定配置和训练集后再看留出集；新数据、新方法需新版本结果目录。
- 地图、轨迹、外参、URDF、时间策略与程序版本一并记录哈希。
- 记录失败和被拒绝的候选，不能只保留成功截图。
- 修改几何、TF、时间、watchdog 后，旧实车验收不能自动沿用。
