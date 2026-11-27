# 实验操作手册

此手册使用新仓库入口。先完成编译与依赖检查；`research/`、`docs/history/` 里的旧日期路径供追溯，不直接照抄执行。

## 1. 恢复 NUC 与硬件基础状态

SSH 地址参考 `pc@10.23.2.190`；SSH 私钥留在各自电脑，仓库不携带。检查 NUC、交换机、雷达、电机的供电和链路；确认 can0 比特率、雷达 IP/MAC、主钟服务健康、手柄可读。运动供电正常不代表雷达和交换机必然有电。

网络参考脚本在 `deployment/network/`，保留旧地址与 Wi-Fi。CAN helper 在 `deployment/system_reference/sbin/`，只读审核后再按现场部署方式安装。PTP 主钟不是应用 `source env.sh` 就会启动的。

## 2. 标定或复核

在平整硬地面墙角，保证两台能看到两面墙和地面，支架牢固。先静止采 A，再平移/旋转采 B，C 用于独立验证；若看过 C 后修改方法，新增未见过的 D 留出复核。完整手册见 [calibration/操作手册.md](../../calibration/操作手册.md)，其中旧绝对目录替换为本仓库 calibration。

```bash
bash calibration/calibrate.sh check
bash calibration/calibrate.sh capture corner_new A --pose-note '两面墙和地面可见，底盘静止'
# 移位并退出遥控，再采 B、C；不要在采集中动车。
bash calibration/calibrate.sh fit corner_new --train A B --validation C
```

移位入口（会建立硬件遥控窗口，仍需手动 START）：

```bash
python3 calibration/run_reposition.py --phase motion --seconds 900
```

双扳机正常退出后再采集。输出不自动安装为运行外参；候选要保留哈希、传感器身份和独立证据。

## 3. 建图前静止检查

先终端一启动只读原始时间监测（原始抓包需要 sudo）：

```bash
sudo python3 mapping_trial/raw_time_monitor.py --seconds 3600
```

终端二运行纯传感器静止融合与后端检查，不启动控制桥，不要求 CAN 或手柄在线：

```bash
bash mapping_trial/start_mapping.sh --sensors-only --seconds 90
```

退出后检查本次 run 的报告、点云/IMU健康、TF 和导出完整性；同一时段不要另开定位或雷达采集。

## 4. 低速双雷达建图与回环

现场具备运动条件后，启动下面窗口，手动 START 后再推摇杆：

```bash
bash mapping_trial/start_mapping.sh --motion --seconds 900
```

当前配置打开 SAM 和回环。沿有结构特征的路线缓慢移动，回到可识别的旧位置，避免长时间只看一面白墙；保留闭合前后路线和现场标记。不要把一次回到屏幕起点当作已成功回环。

退出后重点检查 `runs/<时间>/optimized/` 下 `loop_candidates.csv`、`loop_factors.csv`、`graph_summary.json`、`graph_poses.csv` 和最终 PCD。由 `scripts/verify_map_export.py` 核对导出，不继续累加旧世界系点云作为“优化地图”。

查看过程：

```bash
source scripts/env.sh
python3 scripts/map_console.py
```

在自己电脑建隧道 `ssh -L 18766:127.0.0.1:18766 pc@10.23.2.190`，浏览器打开 `http://127.0.0.1:18766`。完整车体模型另外用 RViz 显示；默认轻量模型即可看占据范围。

## 5. 真数据离线回放

从目标 NUC 取回完整 rosbag2 目录后，在本机隔离域回放：

```bash
source scripts/env.sh
python3 scripts/replay_sam.py /absolute/path/to/rosbag2 --rate 1
```

记录 bag 的哈希与回放结果。现有 11 月 26 日完整 PCD 只有点云输出，不能代替包含点时间和 IMU 的原始 bag 重新做 FAST-LIO 去畸变。

## 6. 地图转导航与定位

对优化输出、地面高度与车体坐标核对后：

```bash
python3 scripts/export_navigation_map.py /absolute/optimized \
  --lio-config mapping_trial/config/lio.yaml --floor-z <核实的地图地面Z> \
  --output /absolute/navigation_map
```

尖括号参数必须替换成实测数值。检查未知区、空洞、障碍、门口和高度裁剪；这里不能直接拿合成测试地图上车。

然后在保持原始时间监测的条件下，分别开启：

```bash
source scripts/env.sh
python3 scripts/run_localization.py
# 另一终端 source 环境后：
ros2 launch ./launch/navigation.launch.py map:=/absolute/navigation_map/map.yaml
```

使用 RViz `2D Pose Estimate` 设置初始位姿，检查实际车体、扫描、地图、TF 对齐。当前网页尚不提供初始位姿设置。先在不启动电机桥的状态下验证规划、目标取消、碰撞 footprint 和定位稳定性。

## 7. 到点验收

`config/acceptance.json` 要记录当前配置哈希和逐项现场证据：车体锚点、实物包络、近场障碍、运动 PTP、地图/重定位、Nav2 规划/取消、停车/看门狗。只有完成对应测试后才能修改状态。

通过后使用 `scripts/navigation_motion.py` 的监督入口，仍需手动 START。网页用 `python3 scripts/map_console.py --navigation --allow-goals` 打开目标模式，拖拽指定点与朝向，再显式发送；检查 action 结果确认到达，不以“目标发送成功”作到点依据。

首次选开阔区域短距离目标，然后测试取消、手柄接管、故障停车，最后才进入窄通道。当前全局规划不是全向 lattice，车体能横移不代表规划器一定会选择横移路径。
