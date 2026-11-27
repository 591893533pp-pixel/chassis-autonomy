# 安装、编译与本机检查

## 环境

目标 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、GCC/CMake、Eigen、PCL、GTSAM 4.2、libusb-1.0、NumPy/SciPy/PyYAML/Matplotlib。系统 ROS 包使用 `/usr/bin/python3`；不要混用另一个 Python 环境的 ROS 扩展。

`deployment/legacy_bundle_reference/packages.tsv` 是原机软件快照，不是要求整机照单安装。`requirements.txt` 只列直接 Python 依赖；GTSAM/PCL/ROS 由系统包或已验证源码提供。

准备系统依赖（由使用者按缺失情况执行）：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libeigen3-dev libpcl-dev \
  python3-numpy python3-scipy python3-yaml python3-matplotlib python3-pytest \
  ros-jazzy-ros-base ros-jazzy-pcl-ros ros-jazzy-sensor-msgs-py \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher
```

还需可被 CMake 找到的 GTSAM 4.2；本机验证位置为 `/opt/ros/jazzy/lib/cmake/GTSAM`，不假定上述 apt 命令必然安装相同版本。PTP/抓包现场还需 linuxptp、iproute2、ethtool、arping、tcpdump；已有主钟服务要先核对后复用，不要同时开两个时钟管理程序。

## 源码构建

使用英文、无空格路径。ROSIDL 在当前环境下曾无法正确处理中文路径。

```bash
cd /home/pc/chassis-autonomy
python3 tools/prepare_workspace.py
python3 tools/check_repository.py
python3 tools/test_offline.py
bash control/build.sh
bash scripts/build_driver.sh
bash scripts/build_sam.sh
source scripts/env.sh
python3 scripts/check_environment.py
```

这些编译与检查不会启动电机。`build_driver.sh` 构建 SDK 和真正的雷达驱动；`build_sam.sh` 默认使用它的安装产物。

若暂不构建真驱动，可用 `bash scripts/build_sam.sh --local-message-only` 编译离线回放/合成测试所需的消息定义和 SLAM。`build_support/livox_ros_driver2` 不是采集驱动，不可拿它替代实际驱动。

源码中保留的供应商 `build.sh`、旧 `deploy.sh` 属于历史/上游入口；本项目优先使用上述主入口。整理时不修改系统服务，不设置开机自动运动。

## 离线合成验证

构建后：

```bash
source scripts/env.sh
python3 tests/synthetic_sam_smoke.py
python3 tests/synthetic_sam_smoke.py --localization
```

检查新生成 `reports/synthetic_*/report.json` 的实际 loop_factors，而不只看进程退出码。该测试没有硬件，不验证真实标定/PTP/窄通道或自主导航。

浏览器测试额外需要 `google-chrome` 命令可用的 Chrome；`tests/browser_smoke.py` 是本机辅助，不是基础 CI 的必要依赖。

## CAD 显示

默认轻量模型不需要网格。完整模型需先导入：

```bash
python3 tools/assets.py import --group cad
python3 tools/prepare_workspace.py
source scripts/env.sh
ros2 launch ./launch/display.launch.py full_mesh:=true
```

跨机器使用 `tools/assets.py import --source-root /path/to/exported-assets --group cad`。模型 URI 已改为 `package://chassis_id_me/...`；prepare 只在被忽略的 `.runtime/` 下建立本机 ament 索引。

## 当前保留的系统耦合

雷达驱动锁仍是 `/home/pc/lidar test/logs/driver.lock`，以便与 NUC 上旧实验程序互斥；当前目标账号是 pc。系统时钟健康路径为 `/run/livox-ptp/health.json`。新账号/新机器应统一迁移所有相关入口的锁路径，而非只改其中一个造成并发占用。

`deployment/system_reference/` 是系统配置参考，`legacy_bundle_reference/` 不是完整离线安装包。全新系统要先完成 CAN/PTP/网络基础部署；本仓库不是自动配置系统权限的一键镜像。
