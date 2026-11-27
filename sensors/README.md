# 雷达驱动与 SDK

- `Livox-SDK2/`：原部署包中完整 SDK2 源码。
- `driver_workspace/src/livox_ros_driver2/`：实际 ROS2 驱动，保留时间诊断/IMU 发布审计等已有修改。
- `config/fastdds_navigation.xml`：本机 DDS 配置。
- 构建：仓库根目录 `bash scripts/build_driver.sh`。SDK 安装在 `sensors/sdk-install/`；驱动安装在 `sensors/driver_workspace/install/livox_ros_driver2/`。

构建脚本不启动驱动、不修改雷达 IP。设备身份/网络和坐标参数使用 `mapping_trial/config/` 及标定配置。本机来源可以追溯；目标 NUC 重新连通后仍要核对是否存在尚未同步回来的现场修改。

`build_support/livox_ros_driver2` 是独立的离线消息定义包，不是这个完整驱动。
