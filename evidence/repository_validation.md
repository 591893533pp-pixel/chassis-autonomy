# 整理版验证记录

2026-11-27，在 `/home/pc/chassis-autonomy` 逻辑英文路径构建与测试。原调试目录不变，没有连接电机/启动雷达驱动/修改远端 NUC。

| 验证 | 结果 |
|---|---|
| 控制库 CMake + CTest | 7/7 通过，SocketCAN 用本地 datagram |
| Livox SDK2 + 完整 ROS2 driver | 编译、安装通过；未实际采集 |
| FAST-LIO-SAM | 对完整驱动消息定义构建通过 |
| Python | 77/77 通过，另有融合坐标脚本断言通过 |
| 新二进制合成回环 | 17 关键帧、1 实际回环因子、34,000 导出点 |
| CAD 全模型资源 | 59 个 visual mesh 引用均可解析 |
| 本地资产导入 | 68 个文件、1,036,153,288 bytes，哈希核对通过 |
| 源码检查 | Python/Bash 语法、主配置、文档链接、二进制/大文件/典型凭据检查通过 |

新二进制哈希、测试结果与缺失运行包见 [repository_validation.json](repository_validation.json)，具体日志在 `repository_20261127/`。未重新宣称定位 TF、网页、Nav2 实车或运动安全验收；此前对应的合成测试仍保留在 reports 与 integration_20261127 中。

本机仍缺 Nav2 map_server、AMCL、Smac、DWB、BT、collision_monitor、lifecycle_manager、RViz 等运行包。配置门禁保持 false。
