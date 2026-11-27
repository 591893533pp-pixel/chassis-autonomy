# 底盘控制模块

这里是唯一主控制源码。原说明见 [README.MD](README.MD)，它包含上游升降功能和当时部署信息；当前独立底盘工作流程只使用 chassis-only 模式。

- 编译与无硬件 C++ 测试：`bash control/build.sh`（从仓库根目录执行）。
- 静止/低速移位入口：`calibration/run_reposition.py`，默认 stationary；运动需显式 `--phase motion` 和人工 START。
- 受建图/导航监督的控制入口：`mapping_trial/guarded_remote.py`，由监督进程管理许可。
- Python 手柄核心测试见 `tools/test_offline.py`；C++ SocketCAN 测试使用本地 datagram，不访问物理总线。

不要直接把库中 `single_wheel`、原升降测试或旧桌面快捷方式当成当前标准入口。当前低速测试阈值、退出和看门狗约定见 [实验手册](../docs/guides/experiments.md)。
