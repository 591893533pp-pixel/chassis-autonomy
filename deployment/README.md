# 部署资料的适用范围

本目录保存网络、CAN、PTP 系统参考和原部署包入口来源。`legacy_bundle_reference/deploy.sh` 依赖完整旧 rootfs/manifest/packages，并非本 Git 源码仓库的一键部署器；不要直接执行它。

当前新源码传输：`tools/stage_source.py`，目标必须为新目录。构建：`scripts/build_driver.sh`、`scripts/build_sam.sh`、`control/build.sh`。现有 NUC 的系统级 PTP/CAN 先核对后复用。

- `network/` 仍以 pc / enp85s0 / 192.168.1.4 为现场配置，网络变更前检查当前 SSH 走哪个接口。
- `system_reference/ptp_master/` 是独立底盘 NUC 主钟模式；不要与旧 RK3588 从钟、phc2sys 服务混启。
- `system_reference/livox-navigation-home-runtime/` 是旧系统受限 root 入口代码参考，内含旧设备假设，不直接拿它代替当前 `mapping_trial/raw_time_monitor.py`。
- `chassis_config/` 保留纯底盘模式与运动参数原始来源。NAV2 候选低速限制在当前主配置与门禁内另有严格校验。

本次未安装或启用 systemd 服务、未调整 sudoers、未自动自启动运动。
