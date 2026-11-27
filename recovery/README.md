# MID-360S 启动扫描故障自动恢复

用途：为已反复出现的 `ERROR (0x04)` + `0x02160003` / `0x02100003` 提供有次数限制的软件重启。它不是电压检测器；没有测量电压，也不证明欠压是唯一根因。供电预算与历史记录见 [启动故障证据](../evidence/lidar_boot_20261127/README.md)。

## 触发和恢复流程

1. 服务每 5 秒检查一次。雷达网口有链路、CAN 接口均未 UP、没有遥控/建图/导航控制进程，且能独占驱动锁、遥控锁和 SDK UDP 端口时，才查询雷达。网络未连接时等待，不能把超时当成故障触发重启。
2. 从 [设备配置](../mapping_trial/config/devices.json) 读取两台设备的 IP 和序列号。发送 SDK2 `0x0101` 只读查询，读取序列号、工作状态、上电次数、8 项 HMS；验证来源 IP/端口、序列号、命令号、报文序号、长度和 CRC。
3. 设备为 ERROR，**同时存在** `0x02160003` 和 `0x02100003`，没有其他 Error/Fatal，且这种状态连续观察至少 30 秒，才进入恢复。样本间隔超过 10 秒、计数变化、身份不符、查询失败、链路或设备占用都会重新计时。单独一个扫描错误暂不自动处理，留待人工诊断。
4. 先将完整故障诊断写入日志并持久化消耗次数，再发送**一次** SDK `0x0200` 重启命令，参数沿用 SDK 的 `uint16 timeout=100`。从不发送 `0x0201` 恢复出厂、不切工作模式、不改固件或 PTP。
5. 最多观察 90 秒。必须是同一序列号、上电计数恰好加 1（含 uint32 回绕）、连续 10 秒 SAMPLING (1)，且无 Error/Fatal，才记 `RECOVERED_SAMPLING`。PTP Warning 可存在，但必须继续接受原有同步门禁；本服务没有验证实际点云质量或时间同步。
6. 两台串行处理：第一台观察结束、释放端口后，第二台重新累计 30 秒故障，避免同时软件重启。每台**每次 NUC 开机最多一次**，重启本服务、链路断连或雷达重新上电都不恢复次数。失败或应答丢失也不会重发。应答丢失后仍会查询，防止把已经执行的重启再发一遍。

持续 PTP `0x04050002`、`0x04020002`、`0x04060002`、健康设备、掉线、IP 不匹配均不触发此重启。独占锁与端口保证正常项目流程不能与恢复同时占用；如果启动采集时提示端口/锁占用，等本轮恢复结束再启动。服务会在已有驱动/控制程序运行时暂停，不会在运行中的建图会话里重启雷达。

## 安装到目标 NUC

代码应位于 `/home/pc/chassis-autonomy`，目标账户为 `pc`。安装脚本仅安装和启用 systemd 服务；运行进程降权到 pc，无需抓包权限。可从操作电脑执行：

```bash
ssh -t -i /home/pc/.ssh/id_ed25519_chassis_nuc pc@10.23.2.190 'sudo bash /home/pc/chassis-autonomy/recovery/install.sh'
```

服务随 NUC 开机启动，不依赖桌面登录。安装操作需 sudo；脚本文件部署到 NUC 并不等于服务已经启用。该服务不初始化 CAN，不控制电机，不启动建图。若 CAN 已 UP，即使没有运动，也暂停查询/恢复；这是保守的运行保护。请先关闭遥控/导航，按既有流程解除 CAN 占用，或在下次开机、启动车体控制之前处理启动故障。不要为了触发恢复在车辆运动时手动拉低 CAN。

查看状态（在目标 NUC 上）：

```bash
systemctl status chassis-lidar-boot-guard --no-pager
journalctl -u chassis-lidar-boot-guard -n 60 --no-pager
cat /home/pc/.local/state/chassis-lidar-boot-guard/status.json
cat /home/pc/.local/state/chassis-lidar-boot-guard/budget.json
```

日志与计数位于 `/home/pc/.local/state/chassis-lidar-boot-guard/`：

| 文件 | 内容 |
|---|---|
| `status.json` | 最近检测时间、当前等待原因或两台诊断 |
| `events.jsonl` | 状态变化、故障现场、重启 ACK、恢复结果，追加保存 |
| `budget.json` | NUC boot_id、各序列号已消耗的次数和结果；写入 fsync 后才发送 |
| `owner.lock` | 防止第二个服务实例重复发送 |

`WAIT_LINK` 表示无链路；`WAIT_OWNERSHIP_OR_ADDRESS` 表示 SDK 端口/锁已占用或本机雷达地址未就绪；`BLOCKED_CAN_UP` / `BLOCKED_ACTIVE_PROCESS` 表示有运行冲突。`OBSERVING` 只是检查循环状态，**不代表设备健康**，要查看 `devices` 中工作状态、HMS、`query_error` 和 `automatic_attempt_used`。最终结果保留在 budget 和事件日志中。

停用：`sudo systemctl disable --now chassis-lidar-boot-guard`。不要删除 budget 来强行反复重启。若一次恢复不成功，保留日志并检查电源支路、线缆和雷达；不要用不断重启 NUC 的方式刷新次数。

## 验证边界

离线测试：`python3 -m unittest discover -s tests -p test_lidar_boot_guard.py -v`，不连接雷达或电机，涵盖 14 个测试场景，包括双雷达串行、误报排除、超时、CRC、应答丢失、计数持久化和恢复判据。

2026-11-27 开发时目标 NUC 的雷达网口没有物理链路，因此暂未实机验证新查询报文的响应和本服务自动恢复。历史人工重启成功不能替代本服务实机验证。下次供电和链路恢复后，先核对两台正常诊断查询；自然复现故障时检查是否保存 `REBOOT_EVIDENCE → REBOOT_ACK_OK / REBOOT_ACK_UNCONFIRMED → RECOVERED_SAMPLING`。不故意制造欠压、不重启健康雷达来测试。

本轮离线、SDK 报文对照和 NUC 无链路试运行证据见 [验证记录](../evidence/lidar_boot_guard_20261127/validation.json)。
