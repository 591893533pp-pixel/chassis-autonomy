# 【纯底盘版】NUC 部署包 —— 新机部署说明

> ⚠️ **这是纯底盘专用包**：适用于**没有可升降立柱、没有上半身（腰/双臂/头）、没有 RK3588** 的机器人，只有 **NUC + 四全向轮底盘 + 2× Livox Mid-360**。
> **不要**部署到带升降立柱、上半身或 RK3588 的完整机器人上（完整机器人用 `nuc_full_deploy_*` 包）。
> 包根目录和程序目录里都有 `CHASSIS_ONLY.txt` 标记；8870 网页右上角显示「纯底盘版」。

一句话：拷贝 → 校验 → 解压 → `./deploy.sh`（会要求确认纯底盘版）→ 按第 5 节验收。

---

## 1. 和完整版的区别

| 项目 | 完整机器人（RK3588 + 升降 + 上半身） | 本包（纯底盘） |
|---|---|---|
| 控制入口 | RK 5010 网页、PICO 遥操作 → NUC 运动服务 | **只有 NUC**：8870 网页（运动服务 + 协议终端：地图目标、速度、增量目标）、手柄独立遥控 |
| PTP 时间同步 | NUC 做从钟跟随 RK（nuc-ptp-slave / phc2sys / health） | **NUC 做主钟**（`nuc-ptp-master`：`ptp_supervisor.py` 软件时间戳主钟，双雷达跟随 NUC；即 RK 接入前的原有方式） |
| 升降立柱 | 运动服务控制升降 | 运动服务用“无升降”替身：升降始终报告失能、高度 0；只使能升降的请求被拒绝，底盘+升降一起的请求只作用于底盘 |
| 上半身自滤波 | 按 RK 关节角剔除手臂点云 | **关闭**（`navigation/config/upper_body_filter.json` enabled=false），不启动上半身 TF 中继 |
| 手柄独立遥控（8870「手柄」） | 底盘 + 升降 | 固定 `--chassis-only`（升降锁定） |
| 碰撞暂停解除 | 需要 RK 网页证明“已暂停” | 没有 RK：障碍清除 + 底盘停稳即解除（首点仍有 6 cm / 10° 检查） |
| 地图 | 全部 | **只带最新一张**（当前默认地图）+ 它的人工基底、三维点云、定位先验 |

开关是 `rk3588 link/config/chassis_only.json`（`{"enabled": true}`）。代码里所有改动都由这个开关控制；文件不存在或 `enabled` 不是 true 时行为与完整版相同（但系统时钟服务仍需按完整版安装，见第 6 节）。

## 2. 包里有什么

| 目录 / 文件 | 内容 |
|---|---|
| `CHASSIS_ONLY.txt` | 纯底盘版标记（deploy.sh 据此走纯底盘流程） |
| `deploy.sh` / `read.md` | 一键部署脚本（与完整版同一脚本，按标记分支）/ 本说明 |
| `ptp_master/nuc-ptp-master.service` | NUC PTP 主钟服务 |
| `verify.py` + `manifest.json` | 全部文件 sha256 |
| `apt-packages.txt`、`packages/linuxptp_*.deb` | 依赖清单、linuxptp 3.1.1 |
| `map_selection.json` | 本包带的那一张地图及其依赖 |
| `rootfs/home/pc/rk3588 link/` | 全部程序（已打纯底盘补丁）、最新地图 |
| `rootfs/opt`、`rootfs/usr/local`、`rootfs/etc` | 运行组件、systemd 服务、PTP 配置、sysctl |
| `environment/` | 源 NUC 环境记录 |

不包含：历史日志、建图录包、地图中间结果与导出、更早的在线地图版本、SSH 密钥、WiFi 密码、RK 端任何东西。RK 相关程序（`RK3588con` 的 RK 协议部分、`rk_web`、`nuc_gateway` 配对）文件保留但在纯底盘机器人上用不到。

## 3. 新机要求

Ubuntu 24.04 x86_64、用户 `pc`、ROS 2 Jazzy 已装、能上网；一个支持的有线网口接底盘交换机（原机 `enp85s0`，不同用 `--iface`）；PEAK XCAN-USB（`can0`）；两颗 Mid-360（192.168.16.84 / .85）。硬件与完整版底盘一致（四全向轮 RS02、雷达安装位置相同），沿用原外参与底盘参数。

## 4. 部署

```bash
sha256sum -c nuc_chassis_only_deploy_*.tar.gz.sha256
tar -xzf nuc_chassis_only_deploy_*.tar.gz && cd nuc_chassis_only_deploy_*/
./deploy.sh                    # 会先显示【纯底盘版】并要求确认
./deploy.sh --iface enp2s0     # 网口名不同
./deploy.sh --autostart        # 同时打开开机自启（验收后再开更稳妥）
```

与完整版不同的步骤：
- **第 5 步 PTP**：不安装 RK 从钟三件套（已存在则停用），安装 `nuc-ptp-master.service`；关闭 NTP（主钟取系统时间，避免运行中被校时跳变）。
- **第 6 步**：安装 `/opt/livox-navigation-home-runtime`（含主钟脚本）后启动 `nuc-ptp-master`，12 秒后检查 `/run/livox-ptp/health.json` 应为 `software MASTER True ok`。
- 其余步骤（依赖、拷贝、DDS、网络 192.168.16.4、免密入口、input 组、用户服务、自检）与完整版相同。

## 5. 验收（有人值守，**本包的 NUC 主钟部分尚未在实机上验证，必须逐项确认**）

```bash
ping -c 3 192.168.16.84; ping -c 3 192.168.16.85          # 两颗雷达（没有 RK，不需要 .36）
systemctl status nuc-ptp-master --no-pager
cat /run/livox-ptp/health.json                              # mode=software, port_state=MASTER, healthy=true
cd "/home/pc/rk3588 link"
./nuc_web.sh start && ./nuc.sh start && ./nuc.sh status     # PTP → 雷达 → 融合 → FAST-LIO → 定位 → Nav2
```

逐项确认：
1. **PTP**：health.json 健康；导航启动日志出现「纯底盘版：NUC作为PTP主钟」「ptp 健康检查通过」。
2. **雷达时间同步与融合**：`./nuc.sh status` 双雷达正常、融合 normal（雷达时间与 NUC 系统时间偏差在 ±5 ms 检查内）。若报时间相关错误，看 `navigation/logs/one_click/<最新>/` 下 01_ptp、融合日志。
3. **定位**：在带来的地图所在场地，定位 ready；换场地需重新建图（建图入口在纯底盘版上未验证）。
4. **运动（低速、随时可急停）**：8870「导航」页 ① 启动定位 → ③ 启动运动服务（控制台出现「纯底盘版：无升降立柱…」）→ ④ 连接协议终端 → 使能 → 发小速度 / 小增量 / 近处地图目标；确认碰撞监视急停有效、障碍清除后能继续。
5. **手柄**：8870「手柄」（独立遥控，与运动服务互斥，先停运动服务）：控制台显示「建图/底盘模式：升降锁定」，START 使能后摇杆驱动底盘。
6. 全部正常后再开机自启：`systemctl --user enable poke-nuc-autostart poke-nuc-web-autostart poke-cloud-latency && sudo loginctl enable-linger pc`。

## 6. 注意

- **只能有一个 PTP 主钟**：纯底盘机器人网段里不能再有 RK 或其他 PTP 主钟。`nuc-ptp-master` 与 RK 从钟服务互斥（systemd Conflicts）。
- 若这台 NUC 以后要装回完整机器人：删除 `config/chassis_only.json`、停用 `nuc-ptp-master`，并用完整版包重新部署（重装 RK 从钟服务）。
- 关机/重启 NUC 前先停运动服务（8870「停止网页运动服务」），与完整版相同。
- 改动由 `patch_chassis_only.py` 打在包内副本上，原 NUC 运行系统未改；补丁、测试见源项目 `deploy/`。
