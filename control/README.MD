# 启动指令

一键启动底盘＋升降柱手柄控制（CAN自动配置；可能需要本机sudo密码）：

~~~bash
cd "/home/pc/rk3588 link/remote control"
./start.sh
~~~

升降默认10mm/s，命令上限150mm/s；指定50mm/s：

~~~bash
./start.sh --lift-speed-mm-s 50
~~~

正常停止：主终端Ctrl+C；也可另开终端：

~~~bash
cd "/home/pc/rk3588 link/remote control"
./stop.sh
./status.sh
~~~

不接触硬件的预览与重新编译：

~~~bash
./start.sh --dry-run
./build.sh
~~~

桌面方式：双击本目录“启动.desktop”，如文件管理器询问，选择允许运行。它打开终端并调用相同start.sh，不额外创建另一套控制器。

# 使用说明

1. 连接DINPUT手柄、CAN can0及设备电源。程序配置经典CAN 1Mbps；已启动且参数相同的接口复用，参数冲突就拒绝更改。
2. 启动不会立即运动。松开所有按键、摇杆和扳机，再按START进入使能状态。
3. 左摇杆/方向键平移，右摇杆旋转，LB降档/RB升档。默认底盘平移上限1.5m/s，共30档，一档0.05m/s。右摇杆旋转独立于平移档位，上限0.4rad/s（约23°/s），摇杆幅度连续控制；LT仍可减速，RT不突破旋转上限。旋转加速度仍为0.30rad/s²，从零到最高转速约1.33秒。方向键配合扳机的弧线转弯仍随平移档位变化。
4. X按住升、Y按住降，松开写零速。默认允许底盘与升降同时操作；松开升降键只停升降，松开摇杆/方向键只停底盘。双升降键/直接反向仍要求松开后重按。首次使能仍需双轴静止和输入中立；需要旧互锁模式时使用 --exclusive-motion。
5. 双扳机退出并请求停车；START也可解除手动使能请求。正常退出执行底盘停止/失能及立柱零速→失能→NMT pre-op。

# 升降限制和验证边界

- 使用2026-09-24已实测的PV速度模式（mode3/60FF），没有返回旧PP分段/HALT方式。
- 编码器换算沿用800×7/48 counts/mm；0–395mm是既有持久零点坐标，启动不重新归零，不自动清除故障。
- 请求速度必须大于0且不超过150mm/s；按整数计数向下量化以不超请求上限。反馈超过150mm/s触发停止。默认10mm/s延续既往实测量级，50mm/s尚未做本轮机械验证。
- 接近上下端按距端点剩余距离降速，保留0.5mm余量；这是软件边界，不保证机械瞬停，也不表示可以精确碰到0或250mm。
- 本轮只做代码整合、独立编译和离线测试，没有启动电机。此前10mm/s日志不能替代50mm/s、两端和不同负载验收。首次50mm/s应在行程中部、有急停可用条件下验证松键停车。
- 原手柄读取使用Linux缓存状态；不能把USB仍连接解释为无线链路一定健康。

# 开发与目录

- start.sh / stop.sh / status.sh：唯一入口、正常停止、只读状态。
- build.sh：只在本目录构建与测试C++底盘桥接，不覆盖旧程序。
- scripts/remote_session.py：单手柄读取、PV工作线程、底盘/升降互锁、进程收尾。
- scripts/lift_policy.py：150mm/s硬上限、0–395mm与接近端点策略。
- program/：底盘C++完整源码、vendor、构建文件及本目录生成的build/remote_bridge。
- lift_reference/：原PV控制与验证源码副本；集成仅复用Bus/Joystick/使能/反馈检查，不运行其独立main。
- config/defaults.json：可修改默认底盘速度、初始档、默认升降速度；修改也不能突破代码限制。
- logs/：每次唯一会话目录，含commands.jsonl、feedback.csv、bridge.log、lift_feedback.jsonl、lift_events.jsonl、CAN原始收发、joystick.jsonl、session/exit信息；console日志记录终端输出。LAST_LOG.txt指向最近日志。
- tests/与VERIFICATION.md：本轮实际验证，不冒充实机验收。

实际CAN通信分工：C++桥只启用四轮；Python PV线程只发立柱node1标准SDO/NMT。互斥锁与旧遥控相同，不允许同时开两个控制会话；不要另外运行旧PV或电机调参工具。

底盘速度参数示例：

~~~bash
./start.sh --max-linear-speed 0.3 --initial-gear 6 --lift-speed-mm-s 10
~~~

本目录为新增副本。原/home/pc/lidar test/chassis_lift_control及历史日志没有删除或改写。




## 当前升降参数（2026-10-08）

配置统一位于 config/lift_limits.json，当前仍使用PV速度模式。

| 参数 | 当前值 | 生效范围 |
|---|---:|---|
| lift_max_profile_velocity_mm_s | 150 mm/s | 遥控请求/输出及反馈速度上限 |
| lift_max_acceleration_m_s2 | 0.30 m/s² | PV正常速度指令轨迹，内部换算300mm/s² |
| lift_max_jerk_m_s3 | 1.50 m/s³ | PV正常速度指令轨迹，内部换算1500mm/s³ |
| lift_max_position_mm / lift_teleop_max_position_mm | 395 mm | 启动、运行、反馈与恢复检查，上限前0.5mm停止发出继续运动目标 |
| lift_csp_max_step_mm | 2 mm | 已保存，CSP专用；PV不启用 |
| lift_csp_following_error_mm / timeout | 25 mm / 20 ms | 已保存，CSP专用；PV不启用 |

默认请求速度仍10mm/s，默认./start.sh不会直接升到150。指定请求速度示例：./start.sh --lift-speed-mm-s 50；最高允许--lift-speed-mm-s 150。可先./start.sh --dry-run --lift-speed-mm-s 150查看配置，不访问CAN。超150直接拒绝。

平滑轨迹使用本目录已有Ruckig库新增的liblift_profile.so；正常速度过渡遵守上述加速度/jerk限制。松键、故障和端点速度封顶优先停车，可覆盖平滑限制，所以这些限制不是紧急停车减速度承诺，也不是已测得的机械加速度。整数速度计数量化也会影响离散差分。未改驱动器非易失参数或写入6083/6084，实际驱动器内部加减速度本轮未回读。请求端加速度/jerk与电机实际响应需实测核对。

本次仅更新独立遥控目录；保留PV、底盘/升降同时操作、原持久位置零点与故障保护。未切换CSP，不会把2mm/25mm/20ms冒充PV有效限制。建图目录副本仍禁用升降，历史试验脚本仍保留旧限值。此前历史段中的50mm/s/250mm为当时状态，现行值以上表为准。

源码备份backups/lift_limits_20261008_134629。构建及6项C++测试、51项Python测试通过；测试包含正反向150mm/s轨迹的加速度/jerk、松键立即零目标、395mm边界与超速拒绝。未开启CAN、未做真实150mm/s或395mm端点测试，不据此声明机械全行程/高速通过。详见logs/lift_parameters_20261008/REPORT.md。
## 2026-09-30 遥控断续修正

启动指令仍为 cd "/home/pc/rk3588 link/remote control" && ./start.sh。旧进程应先正常退出，再启动新版。修正单点零速互锁在反馈微动时反复切断：200ms窗口、150ms证据，车轮0.1/0.15rad/s进入/退出迟滞；保留明显运动、断流、故障与原停车保护。速度/PI未变。

28项离线测试和原日志回放通过，实机待复测。先1档分别测试底盘持续按住/松开，再行程中部升降持续按住/松开。完整参数、证据与命令见 logs/stationarity_fix_20260930/REPORT.md；旧文件已备份。


## 残留升降使能恢复（2026-09-30）

如果启动提示“升降已使能或运动，拒绝接管”，先正常关闭旧遥控、确认没有其他CAN控制器。保留启动保护，不反复按START。独立恢复命令只写PV目标零速、确认停车、失能并回读，不使能、不归零、不清故障：

~~~bash
cd "/home/pc/rk3588 link/remote control"
./recover_lift.sh --hardware
./start.sh
~~~

仅当恢复明确成功才执行start.sh；恢复失败时不要继续启动。直接运行./recover_lift.sh而不加参数只展示说明、不访问CAN。恢复与遥控共享进程锁，但不应把锁解释为能够识别所有第三方CAN控制器。

本次两次启动读取升降statusword=5687(0x1637，OperationEnabled)，第二次实际速度为0。原会话缺少正常清理/exit记录，具体中止原因未确定。本次17:13执行零速→停车确认→6040=0→node1 pre-op，实际回读statusword=4688(0x1250，非使能)、速度0、目标0；位置213.4371→213.4286mm（约1编码器计数差）。没有发送非零目标或使能命令。日志logs/recover_lift_20260930_171309_852857。3项假CAN恢复测试通过，原启动拒绝接管逻辑未改。本次没有替用户重新启动遥控，连续性仍待复测。

日常退出先松开输入再Ctrl+C，等待停车/失能结束；不要直接强杀进程。断电、强杀等情况不能依赖软件finally完成清理。


## 2026-09-30 同时控制更新（现行行为）

默认入口允许独立遥控同时平移/旋转与升降：

~~~bash
cd "/home/pc/rk3588 link/remote control"
./start.sh
~~~

旧遥控需要先松开所有输入、Ctrl+C正常退出并等待清理；然后再启动，代码不会热更新。START首次使能时先保持中立，待升降使能完成后再操作。保持摇杆/方向键的同时按住X/Y即可升降。任一控制器故障仍结束会话并尝试停止两轴；首次使能、反馈/心跳、0–250mm、默认10mm/s、50mm/s上限及松键停车规则保留。

恢复旧互锁模式：./start.sh --exclusive-motion。底盘/建图模式：./start.sh --chassis-only（不创建升降工作线程）。本轮只更新本目录，未更新mapping内部副本、地图、定位或PTP，也未启动实机运动。软件允许同时操作不代表负载/重心变化下的稳定性已完成验收，首次用1档与默认10mm/s短距离确认。

43项离线测试通过，包括混合输入保持、分别松开、首次中立使能、假CAN升降在线轮动时连续控制、心跳过期停止及旧互锁/仅底盘模式。详见logs/simultaneous_control_20260930/REPORT.md。先前互锁说明为历史版本，现行默认以上述同时控制为准。


## 2026-10-08 控制/反馈100 Hz更新（当前）

启动仍为：
```bash
cd "/home/pc/rk3588 link/remote control"
./start.sh
```

保持中立、START完成使能后，可在另一终端检查实际频率（只监听，不发送CAN）：
```bash
cd "/home/pc/rk3588 link/remote control"
./check_rates.sh --can-seconds 10
# 只分析最新会话日志：
./check_rates.sh
```

目标：手柄命令调度、四轮速度发送、四轮反馈消费/日志、独立PV升降的反馈读取及速度写入均100Hz（10ms）。RS02主动反馈原本默认10ms，保留主动上报机制，实际每轮线速率用上述监听核实。升降每周期重新读取6064/606C/6041/6061四项SDO并写60FF，包括不变/零目标；不是将缓存反馈重复发布成100Hz。五次串行SDO事务需在10ms预算内完成，受驱动响应和Linux调度影响，100Hz是目标、并非已通过实机验收。CAN当前DOWN，修改时没有使能或移动硬件。

定时采用单调时钟绝对周期，超时跳过错失时隙，不补发积压命令。lift_timing.jsonl包含实际周期、工作耗时、missed_slots；feedback.csv增加missed_control_slots，commands.jsonl记录主循环超时。保留300ms心跳、0.5s停车检查、反馈有效性及原150mm/s/395mm限制；默认升降请求仍10mm/s，PI/加速度/jerk未改。旧代码与可执行文件在backups/rates_100hz_20261008_153046。

总线估算：4轮命令+4轮主动反馈约800帧/秒，升降四读一写约1000帧/秒，加起来约1800帧/秒；1Mbps经典CAN约26%线占用（保守帧长估算，未包含其他响应/心跳/设备流量）。check_rates.sh输出全帧数量和粗略总线占用，用真实数据确认。主循环100Hz不代表手柄硬件USB报告率变成100Hz。

建图集成副本同步了底盘100Hz调度，继续chassis-only，未移植升降新行程参数；导航速度桥改100Hz刷新，规划/避障输入仍按原有频率，未凭空提高传感器信息量。


## 2026-10-08 升降100Hz轻微超时优化
升降独立循环允许最多2ms轻微迟到后立即继续、随后恢复绝对周期；大于2ms仍跳过积压。使能后重设定时基准，四项SDO反馈及目标刷新仍每周期执行，原停车/心跳门限不变。CAN/升降反馈/定时日志采用64KiB缓冲、活跃时约100ms刷新、正常退出关闭刷新；强杀或断电可能损失末尾缓冲。底盘定时不改。95项测试通过，旧耗时回放约98.89→100Hz，非实机结果。按原start.sh重启，再check_rates.sh --can-seconds 10确认。报告logs/lift_timing_optimization/REPORT.md，备份backups/lift_timing_20261008_154521。
