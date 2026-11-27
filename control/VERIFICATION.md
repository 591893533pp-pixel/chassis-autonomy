# 2026-09-30 集成验证

完成：独立副本编译成功，C++ CTest 6/6通过；Python 32/32通过（13策略/入口、4真实工作线程假CAN测试、15继承PV测试）。假CAN覆盖使能、50mm/s计数量化、松键零速、反馈停车确认、心跳失效清理、已使能设备拒绝接管与轮动互锁。测试没有打开CAN或发送真实运动命令。

所有入口Bash语法/Python AST通过；从/tmp启动带空格路径的--dry-run通过，51mm/s请求被拒绝。日志见logs/offline_pytest.log、offline_ctest.log、offline_static_checks.json。

未完成：实机组合遥控、50mm/s机械速度和停车距离、上下端点/负载验证。默认10mm/s延续原已测试PV量级，但新组合入口仍需现场确认。50mm/s为请求及反馈保护上限，不是机械绝不超调的证明。建图副本只启用底盘。


## 2026-09-30 遥控断续修正

启动指令仍为 cd '/home/pc/rk3588 link/remote control' && ./start.sh。旧进程应先正常退出，再启动新版。修正单点零速互锁在反馈微动时反复切断：200ms窗口、150ms证据，车轮0.1/0.15rad/s进入/退出迟滞；保留明显运动、断流、故障与原停车保护。速度/PI未变。

28项离线测试和原日志回放通过，实机待复测。先1档分别测试底盘持续按住/松开，再行程中部升降持续按住/松开。完整参数、证据与命令见 logs/stationarity_fix_20260930/REPORT.md；旧文件已备份。


## 2026-09-30 同时控制更新（现行行为）

默认入口允许独立遥控同时平移/旋转与升降：

~~~bash
cd '/home/pc/rk3588 link/remote control'
./start.sh
~~~

旧遥控需要先松开所有输入、Ctrl+C正常退出并等待清理；然后再启动，代码不会热更新。START首次使能时先保持中立，待升降使能完成后再操作。保持摇杆/方向键的同时按住X/Y即可升降。任一控制器故障仍结束会话并尝试停止两轴；首次使能、反馈/心跳、0–250mm、默认10mm/s、50mm/s上限及松键停车规则保留。

恢复旧互锁模式：./start.sh --exclusive-motion。底盘/建图模式：./start.sh --chassis-only（不创建升降工作线程）。本轮只更新本目录，未更新mapping内部副本、地图、定位或PTP，也未启动实机运动。软件允许同时操作不代表负载/重心变化下的稳定性已完成验收，首次用1档与默认10mm/s短距离确认。

43项离线测试通过，包括混合输入保持、分别松开、首次中立使能、假CAN升降在线轮动时连续控制、心跳过期停止及旧互锁/仅底盘模式。详见logs/simultaneous_control_20260930/REPORT.md。先前互锁说明为历史版本，现行默认以上述同时控制为准。


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
