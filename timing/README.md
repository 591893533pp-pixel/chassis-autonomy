# 时间同步诊断

`collect_ptp_trace.sh` 采集 PTP 和雷达控制报文，可附加点云/IMU 包头；抓包需管理员权限。分析工具分别解析 PTP 交换、Livox 设备诊断和真实数据包头。

当前运行中的时间门禁在 `mapping_trial/raw_time_monitor.py`、`live_gate.py`、`timing_policy.py`；避免把研究诊断脚本当成运动许可。

诊断与修改建议见 [研究报告](../research/ptp/诊断结论与修改建议.md)。主钟 MASTER、历史 HMS 0x04050002、诊断自报 offset 和真实点时间是不同证据。当前尚未确认根因，没有通过整理代码宣称消除告警。
