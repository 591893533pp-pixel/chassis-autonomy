# 双 MID360S 接入实测

已验证 .132（BDLCN9S0039449）和 .159（ARMCP9M0033988）的网络、10 Hz 点云、200 Hz IMU 及 PTP 标志和设备诊断，详情 summary.json / observation.json。

NUC enp85s0 当前同时有 192.168.16.4/24 和临时 192.168.1.4/24。connect_lidar_network.sh 添加临时地址；persist_lidar_network.sh 保存到 NetworkManager（未执行，需 sudo）。独立采集入口 run_lidar_test.sh 运行120秒后退出，返回124为 timeout 预期返回；observe.py 订阅30秒保存统计。

正式导航配置未改。两台物理安装位置及外参仍需确认；不直接套用原设备标定，也未启动建图/导航或底盘运动。
