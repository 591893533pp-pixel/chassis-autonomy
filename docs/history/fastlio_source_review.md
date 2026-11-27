# FAST-LIO-SAM 源码核对（2026-11-26）

核对对象是目标 NUC `/home/pc/rk3588 link/mapping/programs/FAST-LIO-SAM`，本目录 source 为本次复制的源码快照。已阅读 README、启动配置、laserMapping.cpp 的输入/估计/后端调用/输出、mapOptimization.cpp 的回环和因子图、preprocess.cpp 的 Livox 点云入口及结果目录实现。不是对全工程的完整缺陷审计；没有据此修改或开启运行后端。

## 当前实际链路

双雷达原始点云 → 已冻结相对外参融合 → 后右光学坐标适配 `/slam/points` → FAST-LIO（后右IMU）→ `/Odometry` 与 `/cloud_registered` → 录包与累计预览。

双雷达融合在该工程之外完成；FAST-LIO接收到的是一个融合点云话题，并非内部运行两套雷达SLAM。`preprocess/lidar_type=8` 对应本地 `livox_pcl2_handler`，使用逐点timestamp、line字段，转换相对时间并排序。前端以IMU传播和运动补偿、点到局部地图平面的残差、迭代误差状态卡尔曼滤波更新位姿，ikd-Tree维护在线局部地图。不是粒子滤波。

## 工程能力与本轮配置

| 功能 | 本地实现 | 145615运行 |
|---|---|---|
| FAST-LIO前端 | laserMapping.cpp、IMU处理、ikd-Tree | 开启 |
| SAM位姿图后端 | GTSAM/iSAM2、里程计/回环/可选场景和GNSS因子 | sam_enable=false |
| 回环 | 空间近邻候选＋时间间隔＋ICP验证 | loopClosureEnableFlag=false |
| 地面与楼层约束 | s-graph地面法线与楼层范围筛选 | 均关闭 |
| 已知地图定位 | prior/online+prior模式，初始ICP＋点到面细化 | common/mode=1，只用在线地图 |
| 粒子滤波/AMCL | 本工程未找到实现 | 未启动 |
| 关键帧与优化全图导出 | keyframe_export_en、keyframe_global_pcd_en | 均关闭 |

原工程 mapping_mid360.launch.py 设置sam_enable=True；本轮run_mapping.py直接启动fastlio_mapping并加载隔离lio.yaml，所以不能根据工程名或原launch推断本轮开启了SAM。

## 回环底层流程

1. 当前位姿相对上个关键帧的平移或转角达到阈值，保存关键帧。本轮配置中预留阈值为1m或0.2rad（约11.5°），但SAM关闭时不执行该流程。
2. loopClosureThread等待至少6个关键帧。根据优化后位置在XY平面做半径搜索；如开启楼层表示，再限定楼层范围。不是基于Scan Context的地点描述子检索。
3. 排除过近时间的帧、已用回环帧；构造当前帧和历史邻域子图。源码要求当前下采样点≥300、历史子图点≥1000。
4. ICP以已有位姿差为初值，检查hasConverged及fitness score。源码没有在该接受分支单独检查重叠比例、退化或位姿改变量；重复墙面场景需实际验证，不能将低score等同于回环绝对正确。score是PCL匹配代价，不能直接读成米级位置误差。
5. 将匹配变换作为BetweenFactor加入队列，由iSAM2优化关键帧历史位姿，correctPoses更新轨迹和map→odom修正。
6. 优化后的全图须按更新后的关键帧位姿重新拼接。此前逐帧积累的/cloud_registered历史点不会因为未来回环自动被重新摆正。

当前隔离配置保留的参数：搜索半径15m、最小历史间隔30s、ICP最大对应距离10m、fitness门限0.2、回环频率1Hz。这些在关闭状态下未作回环实测，不能作为本场地已验收参数。

## 定位与“粒子检测”

若指粒子滤波定位：AMCL通常在二维已有地图上维护多个位姿假设。本工程使用误差状态卡尔曼滤波，以及已有点云地图上的ICP/点到面配准。common/mode=2为prior，3为online+prior；支持/reloc/manual人工初值修正。初始ICP依赖位姿初猜，不应等同于任意位置自动全局找回。工程的SAM指平滑与建图，不是AMCL。

## 完整地图与保存注意

本轮全时段配准输出已从MCAP逐帧导出，共2955/2955帧、14,160,514点，XYZI二进制PCD 226,568,434字节；导出没有额外抽帧或降采样。但原运行dense_publish_en=false、前端0.15m体素和预处理抽点/近场裁剪已经生效，因此这是全部已发布配准帧，而不是每个原始雷达点。原始双雷达数据仍保存在NUC约5.8GiB的录包内。

累计估计轨迹长度10.919m，起终位置相差0.237m；未确认现场严格回到同一点同朝向，不能把0.237m称为闭环误差或定位精度。地图没有回环优化、没有动态物体清理，也不是已验收导航地图。

源码关键帧全图在关闭前以featCloudKeyFrames和最终cloudKeyPoses6D重新拼接，适合后续回环结果输出。实时/lio_sam/mapping/cloud_global还受显示范围和体素抽样限制，不等于完整存档。

发现一个保存路径不一致：include/common_utils.h:281附近创建ROOT_DIR/RESULTS/PCD，但laserMapping.cpp:1291及2247实际写ROOT_DIR/PCD。关键帧位姿真实文件名为keyframe_pose.txt，也与README中pose.txt描述不同。后续启用导出前应核对路径与返回状态；本次离线导出不依赖此入口。

## 推荐下一步

先用已保存的/slam/points和/rear_right/imu离线重放，开启SAM及回环候选，比较接受/拒绝日志、回环边、优化前后墙面重影和轨迹，再决定实车参数。重放应隔离ROS域及结果目录。不能仅将两个开关改true，就宣称回环通过或可以点到点自主导航。

## 主要源码位置

- src/preprocess.cpp:213，融合Livox PointCloud2的逐点时间入口。
- src/laserMapping.cpp:221，误差状态卡尔曼滤波器；约2050行起IMU处理、匹配与更新。
- src/laserMapping.cpp:1945，SAM初始化及线程门禁；2157，关键帧与后端调用。
- src/mapOptimization.cpp:288，回环候选与ICP；516，回环因子；820，关键帧与iSAM2更新；929，历史位姿及map→odom修正。
- src/laserMapping.cpp:1214，先验地图初始对齐；1248，发布配准帧及稠密开关。
- src/laserMapping.cpp:2253，关键帧位姿和全局PCD导出。
