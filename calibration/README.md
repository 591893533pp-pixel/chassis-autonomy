# 双雷达标定工具

完整中文操作手册：[操作手册.md](操作手册.md)。先读第1–3节：相对标定、车体外参、墙角摆位。

目标NUC安装位置：`/home/pc/dual_lidar_calibration_20261125`。

## 现场命令速查

以下在NUC执行。每条采集命令结束后才移动车体；A、B、C必须是三个实际不同的位置/朝向，`--pose-note`按实际填写。

```bash
cd /home/pc/dual_lidar_calibration_20261125
./calibrate.sh check
./calibrate.sh capture corner_20261125_01 A --pose-note 'A：距两墙约2米，静止'
# 等采集退出，移位并退出遥控
./calibrate.sh capture corner_20261125_01 B --pose-note 'B：相对A平移约0.6米、转向30度，静止'
# 等采集退出，再次移位并退出遥控
./calibrate.sh capture corner_20261125_01 C --pose-note 'C：再换一个位置与朝向，静止'
./calibrate.sh fit corner_20261125_01 --train A B --validation C
```

每次重新标定换一个会话名。结果在 `sessions/会话名/result/`。`REJECTED`不得采用；自动检查通过后仍要查看独立验证图。输出默认不写入运行配置；绝对车体外参不在本工具自动验收范围。

工具不驱动车体。采集约8秒预热＋30秒数据，退出清理最多额外10秒；独立驱动另有最长时限，异常断开不会无限扫描。预检不启动雷达。

算法核心来自现有部署包的 `mapping/programs/FAST-LIO-SAM/tools/fit_extrinsics.py`，新增了设备与采集检查、静止检测、多场景一致性、独立验证、候选导出和脚本生命周期管理。

仅有历史同步提示0x04050002时，新版区分“严格健康”与“静止几何资格”。原始告警保留，其他故障/实时PTP条件不放宽。旧数据可执行 `./calibrate.sh review-static 会话名 标签`，另存复核报告；本次实际会话使用 `--train A2 B --validation C`。这不是运动导航同步验收。

第二轮冻结候选已通过新D静止相对外参检查，见操作手册第10节及experimental/common_planes_candidate_v1/holdout_D/report.json。原ICP结果仍拒绝，运行配置尚未替换。
