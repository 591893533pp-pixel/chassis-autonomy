# 历史研发记录

历史记录保留当时的命令、路径、失败结论和证据。后续阶段可能已纠正早期判断，因此当前总状态以 [STATUS](../STATUS.md) 为准；新入口以 [实验手册](../guides/experiments.md) 为准。

| 阶段 | 记录 |
|---|---|
| 11/23–27 部署与调试时间线 | [deployment_log.md](deployment_log.md) |
| 手柄映射 | [gamepad_mapping.md](gamepad_mapping.md) |
| 静止融合 | [static_fusion.md](static_fusion.md) |
| 移动建图基线 | [manual_mapping_status.md](manual_mapping_status.md)、[操作记录](manual_mapping_manual.md) |
| FAST-LIO 源码核对 | [fastlio_source_review.md](fastlio_source_review.md) |
| 标定周报与图 | [周报](../weekly/20261125_双雷达标定/周报模块.md) |
| 建图周报与图 | [周报](../weekly/20261126_双雷达融合建图/周报模块.md) |
| PTP 问题分析 | [诊断结论](../../research/ptp/诊断结论与修改建议.md) |
| 相对外参稳定性 | [复核记录](../../research/calibration_validation/复核记录.md) |
| 车体外参和地面差异 | [测量步骤](../../research/body_anchor/核对记录与测量步骤.md) |

`research/` 保留分析脚本，有些仍引用原始日期目录和远端采集数据。它们是研究复现材料，不是当前标准运行入口。未收录的巨大数据按 assets 清单补齐。旧历史文档中的绝对链接若指向原机路径，在新电脑不可打开；保留这些定位信息是为追溯原始实验。
