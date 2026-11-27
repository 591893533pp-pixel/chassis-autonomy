# 来源与第三方说明

本仓库汇集已有研发代码和供应商/开源组件，未统一授予新的 MIT/Apache 等许可。已有文件头、LICENSE、作者和 package.xml 声明保留。下面的“未核实”是确切缺口，不表示能够任意重新授权。

| 组件 | 收录位置 | 当前证据 |
|---|---|---|
| FAST-LIO-SAM 及内部依赖 | `mapping_ws/src/fast_lio_sam/` | 原 README、package.xml 的 BSD 声明和源文件头保留；本地快照未带完整统一根 LICENSE，应继续核对上游及各内部依赖条款 |
| Livox ROS Driver2 | `sensors/driver_workspace/src/livox_ros_driver2/` | `LICENSE.txt` 与 RapidJSON `license.txt` |
| Livox SDK2 | `sensors/Livox-SDK2/` | `LICENSE.txt`；spdlog/FastCRC/RapidJSON 等子目录许可 |
| Ruckig | `control/program/third_party/ruckig/` | `LICENSE` 与 README |
| Elmue / RobStride 适配 | `control/program/third_party/elmue_can/` | 源码头和 README；未发现统一根 LICENSE，发布范围需核对来源授权 |
| 手柄 navslam_core 副本 | `control/program/remote/vendor/` | `PROVENANCE.json` 和代码注释；不得假定此文件与其他组件授权相同 |
| CAD / URDF | `model/cad_source/`，外部资产 cad 组 | 用户提供的 chassis_id_me 包；没有发现明确开源授权声明 |
| 新增标定、融合监督、导航门禁、文档 | 对应模块与 `scripts/`、`tools/` | 本项目研发成果；由项目所有者决定授权 |

`provenance/imported_files.json` 给出每个导入文件的本地来源和原始 SHA-256。若文件后来被整理修改，原始哈希保留用于追溯，并不要求等于现文件。`SOURCE_MANIFEST.sha256` 是交付版校验清单。

依赖环境不是 vendored 授权：GTSAM、PCL、Eigen、ROS2、Nav2、Python 科学计算库由环境安装，版本和来源按构建文档核对。没有把原机 Python runtime、编译器产物、系统二进制、密钥或全量 rootfs 再复制入 Git。
