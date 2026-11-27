# 整理变更边界

`imported_files.json` 保存导入前来源和哈希；交付根目录 `SOURCE_MANIFEST.sha256` 保存整理后的版本。没有将本地目录伪造成已知上游 Git commit。

本次在独立仓库中修改：

1. `mapping_trial`、定位和移位入口从本仓库导入 calibration/control 并引用本仓库驱动安装路径。
2. 标定 settings 的 navigation_root 改为相对 sensors，preflight 解析为绝对路径；统一 env/build 入口。
3. CAD mesh URI 改为 package URI，准备工具在忽略目录建立 ament 包索引；导航 BT 路径由 launch 运行时覆盖。
4. control 构建不再同步仓库外的重复控制源；真实 driver、SDK、SAM 在本仓库重新构建。
5. 旧部署包装器改为只传源码的新目录工具，不自动替换系统服务；完整旧 deploy 入口仅作参考。
6. 新增文档、数据导入/清单/源码归档、自检和无硬件 CI；浏览器夹具显式收录为合成数据。

没有修改本次导入的 FAST-LIO-SAM C++ 算法、标定几何优化算法、低速阈值、时间门禁或电机协议。回环算法变更来自此前候选，原补丁在 `reports/backend_changes.patch`。原已调通目录未变。

交付文本统一为 LF 换行以匹配 Git attributes，导入前哈希仍保留在来源清单中。
