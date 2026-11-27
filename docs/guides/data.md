# 数据、模型与可复现性

普通 Git 保存源码、配置、文档、报告和小图片。约 1 GB 已知本地大文件独立管理：CAD 原包、完整 PCD、PTP 抓包、初期雷达点云。每个文件的大小、SHA-256、原始位置与期望归档位置见 `assets/manifest.json`。

```bash
python3 tools/assets.py list
python3 tools/assets.py import --group all
python3 tools/assets.py verify --group all
```

本机 import 按清单原始位置复制到 `assets/local/`，先核对源哈希，不覆盖冲突文件。复制后将整个 `assets/local/` 另存到数据盘、对象存储或私有 release 资产。另一台机器：

```bash
python3 tools/assets.py import --source-root /data/chassis-assets --group all
```

该目录被 `.gitignore` 排除；**只上传 Git 不会自动上传大文件**。离线测试与轻量模型不依赖全部资产；完整 CAD 显示需要 cad 组；真实回放还需要目标 NUC 上的 bag。

## 还需从 NUC 补取

- `/home/pc/motion_mapping_trial_20261126/runs/20261126_145615`：原始 rosbag2 和运行文件，历史估计约 5.8 GiB；本次未连回核实，所以未编造哈希。
- `/home/pc/dual_lidar_calibration_20261125/sessions`：完整标定 A/B/C/D 与重复采集。

取回后逐文件计算哈希，给数据版本独立编号，把实测大小和对象存储位置补进 manifest。保留报告引用关系。bag 内包含场地数据，不要以为只分享代码时也必须公开全部地图。

## 想全部放在 Git 服务器上

可选择 Git LFS，先查服务器单文件/仓库存储配额，然后在独立的数据路径启用 LFS，再移除相应 ignore。不要直接 `git add -f assets/local` 后才考虑 LFS，否则大文件会先进入普通 Git 历史。可参考：

```bash
git lfs install
git lfs track 'assets/local/**'
# 审核并移除 .gitignore 的 assets/local/ 及相关数据扩展规则后：
git add .gitattributes .gitignore assets/local
```

本次没有自动启用 LFS、没有上传大文件。源包中的 `provenance/imported_files.json` 保留导入前哈希；`SOURCE_MANIFEST.sha256` 描述整理后的文件，两者用途不同。

## 本次交付本机资产

已将 manifest 中可获取的 68 个本地资产复制到 `assets/local/` 并逐一校验，总计 1,036,153,288 bytes。这个目录可作为前述跨机器导入的 `--source-root`。目标 NUC 上尚未取得的 bag 和完整标定原始采集仍在清单中明确标记。
