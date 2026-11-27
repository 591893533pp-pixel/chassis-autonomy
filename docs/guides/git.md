# Git 项目与协作

此目录是独立 Git 仓库。外层 `底盘调试/` 还有一个历史 Git 目录，不要把两层一起 add。请始终先 `cd /home/pc/chassis-autonomy`，`git rev-parse --show-toplevel` 确认目标。

本次整理不替你创建远端、不推送、不替你决定公开授权。上传前先看 `THIRD_PARTY_NOTICES.md` 和资产说明。

## 第一次提交和上传

```bash
cd /home/pc/chassis-autonomy
python3 tools/check_repository.py
python3 tools/test_offline.py
git status --short
git add .
git diff --cached --stat
git commit -m "Import chassis autonomy source, R&D docs and validation evidence"
# 替换成你自己的仓库地址：
git remote add origin <你的Git仓库SSH或HTTPS地址>
git push -u origin main
```

如 Git 未配置作者，用你自己的姓名和邮箱设置本仓库 `git config user.name` / `git config user.email`。不要把 SSH 私钥、密码写进仓库或 remote URL。当前配置中设备内网地址和序列号用于复现，公开发布前可另存为私有现场配置。

## 后续开发

用独立分支做标定、回环或导航修改；合并说明包含触发问题、行为变化、测试、现场限制。代码提交与外参数据版本配对，例：`v0.1-lab-baseline`、`calibration-20261125-D`，不要在未验收时标成生产稳定版。

基础 CI 是无 ROS/无硬件的几何、标定策略和门禁测试。真实 ROS 构建、bag 回放、实车实验需按指南单独跑，并把结果指向当前 commit 和配置哈希。

## 传到目标 NUC

提交完审核后的工作树，使用只传源码的入口：

```bash
python3 tools/stage_source.py --target pc@10.23.2.190 --destination /home/pc/chassis-autonomy-new
# 检查输出后，添加 --apply 实际传输；已有目录会被拒绝。
```

该工具用 Git HEAD 归档，不包括被忽略的大数据和未提交文件；拒绝脏工作树。不会启动服务或电机，也不会覆盖旧部署。部署后按 build 指南构建、导入所需资产和验证。

## 交付源码压缩包

`python3 tools/export_source.py` 会先检查源码，再输出 `release/chassis-autonomy-source-20261127.tar.gz`，包含当前 Git 可见文件和 `SOURCE_MANIFEST.sha256`，不含构建产物、Git 历史和大数据。它可用于首次新建仓库；已存在仓库不应直接覆盖导入。

解压后在源码根目录执行 `sha256sum -c SOURCE_MANIFEST.sha256` 校验。后续编辑代码时该清单不会自动更新，重新发布前再运行 export_source。
