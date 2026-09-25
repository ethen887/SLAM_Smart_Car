# SLAM_Smart_Car
基于SLAM算法的自主导航小车

使用 C 语言开发瑞萨与 STM32 板卡固件，由两位开发者协作完成比赛小车。

## 开发入口

- [协作规则与本地检查](CONTRIBUTING.md)：Issue → 短期分支 → PR → 另一人审核 → Squash 合并 main。
- [GitHub 初始化与分支保护](.github/SETUP.md)：仓库管理员首次配置清单。
- 新任务通过 GitHub Issues 的 Bug、Feature 或 Task 表单创建。

当前处于工程初始化阶段，尚未接入具体板卡代码与编译工具链。CI 已配置 PR 规范、自研 C 代码格式及静态检查；真实 MCU 编译、烧录和实车验证按协作规则逐步接入，不能用 CI 绿灯替代板测。
