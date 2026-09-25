# GitHub 启用清单

仓库文件提供模板和检查；服务器端的合并限制必须在 GitHub Settings 中启用。本次未修改远端设置。

## 首次落地

1. 在 Settings → Collaborators 中邀请另一位开发者并授予写入权限。双方都应能请求和提交 Review。
2. 确认默认分支是 `main`，启用 Issues 和 Actions。
3. 将本次配置通过 `chore/setup-workflow` 或 `codex/setup-workflow` 分支提交，PR 标题用 `chore: 建立双人开发流程`，由另一人审核。先让 CI 成功运行一次，便于后续选择状态检查。
4. Settings → General → Pull Requests：启用 Squash merging，默认 squash 标题取 PR 标题；关闭另外两种合并方式；启用合并后删除分支。
5. Settings → Rules → Rulesets 建立启用状态的分支规则，目标为 `main`；也可使用 Branch protection 实现同等限制，不必重复配置。

## main 必需规则

- Require a pull request before merging。
- Required approvals：**1**。两人团队不设为 2，PR 作者不能批准自己的 PR。
- Dismiss stale pull request approvals when new commits are pushed。
- Require conversation resolution before merging。
- Require status checks to pass：选择本仓库 Actions 最近一次运行的 **`quality`** 作业（界面可能显示 `CI / quality`）；不要误选不存在的编译检查。
- Require branches to be up to date before merging。
- 禁止强制推送和删除 main；限制直接更新由 PR 要求保证。使用传统分支保护时启用管理员也遵守规则的选项，Ruleset 不配置日常 bypass 成员。

暂不启用 Require review from Code Owners：双方账号和模块职责未完全提供，错误的单人 CODEOWNERS 会造成审核瓶颈。当前每个 PR 手动请求另一人；日后双方都列为共同 owner，再启用自动请求审核。

分支保护可用性取决于仓库可见性及 GitHub 套餐；若界面提示不支持，文档和 CI 仍可使用，但无法宣称服务器已强制禁止违规合并。参见 [GitHub 分支保护说明](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/about-protected-branches)。

## Actions 与机器人

工作流使用 `pull_request`，令牌只读，不读取 Secrets，不自动烧录，不自动批准或合并 PR。第三方 Action 固定到完整 commit，Dependabot 每月提出更新 PR。不要改成带写权限执行 PR 代码的 `pull_request_target`。参见 [GitHub Actions 安全说明](https://docs.github.com/en/actions/reference/security/secure-use)。

初次运行只做已有检查；没有固件代码时 C 检查会显示跳过。新增板级编译 job 后，待它首次成功，再把该 job 加入 Required status checks。不要对必需工作流加 paths 过滤而使检查一直 Pending。

## 设置后的验收

开一个测试分支和 PR：不规范标题应检查失败，修改标题后应重跑并通过。确认缺少另一人的 Approve 时不能合并，新增代码提交后旧批准失效，未通过 `quality` 时不能合并。验证完关闭测试 PR 并删除测试分支。

Issue 表单从默认分支加载，所以配置合并后再检查 New issue 中的 Bug / Feature / Task 三个入口。PR 模板检查硬件证据的填写情况由审核人负责，机器人不推断实车是否通过。
