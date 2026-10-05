# AI 助手记忆备份

这里是 **Claude Code 跨会话记忆**的一份副本。

## 为什么放这儿

AI 助手的记忆存在**工作区之外的固定路径**：

```
C:\Users\jeveux\.claude\projects\<工作区路径编码>\memory\
```

那个路径**不会被打进工作区的压缩包**。所以在同一台机器、同一个工作区路径下没问题，
但换机器、重装、或者换工作区路径，记忆就丢了。

放一份在这里，压缩包自带。

## 怎么用

**① 正常情况：什么都不用做**

只要还在 `C:\Users\jeveux\workspace_ccstheia` 这个工作区里开新会话，AI 会自动加载
原来那份记忆，这里只是备份。

**② 换了机器 / 换了工作区路径：把文件拷回去**

新工作区对应的记忆目录是：

```
C:\Users\jeveux\.claude\projects\<新工作区路径编码>\memory\
```

路径编码规则：**盘符小写**，其余保留大小写，`\` 和 `:` 都换成 `-`。
例：
- `C:\Users\jeveux\workspace_ccstheia` → `c--Users-jeveux-workspace-ccstheia`
- `D:\my_projects\26zhaoxin` → `d--my-projects-26zhaoxin`

把这个文件夹里的 `*.md` 全部拷进去即可。

## 内容

| 文件 | 类型 | 内容 |
|---|---|---|
| `MEMORY.md` | 索引 | 每次会话都会加载进上下文 |
| `project-e-ti-aiming.md` | 项目 | **最重要**：题目、三块板分工、四路串口、进度与待办 |
| `mspm0-sysconfig-nvic-gotcha.md` | 技术 | SysConfig 不生成 NVIC 使能（踩了两次） |
| `qd4310-uart-one-shot-protocol.md` | 技术 | 云台一发一收 + 等待窗口要等于发送周期 |
| `nuedc-site-requires-referer.md` | 技术 | 电赛官网抓图要带 Referer |
| `board-is-lckfb-tianmengxing.md` | 硬件 | 天猛星不是 LaunchPad，引脚别套用 LP 的 |
| `sysconfig-version-alignment.md` | 工具 | 项目 pin 1.26.2，MCP 跑 1.28.1 |
| `lckfb-tutorial-source.md` | 参考 | 用户看 wiki.lckfb.com 学天猛星 |
| `verify-before-generalizing.md` | 习惯 | 别一次就下结论，测 3 次以上 |
| `vision-capability.md` | 工具 | 可以直接读图 |

**更详细的接口说明看** [`../硬件与接口.md`](../硬件与接口.md)。
**架构/参数/待办看** [`../PROJECT.md`](../PROJECT.md)。
**已知问题看** [`../BUGS.md`](../BUGS.md)。
