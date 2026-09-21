---
name: nuedc-site-requires-referer
description: 电赛官网 res.nuedc-training.com.cn 的赛题正文是图片，且下载图片必须带 Referer 头，否则 403
metadata: 
  node_type: memory
  type: reference
  originSessionId: 706d5937-1cdb-411d-88d4-39e050b07a43
  modified: 2026-09-21T13:37:56.244Z
---

**电赛培训网 `res.nuedc-training.com.cn`**（全国大学生电子设计竞赛官方）的赛题页面：

- **正文不在 HTML 里**，是一组 PNG（`/s/png/{年}/{月}/{日}/topic_*.png`）
- **页面本身不需要登录**，`curl` 直接抓 HTML 就能拿到全部图片 URL
- ⚠️ **下载图片必须带 Referer**，否则 CDN 返回 **304 字节**的
  `403 Forbidden — denied by Referer ACL`（Tengine）。这个错误页很小，容易被
  误认为"图就是很小"，实际是全部失败

```bash
curl -s -e "https://res.nuedc-training.com.cn/topic/2025/topic_124.html" \
     -A "Mozilla/5.0 ..." -o out.png "<图片URL>"
```

抓下来的 2025 E 题图存在 `my_ti_control/docs/e2025_*.png`。

完整的站点经验写在
`~/.claude/skills/web-access/references/site-patterns/res.nuedc-training.com.cn.md`。

**How to apply**：再抓电赛的题直接用上面那条带 Referer 的命令。
相关：[[project-e-ti-aiming]]
