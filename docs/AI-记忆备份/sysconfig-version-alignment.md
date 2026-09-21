---
name: sysconfig-version-alignment
description: Projects here pin SysConfig 1.26.2 but the ccs-sysconfig MCP runs 1.28.1 — bump the project to 1.28.1; this also fixes the DMA-channel GUI crash
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6ddafb15-05fc-4d21-88dd-2d9610cd35c2
  modified: 2026-09-11T14:54:54.625Z
---

The ccs-sysconfig MCP runs SysConfig **1.28.1** (`D:/CCS/ccs/utils/sysconfig_1.28.1`, bundled with CCS). MSPM0 SDK 2.11.00.07 projects in this workspace are pinned to **1.26.2** (`D:/CCS/sysconfig_1.26.2`). Only those two are installed. When the versions differ, `openFile` refuses with "requires SysConfig version X, but AI assistance is using Y".

**Why:** Without alignment the SysConfig MCP cannot touch the project at all, and `.syscfg` files must never be hand-edited. On 2026-09-11 the user explicitly chose to bump the project up to the MCP's 1.28.1 rather than downgrade the MCP.

**How to apply:** Run `changeSysConfigVersion` (projectName + `getActiveSysConfigMCPVersion`) to move the project to 1.28.1. Note the tool returning Success only updates `.cproject` — the `.syscfg` file's `@versions {"tool":"..."}` header is what `openFile` checks, and it is rewritten by the *next* SysConfig build/save. If `openFile` still reports the old version right after, or says "No registered session for file", just retry it; the session registers on a subsequent call rather than the first.

**The same 1.26.2 device data also breaks the SysConfig GUI** — the user hit `TypeError: Cannot read properties of undefined (reading 'attributes')` at `Common.js:3673` whenever they opened a DMA channel's options. Root cause: `DMAChannel.syscfg.js:412` calls `Common.getAttribute(system.deviceData.peripherals["DMA"], ...)`, but 1.26.2's `MSPM0G350X.json` has **no peripheral named `DMA`** — only `DMA_CH0`..`DMA_CH6` (109 peripherals). 1.28.1's device data adds a plain `DMA` entry (110 peripherals), so `getAttribute` stops receiving `undefined`. Bumping the project to 1.28.1 fixes it, but the already-open SysConfig editor tab must be **closed and reopened** to pick up the new device data. Verified 2026-09-11 by diffing both device-data JSONs.

**How to apply (part 2):** Any project here that will configure DMA needs 1.28.1. `ADC/` was still on 1.26.2 as of 2026-09-11 and will hit the identical crash the moment a DMA channel is added. See [[board-is-lckfb-tianmengxing]] and [[lckfb-tutorial-source]].
