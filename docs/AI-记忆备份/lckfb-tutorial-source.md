---
name: lckfb-tutorial-source
description: The user follows 立创 (LCKFB) wiki tutorials for the 天猛星 MSPM0G3507 board — wiki.lckfb.com cannot be fetched from this environment
metadata: 
  node_type: memory
  type: reference
  originSessionId: 6ddafb15-05fc-4d21-88dd-2d9610cd35c2
  modified: 2026-09-11T14:54:56.806Z
---

The user is working through the LCKFB (立创) tutorial series for the 天猛星 MSPM0G3507 board, at `https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/ccs-beginner/<topic>.html` (e.g. `.../dma.html` for the ADC+DMA lesson). Each project in the workspace — `led_flash`, `gpio_toggle_output`, `TIMER`, `UART`, `ADC`, `DMA`, `pwm`, `systick_delay`, `Button_interrupt`, `motor` — corresponds to one lesson in that series.

**Why:** When the user says "教程" or reports that something "没达到想要的效果", they are comparing against that page. Knowing the source tells you their intended pin assignments and expected output, which is what the mismatch is usually against.

**How to apply:** `WebFetch` on `wiki.lckfb.com` fails with "Unable to verify if domain is safe to fetch" — this environment blocks it, so do not burn a call retrying. Ask the user to paste the relevant text or a screenshot instead. Note the tutorials often pick different pins than the TI LaunchPad docs, and the user's board is not a LaunchPad — see [[board-is-lckfb-tianmengxing]]. On 2026-09-11 a tutorial said to feed PA27 while the project's ADC was configured for PA15 (`ADC1` channel 0), which produced plausible-looking but wrong readings.
