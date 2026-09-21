---
name: board-is-lckfb-tianmengxing
description: "The dev board is an LCKFB 天猛星 (MSPM0G3507), NOT a TI LP-MSPM0G3507 LaunchPad — the LaunchPad board docs' pinout does not apply"
metadata: 
  node_type: memory
  type: project
  originSessionId: bbde3e85-fd34-4f8b-89df-232aad140bf1
  modified: 2026-09-11T04:23:16.945Z
---

The physical board is a **立创 (LCKFB) 天猛星 MSPM0G3507** development board, not a TI LaunchPad. Photo of the silkscreen shows "LCKFB 立创开发板" and it carries the MSPM0G3507 in LQFP-64(PM).

**Why:** `D:/CCS/ccs/theia/resources/ai/boards/LP-MSPM0G3507/AGENTS.md` documents a completely different header layout and onboard peripherals (LED1=PA0, LED2 RGB=PB22/PB26/PB27, S1=PA18, backchannel UART PA10/PA11, light sensor, thermistor). None of that is on the LCKFB board. Applying it would produce wrong pin assignments that build fine and fail on hardware.

Hard evidence it isn't a LaunchPad: the LP board routes PA16 to BoosterPack pin 29 through jumper J15, PA15 to pin 30 as DAC_OUT, and does not break PA14 out to the header at all — yet on this board PA14/PA15/PA16 are all freely available on headers and were wired directly to a TB6612FNG.

**How to apply:** Configure only the MSPM0G3507 *device* characteristics from the SDK docs (peripheral capabilities, pinmux options, clock tree). Do not copy LaunchPad pin assignments from the board AGENTS.md. Take pin choices from the user or from SysConfig's pinmux solver instead. See [[sysconfig-version-alignment]] for the toolchain side.
