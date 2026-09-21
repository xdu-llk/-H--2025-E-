---
name: vision-capability
description: "deepseek-flash reads images natively, so no vision bridge or skill is needed — read pasted images directly"
metadata: 
  node_type: memory
  type: project
  originSessionId: 47330f12-8c99-4163-8740-9d013edbb399
  modified: 2026-09-11T02:53:18.840Z
---

The session model `deepseek-flash` (served from `api.deepseek.com/anthropic`) is
**multimodal**. Verified 2026-09-11 by posting a known test image straight to
that endpoint with an image content block — it came back described correctly,
down to the aliasing on the drawn circle's edge.

**Why:** On 2026-09-10 a vision-bridge skill (`ask-vision`) and the `modlens`
skill were built and installed on the assumption this model was text-only. That
assumption was wrong — it came from model names and project READMEs, not from
testing. The detour cost real time (one modlens read took 59.6 s on an image
that was already visible), and both skills were deleted on 2026-09-11.

**How to apply:** Read pasted images directly; do not rebuild vision tooling for
this harness. Note that a *pasted* image arrives as a normal base64 block, and
that the Windows clipboard is a separate thing — pasting into the chat does not
put the image on the clipboard, so clipboard-based workarounds are a dead end.

When unsure whether a model can see, post a known test image to its endpoint and
check the answer. That is cheaper and more reliable than reasoning from model
names or documentation, both of which said text-only here and were wrong.
