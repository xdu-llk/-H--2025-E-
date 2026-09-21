---
name: verify-before-generalizing
description: "Measure a flaky relay or endpoint several times, against a control, before declaring what it does or does not support"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 47330f12-8c99-4163-8740-9d013edbb399
  modified: 2026-09-11T03:05:30.693Z
---

On 2026-09-11 the user had to correct the same mistake twice: a single failed
probe turned into a sweeping claim.

1. "This model has no vision" — inferred from the model name and a project
   README, never tested. One API call disproved it.
2. "xhyapi's `/v1/responses` returns 502" — concluded from one probe that hit a
   transient gateway error. Retested, it returned 200 three times running, and
   the user's Codex config using exactly that route had always worked. The
   advice built on top of that false premise ("change `wire_api` to `chat`")
   was wrong too.

**Why:** Both relays in this setup fail *intermittently* — by hanging, by
returning a gateway error, or by answering in 2 s where the same call took 60 s
before. One sample therefore says almost nothing. A confident wrong claim is
worse than no claim, because it sends the user off to change a configuration
that was never broken.

**How to apply:** Before stating that an endpoint, model, or capability does not
work, measure at least three times and include a control (a known-good request
on the same path). Prefer "I saw X once, let me confirm" over a verdict. And when
the user says something works, treat that as evidence to re-test against — not
as a claim to argue down.
