---
name: metal4-only
description: Use this skill when working in IsaacSwift. This project must be designed and implemented with Metal 4 only, and Metal 3 based approaches, fallbacks, examples, and compatibility-first decisions are not allowed.
---

# Metal 4 Only

This repository is `Metal 4 only`.

## Core rule

- Always design, implement, and explain solutions with `Metal 4`.
- Do not propose `Metal 3` fallbacks.
- Do not reuse `Metal 3` sample structure unless it is explicitly rewritten for `Metal 4`.

## When implementing

- Prefer `Metal 4` APIs, patterns, and architecture decisions first.
- If an existing file reflects older Metal template code, treat it as migration material, not as the target architecture.
- If a choice is unclear, resolve it in favor of a stricter `Metal 4` design.

## When writing docs

- State clearly that the app is built for `Metal 4`.
- State clearly that `Metal 3` is out of scope.
