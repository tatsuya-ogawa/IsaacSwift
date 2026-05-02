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

## Build and signing

- Verify builds with `./scripts/build-device.sh` or `./scripts/agent-ci.sh`; do not switch to `iphonesimulator` or `My Mac (Designed for iPad)` as a workaround.
- GitHub-visible project settings intentionally omit local signing configuration. If a device build fails with signing, provisioning, team, or account errors, ask the user to configure local Xcode signing/development-team settings or provide local signing overrides, then rerun the same device build command.
- Treat signing errors as environment setup issues, not renderer or Metal API failures.

## When writing docs

- State clearly that the app is built for `Metal 4`.
- State clearly that `Metal 3` is out of scope.
