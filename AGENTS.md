# AGENTS

This repository is `Metal 4` only. `Metal 3` based designs, compatibility-first fallbacks, and downgrade paths are out of scope.

## Core Rules

- Design, implement, and explain the project with `Metal 4` as the baseline.
- Do not choose `Metal 3` APIs, compatibility-first fallbacks, or older sample structures as the target architecture.
- Treat `Metal 4` as unavailable on the simulator.
- `My Mac (Designed for iPad)` is not a valid renderer verification target.

## Build / Test

- Do not build for `iphonesimulator`.
- Build for `iphoneos` and real-device destinations.
- Use `./scripts/build-device.sh` for command-line device builds.
- `./scripts/build-device.sh` must verify the `Metal Toolchain` before building.
- If you see `cannot execute tool 'metal' due to missing Metal Toolchain`, the likely cause is the current permission mode, not a missing installation. Retry in normal or escalated mode as needed.
- Run Swift unit tests with `./scripts/test-unit.sh` and UI tests with `./scripts/test-ui.sh`.
- `./scripts/test-smoke.sh` is a lightweight smoke path and belongs on the unit-test side.
- UI tests are a bonus launch smoke check, not the primary policy/physics
  verification path. Keep UI tests minimal; put walking, cadence, and
  renderer-independent policy coverage in unit/headless tests.
- `./scripts/test-ui.sh` targets a real iOS device when one is available.
  Override the destination with `ISAACSWIFT_UI_TEST_DESTINATION='id=<device-id>'`
  if needed.
- Quiet mode is the default. Use `--verbose` only when full logs are required.
- Agents should normally use `./scripts/agent-ci.sh` to verify build plus smoke coverage in one command.

## Output Rules

- Keep build and test output minimal.
- Run `./scripts/build-device.sh` and `./scripts/agent-ci.sh` without arguments unless you explicitly need verbose output.
- Use `--verbose` only when full logs are necessary.
- For ad hoc commands, summarize output with filters such as `2>&1 | grep -Ei 'error:|warning:|FAILED|SUCCEEDED|Test Suite|Executed'` to limit token usage.

## Skills

- `skills/metal4-only/SKILL.md`: Metal 4 only guidance.
- `skills/asset-pipeline/SKILL.md`: Regenerating and validating USDZ assets from Isaac Sim sources.
- `skills/policy-integration/SKILL.md`: Porting Isaac Lab policies into the Swift/Jolt runtime.

Keep topic-specific knowledge in the matching skill instead of adding more top-level Markdown files.
