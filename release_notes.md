# EE871-E2 v0.2.1 Release Notes
Date: 2026-02-28

## Highlights
- Unified around the common examples/01_basic_bringup_cli structure and command baseline.
- Standardized CLI help and runtime output style across I2C libraries with functional color usage for actionable states.
- Improved health and lifecycle consistency (begin, tick, end, probe, recover) with clearer state/counter diagnostics.
- Expanded safe stress, stress_mix, and selftest workflows and reporting.
- Added portability-oriented timing abstraction patterns to keep PlatformIO + Arduino stable while easing future ESP-IDF migration.
- Refreshed docs including unification and porting guidance.
- Improved E2 bus diagnostics, CLI error clarity, and safe selftest behavior for not-initialized or absent-device cases.

## Compatibility and Migration
- This release prioritizes API and CLI consistency across libraries; limited compatibility shims may remain where practical.
- In-repo consumers were updated toward the canonical interfaces and naming style.

## Tag
- v0.2.1

## Suggested GitHub Release Title
- EE871-E2 v0.2.1
