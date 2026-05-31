# EE871-E2 Hardening Prompt 01 — Branch Setup, Core Contracts, and Public API Safety

You are working in the EE871-E2 repository after the IDF-merged industry-readiness audit. Use the audit report as the baseline, but verify the code yourself before changing anything.

Important context:
- EE871-E2 is **not a normal hardware I2C driver**. It uses GPIO-style E2 bus signaling with injected SCL/SDA callbacks and injected timing callbacks.
- Do not blindly copy ADS1115 patterns.
- Preserve the framework-neutral core.
- Do not introduce Arduino, ESP-IDF, FreeRTOS, logging, heap-heavy framework types, or global bus ownership into `include/` or `src/`.
- Examples may use Arduino or ESP-IDF, but the core must stay platform-neutral.
- The goal is production-grade structure, deterministic behavior, precise diagnostics, and honest validation claims.

This is the first hardening prompt. Start the production-readiness branch.

## Branch setup

Start from a clean worktree.

```bash
git status --short
git branch --show-current
```

If there are uncommitted user changes, stop and report them.

Then create the hardening branch from `main`:

```bash
git checkout main
git pull --ff-only
git checkout -b hardening/ee871-e2-industry-readiness
```

If the branch already exists, switch to it only if it is clearly the intended branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git status --short
```

## Subagents to spawn

Spawn these subagents and have them return short, factual reports:

1. `core-contracts-agent`
   - Review `include/EE871/` and `src/`.
   - Confirm the core remains framework-neutral.
   - Check copy/move behavior.
   - Check public thread-safety and ISR-safety documentation.
   - Check whether callbacks can re-enter the driver.

2. `docs-contract-agent`
   - Review README, public headers, Doxygen comments, and existing docs.
   - Identify exactly where the public contracts should be documented.

3. `integration-review-agent`
   - Review the final diff before commit.
   - Confirm no broad refactor and no framework leakage into core.

## Implement only this scope

### 1. Update `AGENTS.md`

Create or update `AGENTS.md` with EE871-specific rules:

- EE871-E2 is a GPIO-style E2 bus driver, not hardware I2C.
- Core must remain framework-neutral.
- E2 line control, delays, and timebase must remain injected.
- Core must not own GPIOs, pins, Arduino `Wire`, ESP-IDF handles, FreeRTOS tasks, logging, or global bus objects.
- Public APIs that touch the E2 bus are blocking and not ISR-safe unless explicitly proven otherwise.
- Instances are not thread-safe; callers must externally serialize access.
- Transport callbacks must not recursively call into the same driver instance.
- Hardware validation and ESP-IDF build claims must not be invented.

### 2. Delete copy/move operations

In the public `EE871` class, explicitly delete copy and move operations unless a strong reason exists not to.

Expected pattern:

```cpp
EE871(const EE871&) = delete;
EE871& operator=(const EE871&) = delete;
EE871(EE871&&) = delete;
EE871& operator=(EE871&&) = delete;
```

Do not break normal construction.

Add a native compile-time test:

```cpp
static_assert(!std::is_copy_constructible_v<EE871::EE871>);
static_assert(!std::is_copy_assignable_v<EE871::EE871>);
static_assert(!std::is_move_constructible_v<EE871::EE871>);
static_assert(!std::is_move_assignable_v<EE871::EE871>);
```

Adjust namespace/class spelling to the actual code.

### 3. Document thread/ISR/reentrancy contract

Add explicit public documentation in README and public headers:

- `EE871` instances are not thread-safe.
- Use one owner task or external mutex/serialization.
- Public APIs are not ISR-safe because they can perform E2 bus I/O and delays.
- Do not call EE871 public methods from transport callbacks.
- Shared GPIO/E2 bus users must serialize externally.
- Line callbacks must be bounded and deterministic.

### 4. Document that EE871-E2 is not hardware I2C

Make this clear in README and relevant docs:

- The protocol is E2 GPIO-style signaling.
- `deviceAddress` is an E2 protocol address, not an ESP-IDF I2C device address.
- ESP-IDF examples use GPIO callbacks, not `driver/i2c_master`.

Do not over-edit marketing text. Keep it short and technical.

### 5. Create/update final hardening report draft

Create:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

For now include:

- Branch name.
- Prompt 01 scope.
- Files changed.
- Tests run.
- Remaining prompts/work planned.
- No hardware claims unless actual hardware was tested.

## Required validation

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

If a command is unavailable, record the exact error.

## Commit and sync

Commit with a focused message:

```bash
git add AGENTS.md README.md include src test docs
git commit -m "docs: tighten EE871 core contracts"
git push -u origin hardening/ee871-e2-industry-readiness
```

Only include files actually changed.
