# EE871-E2 Industry-Readiness Hardening Prompt Bundle

Send these prompts one by one to the coding agent. Do not send the next prompt until the previous one is committed, pushed/synced, and its report section is updated.

Files in this bundle:

1. `ee871_prompt_01_core_contracts.md`
2. `ee871_prompt_02_e2_fault_harness.md`
3. `ee871_prompt_03_persistent_dirty_state.md`
4. `ee871_prompt_04_idf_ci_docs.md`
5. `ee871_prompt_05_final_validation_report.md`

---


---

## ee871_prompt_01_core_contracts.md

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


---

## ee871_prompt_02_e2_fault_harness.md

# EE871-E2 Hardening Prompt 02 — Native Fake E2 Fault Harness and Runtime Fault Tests

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty, stop and report it.

This prompt implements the largest missing test foundation: a deterministic native fake E2 transport.

## Important context

EE871-E2 is a GPIO-style E2 bus driver, not hardware I2C. The fake should model E2 line behavior at the driver callback boundary, not ESP-IDF I2C behavior.

Preserve framework neutrality. Do not add Arduino, ESP-IDF, FreeRTOS, logging, or global buses to `include/` or `src/`.

## Subagents to spawn

1. `e2-protocol-test-agent`
   - Inspect current E2 read/write implementation.
   - Identify callback sequence expectations.
   - Propose the smallest fake transport that can test runtime failures.

2. `health-fault-agent`
   - Inspect health/offline/recover behavior.
   - Define expected status and health transitions for protocol faults.

3. `test-integration-agent`
   - Ensure tests remain native and framework-neutral.
   - Ensure no test depends on Arduino or ESP-IDF.

## Implement only this scope

### 1. Build a native fake E2 transport

Add a test helper under `test/` or `test/support/`, for example:

```text
test/support/FakeE2Transport.h
```

The fake should be deterministic and simple. It does not need to simulate analog timing perfectly, but it must be able to script:

- normal line idle state;
- line reads for SCL/SDA;
- controlled ACK/NACK behavior if applicable to the current abstraction;
- SCL held low until `bitTimeoutUs`;
- SDA stuck low/high where relevant;
- PEC mismatch on read response;
- read response bytes for status/CO2/custom registers;
- write accepted but readback verify mismatch;
- device absent / no response;
- unplug/replug style transitions.

Do not over-engineer. Prefer clear scripts and helper methods over a complex bus emulator.

### 2. Add runtime fault tests

Add native tests for at least these cases:

#### Clock stretch / stuck SCL timeout

- Configure fake SCL held low.
- Call a public read or probe path that must wait for SCL.
- Expect the correct timeout/status.
- Confirm the operation is bounded.

#### PEC mismatch

- Provide a valid-looking response with wrong PEC.
- Expect `PEC_MISMATCH` or the actual project status code for PEC failure.
- Confirm no false success.
- Confirm health failure counters update only on tracked public operations, not raw diagnostic operations if that is the contract.

#### Device absent / line no-response

- Simulate no valid device response.
- `probe()` should fail without health side effects if that is the documented contract.
- Tracked reads should affect health.

#### Write verify mismatch

- Simulate `customWrite()` write accepted but readback returns old/different value.
- Expect a precise error.
- Confirm the caller sees failure.

#### Offline threshold and recover

- Cause repeated tracked failures until OFFLINE or equivalent degraded/offline state.
- Confirm normal tracked operations stop or behave as documented while offline.
- Simulate device recovery and call `recover()`.
- Confirm health returns to READY/online only after successful recovery.

#### Probe no-health-side-effect

- Confirm `probe()` does not mutate health counters/state, even when failing.

### 3. Keep API behavior stable

Do not redesign the driver in this prompt unless a test exposes a clear bug. If a real bug is found, fix it minimally and document the behavior.

### 4. Update final report

Append a Prompt 02 section to:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

Include:

- Fake transport design summary.
- Runtime fault tests added.
- Any behavior changes.
- Exact test results.

## Required validation

Run:

```bash
python tools/check_core_timing_guard.py
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

Also run any project-specific test command if present.

## Commit and sync

Commit with a focused message:

```bash
git add test src include docs
git commit -m "test: add EE871 E2 fault injection coverage"
git push
```

Only include files actually changed.


---

## ee871_prompt_03_persistent_dirty_state.md

# EE871-E2 Hardening Prompt 03 — Persistent Multi-Byte Dirty-State Diagnostics

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty, stop and report it.

This prompt addresses the high-severity audit issue: multi-byte persistent writes can partially apply and leave the sensor in a dirty/unknown persistent configuration state.

## Important context

EE871-E2 persistent writes are not bus-atomic. A low byte can commit before a high byte fails, or a write can succeed before verify fails. The caller needs a clear diagnostic that persistent sensor state may need resync or operator inspection.

Do not over-redesign the API. Add the smallest clean diagnostics that solve the production problem.

## Subagents to spawn

1. `persistent-state-agent`
   - Inspect `writeMeasurementInterval()`, `writeCo2Offset()`, `writeCo2Gain()`, and any other multi-byte persistent/config writes.
   - Identify every transaction position where partial hardware state can occur.

2. `api-diagnostics-agent`
   - Propose the smallest public diagnostic API/snapshot additions.
   - Preserve compatibility where possible.

3. `fault-test-agent`
   - Add fake-transport tests for each partial-failure position.

## Implement only this scope

### 1. Add persistent dirty-state tracking

Add private state similar in intent to:

```cpp
bool _persistentConfigDirty;
Status _persistentConfigDirtyError;
```

Use project naming/style.

Set dirty state when a multi-byte persistent operation may have partially committed. At minimum cover:

- `writeMeasurementInterval()`;
- `writeCo2Offset()`;
- `writeCo2Gain()`;
- any equivalent multi-byte persistent setting discovered by code audit.

Do not set dirty before the first byte could have committed. Do set it after the first successful byte if a later byte/write/verify/delay/readback fails.

Preserve the original failing `Status`.

### 2. Expose diagnostics

Add public diagnostics with names fitting the repository style, for example:

```cpp
bool persistentConfigDirty() const;
Status persistentConfigDirtyError() const;
```

If the repo already has a settings/health snapshot, include:

- `persistentConfigDirty`;
- `persistentConfigDirtyError`.

Avoid large API redesign.

### 3. Clear dirty state only after verified resync

Clear dirty state only when the driver has positively confirmed persistent settings are coherent. Acceptable options:

- successful full `recover()` that reads/verifies the affected persistent registers;
- new explicit `resyncPersistentConfig()` if cleaner;
- successful full re-read of the persistent settings and validation.

Do **not** clear dirty state just because a later unrelated read succeeds.

### 4. Return precise failure

Existing write functions should still return the actual failing status, not a vague dirty-state status. Dirty state is an additional diagnostic, not a replacement for precise errors.

### 5. Add tests

Using the fake E2 harness from Prompt 02, test:

- `writeMeasurementInterval()` failure on low-byte write;
- failure after low byte succeeds but high byte fails;
- failure during verify after writes;
- `writeCo2Offset()` high-byte failure;
- `writeCo2Gain()` high-byte failure;
- dirty status exposes original error;
- unrelated successful read does not clear dirty state;
- successful recover/resync clears dirty state only when persistent fields are coherent;
- dirty state survives OFFLINE if that is the safest behavior.

### 6. Update docs

Update README/public docs:

- Multi-byte persistent writes are not bus-atomic.
- A failure can leave sensor persistent state partially changed.
- Use dirty diagnostics and recover/resync before trusting persistent configuration.
- Persistent writes are maintenance operations with long latency and endurance implications.

### 7. Update final report

Append Prompt 03 section to:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

Include API changes, tests, and remaining limitations.

## Required validation

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

## Commit and sync

Commit with a focused message:

```bash
git add include src test README.md docs
git commit -m "feat: report EE871 persistent dirty state"
git push
```

Only include files actually changed.


---

## ee871_prompt_04_idf_ci_docs.md

# EE871-E2 Hardening Prompt 04 — ESP-IDF Build Coverage, CI, and Example Contract Honesty

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty, stop and report it.

This prompt addresses the IDF validation gap. Do not change the core architecture unless a build failure requires a small fix.

## Important context

The audit found IDF artifacts and a native IDF example, but no local or CI proof that pure `idf.py` builds work. PlatformIO Arduino builds are not a substitute for pure ESP-IDF component/example validation.

## Subagents to spawn

1. `idf-build-agent`
   - Inspect root `CMakeLists.txt`, `idf_component.yml`, `examples/idf/basic_bringup`, and any IDF-specific docs.
   - Try to determine what `idf.py` commands should work.

2. `ci-agent`
   - Inspect `.github/workflows/ci.yml`.
   - Add pure ESP-IDF build coverage if feasible.

3. `idf-example-review-agent`
   - Confirm the IDF example remains native IDF.
   - Check GPIO ownership, timing callbacks, nonblocking loop/tick, and diagnostic-vs-production wording.

## Implement only this scope

### 1. Add pure ESP-IDF CI coverage

Update CI to run pure ESP-IDF builds for:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Use an official or common ESP-IDF CI container/action. Keep it simple and maintainable.

Make sure the CI job also runs:

```bash
python tools/check_idf_example_contract.py
```

### 2. Verify IDF component metadata

Check:

- root `CMakeLists.txt`;
- `idf_component.yml`;
- example `CMakeLists.txt`;
- include paths;
- dependency declarations.

Fix only obvious build/config mistakes.

### 3. Preserve framework boundary

Ensure IDF files do not leak into `include/` or `src/`.

Run or update guard scripts if needed, but avoid broad tool rewrites.

### 4. Improve IDF example honesty

Update README/docs to say:

- The IDF example is a diagnostic/basic bring-up example.
- It owns GPIO line setup for demonstration.
- Production users should integrate line callbacks into their application’s GPIO/bus manager.
- External serialization is required if multiple tasks can access the same EE871 instance.
- The device uses E2 GPIO signaling, not ESP-IDF hardware I2C.

### 5. Local IDF build attempt

Attempt:

```bash
idf.py --version
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

If `idf.py` is unavailable, record the exact error. Do not claim local IDF build success.

### 6. Update final report

Append Prompt 04 section to:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

Include:

- CI changes.
- Whether local `idf.py` was available.
- Exact local build/CI-relevant commands run.
- Remaining IDF validation gaps.

## Required validation

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_idf_example_contract.py
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
idf.py --version
git diff --check
```

If `idf.py` exists, run the two `idf.py -C ... build` commands above.

## Commit and sync

Commit with a focused message:

```bash
git add .github CMakeLists.txt idf_component.yml examples docs README.md tools
git commit -m "ci: add EE871 pure ESP-IDF build coverage"
git push
```

Only include files actually changed.


---

## ee871_prompt_05_final_validation_report.md

# EE871-E2 Hardening Prompt 05 — Hardware Validation Matrix, Final Integration Review, and Release/Merge Report

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty from unrelated user changes, stop and report it.

This prompt finalizes the hardening pass. It should not start large new implementation work unless a critical regression is found.

## Subagents to spawn

1. `final-diff-review-agent`
   - Review the full branch diff.
   - Look for broad refactors, accidental generated files, framework leakage, or docs overstating validation.

2. `hardware-validation-agent`
   - Create a realistic EE871-E2 hardware validation matrix and CLI sequence.
   - Do not claim hardware results unless commands are actually run.

3. `release-readiness-agent`
   - Decide what is merge-ready, release-ready, and still pending.
   - Identify public API changes and compatibility notes.

## Implement only this scope

### 1. Review the full diff

Run:

```bash
git diff --stat main...HEAD
git diff --check
git status --short
```

Inspect the actual diff. Confirm:

- no build artifacts;
- no accidental tarballs;
- no framework headers in core;
- no broad formatting churn;
- all tests are meaningful;
- docs match actual behavior.

### 2. Create hardware validation matrix

Create or update:

```text
docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md
```

Include concrete scenarios:

- power-up `begin()`;
- probe no-health-side-effects;
- read status;
- read CO2 fast/average values;
- PEC success on normal reads;
- persistent interval write/readback/power-cycle persistence;
- CO2 offset/gain write/readback, if safe to run;
- unplug/replug recovery;
- line stuck low/high if hardware jig exists;
- wrong wiring/no sensor;
- warm-up behavior;
- stale sample behavior, if observable;
- ESP32-S2 and ESP32-S3 boards separately.

Mark every item:

- `NOT RUN`;
- `PASS`;
- `FAIL`;
- `BLOCKED`;
- `NOT APPLICABLE`.

Default to `NOT RUN` unless actually run.

### 3. Add safe CLI/hardware test recipe

Add a concise hardware validation recipe to docs or README, for example:

```text
version
drv
probe
status
co2
co2avg
features
selftest
recover
```

Use the actual CLI command names in the repository. Do not invent commands. If commands do not exist, document the closest available commands or recommend adding them in future work.

Include warnings before persistent writes:

- persistent writes may change sensor configuration;
- they may have long delays;
- only run on a bench sensor where configuration changes are acceptable.

### 4. Finalize comprehensive hardening report

Complete:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

It must include:

- Date.
- Branch.
- Starting audit findings.
- Prompt-by-prompt change summary.
- Public API changes.
- Core changes.
- Test infrastructure changes.
- ESP-IDF/CI changes.
- Documentation changes.
- Exact commands run and exact results.
- Commands not run and why.
- Hardware validation status.
- Known remaining gaps.
- Future work needed for full industry-grade claim.
- Merge readiness verdict.
- Release readiness verdict.
- Compatibility/breaking-change notes.

Do not claim industry-grade unless all validation evidence exists.

Suggested verdict wording:

- “Ready to merge after CI passes” if tests/builds/docs are good but hardware validation is pending.
- “Not yet field/industry-grade until hardware fault validation and pure IDF CI pass.”
- “Production-oriented hardening complete; physical validation remains.”

### 5. Optional: package validation

Run package validation if project uses PlatformIO packaging:

```bash
python -m platformio pkg pack
```

Delete generated tarball after validation unless the project intentionally tracks it.

### 6. Final validation commands

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
python -m platformio pkg pack
git diff --check
git status --short
```

If `idf.py` is available:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Record exact results.

Delete generated package artifacts if appropriate, then re-run:

```bash
git status --short
```

### 7. Commit and sync

Commit with a focused message:

```bash
git add docs README.md .github tools test include src examples CMakeLists.txt idf_component.yml
git commit -m "docs: finalize EE871 industry hardening report"
git push
```

Only include files actually changed.

## Final response expected from the coding agent

Return a concise engineering summary:

1. Branch name.
2. Commits made.
3. Tests/checks run and exact results.
4. Hardware validation status.
5. Merge readiness.
6. Release readiness.
7. Remaining work for full industry-grade claim.
