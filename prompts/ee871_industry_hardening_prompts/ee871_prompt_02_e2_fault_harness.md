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
