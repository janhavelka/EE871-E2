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
