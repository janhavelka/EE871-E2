# Prompt 04A: EE871 Write-Completion Budget Correction

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in:

```text
EE871-E2
```

Prompts 01-04 and their handoffs are the implementation baseline. This is a
targeted pre-HIL correction of one confirmed protocol-timing defect. Do not
edit consuming firmware, add tasks or queues, choose pins, change product
policy, or begin live HIL.

Keep the library synchronous, framework-neutral, fixed-state, and bounded.
Prefer a small refactor of the existing deadline owner over adding another
write path or increasing timeout constants.

## Authoritative Correction

This prompt supersedes Prompt 01 only where Prompt 01 says that final-ACK and
STOP protocol overhead consume the device's 150/300 ms completion allowance.

The supplied EE871 documents allow:

- up to 150 ms for a normal `0x10` flash write and `0x50` pointer operation;
- up to 300 ms for the committing `0xC7` write of the `0xC6`/`0xC7` interval
  pair;
- SCL to remain low until the internal routine completes when communication is
  attempted during that interval.

Therefore an exact 150,000 us or 300,000 us legal device-held-low interval
must succeed. Deterministic master protocol work needed to sample the final
ACK and issue STOP is bounded protocol tail, not part of the device's maximum
internal completion duration.

Still allow only one cumulative completion allowance. Do not permit a full
150/300 ms independently on final ACK and again on STOP, and do not wait
another full completion interval after already consuming it as clock stretch.

## Read Before Editing

Read completely:

- `AGENTS.md`;
- `docs/prompts/README.md`;
- `docs/prompts/01_ee871_protocol_timing_and_fault_precision.md`;
- Prompt 01 and Prompt 04 handoffs under `docs/reports/`;
- `include/EE871/Config.h`;
- `include/EE871/CommandTable.h`;
- `include/EE871/EE871.h`;
- `src/EE871.cpp`;
- the timing fake and completion tests in `test/`;
- `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`;
- `docs/pdf-extracted-md/EE871_E2_interface_addendum.md`;
- `docs/pdf-extracted-md/EE871_E2_CO2_interface_AN1611-1.md`.

Inspect the complete working tree and preserve unrelated changes. Record the
branch, commit, version, dirty state, and baseline validation in the handoff.

## Confirmed Defect

The current implementation starts one 150/300 ms deadline after PEC but also
charges final-ACK setup/high/low timing and STOP setup/hold timing to that same
limit. Native tests consequently require:

- approximately 149.7 ms to succeed but exactly 150 ms to fail;
- approximately 299.7 ms to succeed but exactly 300 ms to fail.

That rejects a standards-compliant worst-case sensor. It can also leave a
successful persistent request reported as timed out and unresolved.

Do not solve this by changing defaults to arbitrary larger numbers. Correct
the ownership of completion time.

## Required Targeted Refactor

Keep the ordinary `ByteDeadline` behavior for normal data bits and bytes.
Refactor the existing long-completion path so it separately accounts for:

1. cumulative device-completion time:
   - SCL-low polling during the final PEC ACK and final STOP;
   - any remaining quiet completion wait after STOP;
2. deterministic protocol-tail time:
   - data setup;
   - clock-high and clock-low phases;
   - STOP setup and hold timing.

Use one small private record, with equivalent naming allowed:

```cpp
struct CompletionBudget {
  uint32_t consumedUs{0};
  uint32_t limitUs{0};
};
```

If total diagnostic elapsed time is still needed, keep it separately. Do not
reuse one counter with two meanings.

The required behavior is:

- start the completion allowance immediately after the complete PEC byte has
  been transferred;
- only the final PEC ACK and final STOP SCL-high waits may consume long
  clock-stretch time;
- every long SCL-low poll consumes the one shared `CompletionBudget`;
- deterministic ACK/STOP protocol delays do not reduce the sensor's allowed
  completion duration, but remain explicitly bounded by validated `Config`;
- after STOP, cooperatively wait only
  `limitUs - consumedUs`;
- if the device consumes the exact limit as clock stretch, perform no extra
  quiet completion wait and complete the bounded ACK/STOP tail;
- reject the first poll beyond the configured limit as `TIMEOUT`;
- a split stretch across final ACK and STOP shares the same allowance;
- ordinary pre-PEC bits retain the 25 ms per-bit and 35 ms per-byte limits.

Reuse the current `_waitSclHigh()`, `_readAck()`, `_e2Stop()`, and
`_writeCommandRaw()` ownership where practical. It is acceptable to add a
small completion-specific wait/delay helper when that makes the two accounting
domains explicit. Do not duplicate the transaction frame or introduce a
generic deadline framework.

Preserve precise write progress:

- final NACK remains definite `NACK`;
- no observed final ACK remains `INDETERMINATE` where already required;
- an observed final ACK followed by tail failure remains acknowledged and
  unresolved;
- a completion overrun remains `TIMEOUT`;
- readback verification behavior is unchanged.

Long waits must continue using the existing bounded `delayMs`/`yield` slices.
Do not add `delay()`, wall-clock dependencies, heap state, or background work.

## Public Timing Bounds

Do not add a new public operation kind or change existing enum values.

Update the existing operation-bound calculation so every completion-write
bound is built once from the same conservative terms:

```text
START
+ four ordinary transmitted-byte bounds
+ configured device completion allowance
+ fixed bounded final-ACK waveform
+ worst bounded STOP/cleanup tail
```

Use one shared helper for this formula. It may remain deliberately
conservative and may overestimate safely, including final-NACK cleanup. It
must never underestimate the actual implementation, omit or double-count the
tail, double a full completion allowance, wrap integer arithmetic, or depend
on measured hardware speed.

Update:

- normal `0x10` write/verify bounds;
- `0x50` pointer and dependent-read bounds;
- interval-pair bounds;
- lifecycle/capability/checked-sample bounds that include pointer writes;
- maximum valid configuration tests and published examples.

Keep `writeDelayMs = 150` and `intervalWriteDelayMs = 300` defaults and their
existing normalization/maxima unless a separate accepted decision changes
them.

## Native Fake Coverage

Repair tests that encode the defect and add focused coverage proving:

1. exact 150,000 us final-ACK stretch succeeds at the normalized default;
2. the first fake step beyond 150,000 us returns `TIMEOUT`;
3. exact 300,000 us interval-commit stretch succeeds;
4. the first fake step beyond 300,000 us returns `TIMEOUT`;
5. exact 150,000 us STOP stretch on a `0x50` pointer write succeeds and the
   first fake step over fails;
6. exact 300,000 us STOP stretch on the committing `0xC7` write succeeds and
   the first fake step over fails;
7. final-ACK and STOP stretches consume one cumulative allowance;
8. a split total exactly equal to the allowance succeeds;
9. a split total above the allowance fails;
10. no second full completion wait occurs after a long stretch;
11. a short stretch is followed only by the remaining quiet wait;
12. final ACK observed followed by STOP failure remains `ACKNOWLEDGED` and
    unresolved;
13. final PEC NACK remains primary `NACK` with no accepted mutation even if
    cleanup STOP also fails;
14. an acknowledged completion timeout retains its existing precise mutation
    classification;
15. pre-PEC stretch still uses ordinary bit/byte limits;
16. pointer reads never begin before corrected completion finishes;
17. all published `OperationTimingBound` values cover observed fake elapsed
    time at exact limits and largest accepted configuration.

Tests must exercise the production transaction implementation. Do not create a
test-only timing algorithm.

## Documentation and Superseded Guidance

Update all active statements that describe the old total-budget
interpretation, including:

- `README.md`;
- `CHANGELOG.md` under `Unreleased`;
- `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`;
- current active hardening guidance;
- Prompt 01 or the prompt-series README with an explicit supersession note so
  the incorrect rule is not executed again.

Explain the distinction between:

- one cumulative sensor completion allowance;
- bounded master protocol tail;
- remaining post-STOP quiet wait.

Do not rewrite historical validation as though hardware was rerun. Record this
as a source/native correction pending HIL.
Do not rewrite versioned historical release notes or earlier handoff evidence.
Put release-facing text in `CHANGELOG.md` under `Unreleased` and in the new
corrective handoff unless the repository has an explicit errata policy.

Do not edit generated `Version.h`. Do not bump, tag, or release unless the
outer user instruction explicitly authorizes it. If version `1.1.0` is already
published, report that a patch-release decision is required rather than
inventing one.

## Validation

Run at least:

```text
python -m platformio test -e native
python -m platformio run -e ex_bringup_s2
python -m platformio run -e ex_bringup_s3
python tools/check_core_timing_guard.py
python tools/check_public_timing_contract.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
git diff --check
```

Also run every repository timing/contract command required by current
guidelines. If ESP-IDF tooling is available and current repository guidance
requires it, run the documented build; otherwise report it as unavailable.

Do not run live HIL in this prompt.

## Handoff

Create a concise report under `docs/reports/` containing:

- baseline and final commit/version state;
- the corrected time-accounting model;
- files and targeted refactors;
- exact-limit and over-limit test evidence;
- operation-bound changes;
- tests/builds actually run;
- remaining hardware validation.

## Acceptance Criteria

- An exact documented 150 ms or 300 ms device-held-low completion succeeds.
- The first completion step beyond the configured allowance fails boundedly.
- ACK and STOP share one cumulative device-completion allowance.
- Deterministic protocol tail no longer steals legal device completion time.
- No second full completion delay is added.
- Ordinary bit/byte deadlines and error precision are unchanged.
- Persistent uncertainty remains truthful on every failure phase.
- Published blocking bounds cover the corrected implementation.
- No async framework, scheduler, retry policy, or firmware behavior was added.
- Native tests and both ESP32 example builds pass.
- No HIL result is invented.
