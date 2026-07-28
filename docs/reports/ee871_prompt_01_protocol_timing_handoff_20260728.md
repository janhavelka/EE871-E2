# EE871 Prompt 01 Protocol Timing Handoff

Date: 2026-07-28

Implementation branch: `feature/ee871-hardening-series`

## Baseline

- Starting branch: `main`
- Starting commit: `6fd31e1a9a4181d063f47af0be7b04ace0fb91ec`
- `origin/main`: same commit
- `library.json` version: `1.0.0`
- Starting worktree: clean
- Version was not bumped.

Baseline validation before editing:

| Command | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS |
| `python scripts/generate_version.py check` | PASS |
| `python -m platformio test -e native` | PASS, 31/31 |
| `python -m platformio run -e ex_bringup_s3` | PASS |
| `python -m platformio run -e ex_bringup_s2` | PASS |
| `idf.py` availability | Unavailable on `PATH` |

## Implementation

The core remains a synchronous, framework-neutral, injected-callback library.
No product, task, queue, pin, schema, logging, or RTOS policy was added.

The targeted private refactor:

- centralized all `Config` validation and normalization in `_validateConfig`;
- introduced explicit ordinary, write-completion, and interval-commit wait
  classes;
- introduced saturating `ByteDeadline` accounting and a single raw/tracked
  write path with `WriteProgress`/`WriteEffect`;
- separated raw bus reset and pointer-write helpers;
- made START verify released physical SDA before creating a transition;
- bounded cleanup without replacing a more precise primary failure;
- began one completion budget after PEC and allowed only the final ACK/STOP to
  use it;
- delayed only the remaining completion budget after STOP;
- ordered every pointer write before dependent `0x51` traffic;
- retained pointer auto-increment for block reads;
- added cooperative, fixed-slice task-context completion waits.

Public additions:

- `Config::delayMs`, `Config::yield`, and `Config::longDelaySliceMs`, appended
  after all previous aggregate fields;
- timing validation constants in `CommandTable.h`;
- append-only `Err::VERIFY_MISMATCH = 15`;
- `OperationKind`, `OperationTimingBound`, and static/instance
  `operationTimingBound()` queries.

The native fake now supports phase-specific exact SCL stretch duration,
stuck-SDA START faults, line/transaction/delay/yield counts, transaction command
and effective custom-read address records, wrong readback, pointer/interval
completion ordering, and forced final-PEC NACK evidence.

## Post-Publication Audit Corrections

The audit of published Prompt 01 commit `d6100b0` found and corrected four
in-scope edge cases:

- ordinary STOP applies `bitTimeoutUs` only while polling SCL high; configured
  START/STOP holds are fixed timing outside that stretch deadline;
- a known `PEC_MISMATCH` remains primary if bounded cleanup STOP also fails;
- final-PEC NACK cleanup uses the existing 150/300 ms completion deadline,
  keeps `NACK` primary, and leaves the bus released when cleanup succeeds;
- public diagnostic `busReset()` is raw and health-neutral, while `recover()`
  remains the explicit tracked recovery path.

The fake timer now starts at the same completed-PEC boundary as production.
Interval tests prove `0xC6` ordinary staging before the `0xC7` commit, wait the
remaining commit window before verification, and exercise ordinary deadline
failure on the staged transaction. Delay tests verify exact 50/50/49 ms
fallback slices plus the microsecond remainder and prove stretch polling never
calls the task-context yield callback.

## Timing Formulas

Definitions:

```text
START = bitTimeoutUs + 2*startHoldUs + clockLowUs
STOP = 10 + bitTimeoutUs + 2*stopHoldUs
READ = START + 3*byteTimeoutUs + STOP
NORMAL_WRITE = START + 4*byteTimeoutUs + STOP
COMPLETION_WRITE(D) = START + 4*byteTimeoutUs + 1000*D
RESET = 9*(clockLowUs + bitTimeoutUs + clockHighUs)
        + clockLowUs + 10 + bitTimeoutUs + 2*stopHoldUs
```

| Operation | Formula |
| --- | --- |
| Control read | `READ` |
| Pointer write | `COMPLETION_WRITE(writeDelayMs)` |
| Custom byte read | pointer write + `READ` |
| Custom block read | pointer write + `count*READ` |
| Custom byte write/verify | two pointer/write completion frames + `READ` |
| Interval write/verify | `NORMAL_WRITE` + interval completion + pointer completion + `2*READ` |
| Part-name write/verify | 16 custom-byte write/verify procedures |
| Raw CO2 read | `2*READ` |
| Bus reset | `RESET` |

Exact derivation, normalization, count rules, and callback assumptions are in
`docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`.

## Files Changed

- Public API: `include/EE871/CommandTable.h`, `Config.h`, `EE871.h`, `Status.h`
- Core: `src/EE871.cpp`
- Native test support: `test/support/FakeE2Transport.h`
- Native tests: `test/test_basic.cpp`
- Diagnostic error formatting: Arduino and native ESP-IDF bring-up examples
- Maintained docs: `README.md`, `CHANGELOG.md`, `docs/README.md`, protocol map,
  hardening report, and the new timing-bound reference

## Final Software Validation

| Command | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS |
| `python scripts/generate_version.py check` | PASS |
| `python tools/check_cli_contract.py` | PASS |
| `python tools/check_idf_example_contract.py` | PASS |
| `python -m platformio test -e native` | PASS, 51/51 |
| `python -m platformio run -e ex_bringup_s3` | PASS |
| `python -m platformio run -e ex_bringup_s2` | PASS |
| `doxygen Doxyfile` | PASS with Doxygen 1.15.0 |
| `git diff --check` | PASS; line-ending conversion warnings only |

Local pure ESP-IDF validation was not run because `idf.py` is unavailable.
HIL, physical sensor, long-run, network, and fault-jig validation were not run
for this change. No such result is claimed.

## Intentional Deferrals

- Prompt 02: lifecycle/identity snapshot and probe behavior beyond this
  low-level transport correction.
- Prompt 03: checked CO2 sample types and status/measurement semantics.
- Prompt 04: full persistent-field uncertainty diagnostics, maintenance
  workflows, release/version work, and hardware evidence.

No later-prompt API or firmware integration was implemented early.
