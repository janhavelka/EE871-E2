# EE871 Prompt 04A Completion-Budget Handoff

Date: 2026-07-29

## Baseline And Scope

- Branch: `feature/ee871-hardening-series`
- Baseline/final HEAD: `1bffb9ef4565ab8a848c845088fc5e37c65c5faf`
  (`feat: harden comprehensive EE871 HIL runner`)
- Library version: `1.1.0`
- At report generation, no commit, tag, version bump, release, push, firmware
  change, or live HIL had been performed.
- The baseline working tree already contained a modified
  `docs/prompts/README.md` and untracked Prompt 04A/04B/04C files. Those prompt
  authoring changes were preserved; the 04A supersession text in the README is
  applicable to this correction.

Baseline software validation passed: core/public timing, CLI, IDF example, and
generated-version contract checks; 91/91 native tests; and both
`ex_bringup_s2` and `ex_bringup_s3` builds. `git diff --check` reported only
the repository's CRLF conversion warnings.

## Corrected Accounting

The long-completion path now owns two explicit timing domains:

1. one cumulative `CompletionBudget` for sensor-held-low polling during final
   PEC ACK and STOP, followed by only the unused quiet-wait remainder;
2. deterministic DATA setup, ACK clock-high/low, and STOP setup/hold timing as
   separate, configuration-bounded master protocol tail.

The allowance starts after the eight PEC data bits. An exact 150,000 us normal
write/pointer stretch or 300,000 us interval-commit stretch succeeds. The next
5 us fake poll fails with `TIMEOUT`. ACK and STOP share the same allowance, so
a split exact total succeeds and an over-limit total fails without a second
completion delay.

Ordinary pre-PEC bits and bytes still use the 25 ms/35 ms deadlines. Final NACK
remains primary `NACK`; an unobserved final ACK remains `INDETERMINATE`; and an
observed ACK followed by STOP failure remains `ACKNOWLEDGED` and unresolved.
Readback verification is unchanged.

## Targeted Changes

- `include/EE871/EE871.h`, `src/EE871.cpp`: added the private completion record,
  routed final-ACK/STOP polling through it, retained the ordinary
  `ByteDeadline`, and reused the existing bounded cooperative long-wait helper
  for only the budget remainder.
- `test/support/FakeE2Transport.h`, `test/test_basic.cpp`: added production-path
  final-ACK plus STOP split-stretch injection and repaired/extended exact-limit,
  over-limit, elapsed-time, ordering, mutation, and bound tests.
- `include/EE871/Config.h`, `README.md`, `CHANGELOG.md`, protocol/timing docs,
  active hardening guidance, and Prompt 01: documented the corrected sensor
  allowance versus master-tail model and the Prompt 04A supersession.

The shared conservative completion-write term is now:

```text
START
+ 4*byteTimeoutUs
+ completionAllowance
+ (SETUP + clockHighUs + clockLowUs)
+ (SETUP + 2*stopHoldUs)
```

Minimum-hold public bounds changed where millisecond rounding exposes the
added tail:

| Operation | Previous | Corrected |
| --- | ---: | ---: |
| Interval write/verify | 1281 ms | 1282 ms |
| 16-byte part-name write/verify | 12566 ms | 12573 ms |
| 2-byte block write/verify | 1571 ms | 1572 ms |
| 3-byte block write/verify | 2357 ms | 2358 ms |
| Persistent resync | 7025 ms | 7027 ms |
| Auto-adjust maintenance | 1256 ms | 1257 ms |
| Largest accepted 256-byte block read | 112113 ms | 112246 ms |
| Largest accepted 16-byte write composite | 176154 ms | 180412 ms |

Other published example bounds remain numerically unchanged after ceiling to
milliseconds, while still using the corrected shared formula.

## Corrective Audit

A separate complete-diff audit found no additional production timing defect.
It strengthened requirement-17 evidence with maximum-config production calls
covering reset, interval stage/commit, pointer/read, and the largest 16-byte
write composite. It also added direct quiet-remainder/no-second-wait callback
assertions and exact timeout-detail assertions. The audit also split ordinary
and completion SCL-high wait helpers and removed unreachable completion-error
branches from the ordinary byte-delay helper. Active downstream prompt
preconditions now require completion of corrective Prompts 04A-04C before an
authorized immutable release can be exact-pinned.

## Final Validation

| Command | Result |
| --- | --- |
| `python -m platformio test -e native` | PASS, 92/92 |
| `python -m platformio run -e ex_bringup_s2` | PASS |
| `python -m platformio run -e ex_bringup_s3` | PASS |
| `python tools/check_core_timing_guard.py` | PASS |
| `python tools/check_public_timing_contract.py` | PASS |
| `python tools/check_cli_contract.py` | PASS |
| `python tools/check_idf_example_contract.py` | PASS |
| `python scripts/generate_version.py check` | PASS at 1.1.0 |
| `doxygen Doxyfile` | PASS |
| `git diff --check` | PASS; CRLF conversion warnings only |

The focused native cases cover exact/first-over 150 ms final ACK, exact/first-
over 150 ms pointer STOP, exact/first-over 300 ms interval final ACK and STOP,
split cumulative completion, direct no-duplicate/remaining-quiet-wait callback
evidence, exact timeout diagnostic boundaries, NACK/cleanup precedence,
acknowledged timeout classification, ordinary pre-PEC deadlines, pointer
ordering, every public timing kind, and maximum-config production paths for
reset, interval commit, block read, and the largest 16-byte write composite.

`idf.py` was unavailable on `PATH`, so no pure ESP-IDF build was run. No
physical sensor, waveform, power, stuck-line, persistent-write, calibration,
address, auto-adjust, or other HIL validation was run. Exact-limit timing and
waveform behavior therefore remain pending HIL.

The repository has no local `v1.1.0` tag. If 1.1.0 has already been published
externally, maintainers must make an explicit patch-release decision for this
correction; this work does not invent that release.

Prompt 04A is not the complete release gate. Corrective Prompts 04B and 04C
remain required before an authorized immutable release or downstream
exact-pinning. The pre-existing Prompt 04A-04C authoring files should be
reviewed and staged according to their own scope rather than swept into an
unrelated implementation commit.
