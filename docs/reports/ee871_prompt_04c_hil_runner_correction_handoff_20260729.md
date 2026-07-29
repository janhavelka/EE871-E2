# Prompt 04C HIL Runner Correction Handoff

Date: 2026-07-29

Branch: `feature/ee871-hardening-series`

Baseline commit: `d40f70a41b88584c020f9efc16889ffcade69117`

Library version: `1.1.0` (unchanged)

Runner version: `2.1` -> `2.2`

Hardware/HIL status: not run

## Scope And Final State

This change corrects the repository HIL runner, parser/planner tests, and
active HIL guidance. It does not change the synchronous core library, public
API, diagnostic firmware command surface, pins, tasks, or product policy.

The starting worktree was clean. Baseline validation passed 50 runner-parser
tests, 104 native tests, both CLI contract checks, and Arduino ESP32-S2/S3
builds. The final source adds fail-closed preflight and evidence handling while
keeping quick/default and complete-safe plans non-destructive.

## Corrections

- D9 is no longer globally volatile. `custom_memory_diff()` accepts D9 only
  when the selected mutation target is `AUTO_ADJUST`; every unrelated and
  final-restoration comparison treats a D9 change as unexpected.
- Response classification now evaluates semantic failures before success
  tokens. Definite non-OK statuses cannot become operator review merely
  because success-only text is absent. The narrow intentional non-OK contracts
  remain explicit.
- Destructive admission now independently requires a complete baseline group,
  immutable complete 256-byte snapshot, capture timestamp, fresh clean
  mutation diagnostic, matching configured/persisted address, capability-aware
  idle auto-adjust evidence, and a typed baseline for the selected target.
- Baseline evidence includes device firmware, diagnostic E2 specification
  version, all typed persistent reads, explicit `NOT_SUPPORTED` results, and
  the full memory image. Supported/unsupported typed results are checked
  against cached capabilities.
- Each reversible typed row now verifies test readback, exact mutation
  evidence, target-only full-image diff, typed restoration, restored readback,
  exact restoration mutation evidence, and full-image equality. Any failure
  or uncertainty blocks later writes; raw memory is never replayed.
- Checked samples capture `features`/`caps` first and validate both legitimate
  detailed-error capability branches. A coherent sensor-domain error remains
  a healthy-bench failure without an invented transport failure.
- SDA-low and SCL-low plans now capture pre-fault health, prove only the
  selected single line is low, require exact `BUS_STUCK` from `buscheck`,
  tracked `status`, and `libreset`, capture in-fault health while the jig is
  applied, then require explicit READY recovery.
- Baseline bytes, typed baselines, and
  `baseline_custom_memory_captured_utc` remain immutable. Checkpoints use a
  separate `updated_utc`.
- Hazardous live-plan metadata rejects trimmed empty, whitespace-only, and
  case-insensitive `unspecified` values.
- The effectful filter-write runner option and restoration branch were
  removed. Filter remains read-only baseline evidence. Interval-factor zero is
  rejected; valid explicit values are `-128..-1` and `1..127`.
- Auto-adjust remains one-shot, non-replayable, non-cancellable,
  non-restorable, and forensic. The runner no longer automatically invokes
  resync after the action.

## Preflight Invariants

| Evidence | Required before every destructive transmission |
| --- | --- |
| Baseline group | Every baseline row passed |
| Custom memory | Complete immutable 256-byte image and original capture time |
| Mutation state | Fresh epoch; dirty false; resync false; unresolved false; resolved dirty error OK |
| Address | Typed baseline equals immutable `configured_device_address` |
| Auto-adjust advertised | Typed status parsed as idle |
| Auto-adjust unadvertised | Clean `NOT_SUPPORTED` with no contradictory typed state |
| Selected target | Parsed typed restore baseline in its valid typed range |

No validator or guard invokes resync, acknowledges uncertainty, scans an
address, retries a one-shot action, or constructs a write from raw baseline
bytes.

## Plan Corrections

- Complete-safe records feature/capability evidence before checked samples.
- Reversible rows end with a target-owner restore and a full-image equality
  check.
- Address candidate reconciliation records a fresh resolved `dirty` result
  before the separately authorized restore request.
- Auto-adjust contains one `autoadj start`, no restore, no retry, and no
  automatic resync.
- Each stuck-line group follows:
  `levels`, pre-fault `drv`, apply jig, in-fault `levels`, `buscheck`,
  tracked `status`, in-fault `drv`, `libreset`, release jig, `levels`,
  `recover`, final `drv`.
- Quick/default and complete-safe contain no destructive command.
- No plan contains an effectful filter command.

## Validation Performed

| Command/check | Result |
| --- | --- |
| `python -m py_compile tools/ee871_hil_runner.py` | PASS |
| `python test/test_hil_runner_parser.py` | PASS, 68/68 |
| `python tools/check_cli_contract.py` | PASS |
| `python tools/check_idf_example_contract.py` | PASS |
| `python tools/check_core_timing_guard.py` | PASS |
| `python tools/check_public_timing_contract.py` | PASS |
| `python scripts/generate_version.py check` | PASS |
| `python -m platformio test -e native` | PASS, 104/104 |
| `python -m platformio run -e ex_bringup_s2` | PASS |
| `python -m platformio run -e ex_bringup_s3` | PASS |
| Runner `--version` / `--help` inspection | PASS; version 2.2, no filter-write option |
| Structured dry-run plan inspection | PASS for quick/default, complete-safe, interval, nonzero factor, mode, part name, calibration, address, auto-adjust, unplug/replug, both stuck lines, and power cycle |
| Safe-plan/filter assertions | PASS; no safe destructive command and no filter-write plan |
| `git diff --check` | PASS; line-ending conversion warnings only |
| Native ESP-IDF example build | NOT RUN; `idf.py` unavailable on `PATH` |

The local E2/EE871 PDFs were cross-checked for D9 action semantics,
capability bits, nonzero factor meaning, product-specific filter values, and
held-low timing. Poppler rendering tools were unavailable; text extraction
was used for this targeted protocol check.

## Remaining Operator And HIL Work

No serial port was opened and no live, destructive, calibration, address,
auto-adjust, unplug, stuck-line, power-cycle, waveform, or other hardware HIL
was run. All current hardware-matrix rows retain their truthful evidence
status.

Before live hazardous HIL, the operator still needs dedicated hardware,
reviewed open-drain/current-limited fixtures, verified level shifting and
pull-ups, isolated sensor power where applicable, meaningful metadata,
restoration authority, calibration references, and external waveform capture
for timing claims. Filter-write HIL remains blocked until an authoritative
numeric table and reviewed restoration procedure exist.
