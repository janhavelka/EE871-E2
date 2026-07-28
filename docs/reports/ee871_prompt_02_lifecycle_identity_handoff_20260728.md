# Prompt 02 Lifecycle, Identity, And Recovery Handoff

Date: 2026-07-28
Branch: `feature/ee871-hardening-series`
Baseline commit: `739beeca9017bc678f026d4b466aa2bb6332b08e`
Version: `1.0.0` (unchanged)

## Baseline

The branch was clean and synchronized before Prompt 02. Prompt 01's handoff
existed, and its required core timing guard, generated-version check, 51 native
tests, and ESP32-S3/S2 Arduino example builds passed. Local `idf.py` was not
available.

## Implementation

- Added append-only `BeginPolicy`, `Err::OFFLINE`, lifecycle
  `OperationKind` values, `DeviceIdentity`, and `CapabilitySnapshot`
  contracts.
- Appended policy, startup diagnostic, identity, and capability fields to
  `SettingsSnapshot`; legacy feature bytes remain and are populated from the
  same capability snapshot.
- Reused one identity procedure for raw begin/probe and tracked recovery. It
  validates group `0x0367`, subgroup `0x09`, and advertised CO2 support.
- Loads capability bytes `0x03..0x09` with one completed pointer write and
  seven sequential reads. Identity and capabilities publish together only
  after the complete operation succeeds.
- Added one common guard to all tracked read, pointer, write, and reset
  wrappers. Normal operations return exact `OFFLINE` without line I/O; raw
  diagnostics and a narrowly scoped recovery bypass remain available.
- Recovery now performs a tracked reset, full identity validation, and full
  capability reload. It is the sole route from initialized `OFFLINE` to
  `READY`.
- Extended the deterministic callback-boundary fake with configurable identity,
  seven capability bytes, and one-shot failure by transfer index.

No task, queue, pin, warm-up, cadence, schema, retry, product, or firmware
policy was added.

## State Transitions

| Entry | Result | State | Cache |
| --- | --- | --- | --- |
| Strict begin, all discovery succeeds | `OK` | `READY` | Full identity and capabilities valid |
| Strict begin, any discovery fails | Original failure | `UNINIT` | Invalid and zero |
| Optional begin, definite identity absence | `OK` | `OFFLINE` | Invalid and zero |
| Optional begin, other discovery failure | Original failure | `UNINIT` | Invalid and zero |
| Normal tracked operation reaches threshold | Original transfer failure | `OFFLINE` | Existing cache retained until failed offline recovery or semantic incompatibility |
| Raw `probe()` | Original diagnostic result | Unchanged | Unchanged |
| Complete `recover()` | `OK` | `READY` | Fresh full identity and capabilities |
| Failed recovery entered from `OFFLINE` | Original failure | `OFFLINE` | Invalid and zero |
| Responding incompatible recovery | `NOT_SUPPORTED` | `OFFLINE` | Invalid and zero |
| `end()` | n/a | `UNINIT` | Policy diagnostics and identity/capabilities reset |

The accepted-absence and semantic-incompatibility latch normalizes
`consecutiveFailures` to at least `offlineThreshold` to preserve the existing
four-state invariant. This does not increment lifetime transport failures or
invent a transfer timestamp.

## Accepted And Rejected Optional-Startup Results

| Identity discovery result | `ALLOW_ABSENT` behavior |
| --- | --- |
| Cleanly terminated `NACK` | Accept as absent; initialized and `OFFLINE` |
| Definite `DEVICE_NOT_FOUND` | Accept as absent; initialized and `OFFLINE` |
| `NACK` with failed cleanup STOP | Reject, retain primary NACK, and remain `UNINIT` |
| `TIMEOUT`, `BUS_STUCK`, `PEC_MISMATCH` | Reject and remain `UNINIT` |
| Wrong group/subgroup or missing CO2 bit | Reject as `NOT_SUPPORTED` |
| Any capability pointer/read failure | Reject and remain `UNINIT` |
| Invalid configuration or internal/cleanup error | Reject unchanged |

The original accepted absence is retained in
`SettingsSnapshot::beginProbeStatus`; transport counters, `lastError`, and
`lastErrorMs` remain unchanged.

## Cache Atomicity

Identity and capability helpers write only zero-initialized local candidates.
The live cache is updated by one publish helper after both candidates are
complete. A partial capability read cannot publish any of the seven bytes.
`identity()`, `capabilities()`, and both settings accessors are cache-only.
Raw probe validates a temporary identity but publishes nothing.

Recovery entered from `DEGRADED` follows existing per-transfer health
semantics: successful sub-transfers reset the earlier streak, and a later
failure begins a new streak. It does not restore a stale entry streak or
publish a partial replacement cache.

## Timing Bounds

Public numeric values were appended without renumbering:

| Kind | Value | Minimum-hold reference bound |
| --- | ---: | ---: |
| `BEGIN_REQUIRE_PRESENT` | 9 | 2274 ms |
| `BEGIN_ALLOW_ABSENT` | 10 | 2274 ms |
| `PROBE_IDENTITY` | 11 | 621 ms |
| `RECOVER_IDENTITY_AND_CAPABILITIES` | 12 | 2274 ms |

Strict/optional begin and recovery reserve reset, four identity reads, one
completed pointer write, and seven capability reads. Probe reserves four
identity reads. Native tests verify the queries are bus-silent and observed
fake time does not exceed the advertised bounds.

## Files

- `include/EE871/CommandTable.h`
- `include/EE871/Config.h`
- `include/EE871/EE871.h`
- `include/EE871/Status.h`
- `src/EE871.cpp`
- `test/support/FakeE2Transport.h`
- `test/test_basic.cpp`
- `examples/01_basic_bringup_cli/main.cpp`
- `examples/idf/basic_bringup/main/main.cpp`
- `README.md`
- `CHANGELOG.md`
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`
- this handoff

## Validation

- `python tools/check_core_timing_guard.py`: PASS.
- `python tools/check_cli_contract.py`: PASS.
- `python tools/check_idf_example_contract.py`: PASS.
- `python scripts/generate_version.py check`: PASS; `Version.h` is current.
- `python -m platformio test -e native`: PASS, 65/65.
- `python -m platformio run -e ex_bringup_s3`: PASS.
- `python -m platformio run -e ex_bringup_s2`: PASS.
- `git diff --check`: PASS; only Git line-ending conversion warnings.
- `idf.py --version`: unavailable, so local pure ESP-IDF builds were not run.

No HIL, physical sensor, waveform, network, Cloud, or long-run validation was
performed for this prompt.

## Post-Publication Audit Correction

The audit of published Prompt 02 commit `974f730` found that optional startup
classified a primary NACK as absence even when its cleanup STOP timed out.
Production now carries private clean-termination evidence alongside the raw
identity read. It retains Prompt 01's precise primary NACK but accepts that NACK
as absence only after cleanup completed. A deterministic absent-device plus
STOP-timeout regression proves the driver remains `UNINIT`, with invalid
caches, zero health counters, and no retry.

The audit also:

- removed obsolete example-side group/subgroup remapping to
  `DEVICE_NOT_FOUND`;
- aligned the binding lifecycle guideline with recover-only OFFLINE revival;
- clarified semantic `NOT_SUPPORTED` in `lastError` without a fake transport
  timestamp or counter;
- removed handoff whitespace that invalidated the originally recorded diff
  check.

The complete validation set above was rerun after these corrections. The
native suite remained 65/65, both Arduino firmware builds passed, the
repository contract checks passed, and the cumulative Prompt 02 diff is clean.

## Explicit Deferrals

- Checked CO2 value/status/error sample procedures remain Prompt 03.
- Persistent maintenance/calibration completion and release preparation remain
  Prompt 04.
- All task ownership, GPIO/pins, product profiles, data schemas, retry cadence,
  health/UI/storage/Cloud integration, and Co2Control-only production
  composition remain downstream firmware Prompts 5-8.
- This prompt does not create a release or change the version.
