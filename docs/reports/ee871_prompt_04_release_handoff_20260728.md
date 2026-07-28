# Prompt 04 Persistent Maintenance And Release Handoff

Date: 2026-07-28

Branch: `feature/ee871-hardening-series`

Baseline commit: `5df2515a46c604ee5dcbe5da02c7950a5bc3169e`

Baseline version: `1.0.0`

Final source version: `1.1.0`

Release/tag status: not created; separate authorization required

## Baseline

Prompts 01 through 03 and their audits were committed before this work.
Prompt 03 recorded 75/75 native tests plus passing Arduino ESP32-S3/S2 builds
and source/contract checks. Local `idf.py` was unavailable. The Prompt 04
implementation began from the clean baseline commit above; concurrent Prompt
04 implementation changes were intentionally reviewed as one final diff.

## Implementation And Cleanup

- Replaced independent persistent dirty/write-progress decisions with one
  mutation admission, frame-completion, effect-classification, fixed intent,
  and public diagnostic model.
- Routed every effectful public custom-memory operation through that path.
- Added precise element progress and retained first-cause/observation evidence.
- Added bus-silent mutation admission rejection while uncertainty is active.
- Refactored resync around `MutationTarget`, including capability-aware
  no-target coherence reads.
- Implemented explicit candidate-session bus-address reconciliation without
  old-address readback or scanning.
- Implemented auto-adjust pre/post observation and the one narrow cache-only
  operator acknowledgement.
- Capability-gated every typed optional-setting read and write, including
  offset/gain, before bus I/O.
- Preserved unresolved evidence across stopped/rebegin state on the same
  object.
- Preserved source-compatible dirty accessors as derived mirrors; no parallel
  dirty state or compatibility shim remains.
- Added exhaustive timing documentation and a source audit tied to public
  Doxygen markers, `OperationKind`, and the calculator.
- Kept Arduino/native ESP-IDF raw commands and added equivalent checked
  commands and mutation evidence.
- Bumped version metadata to 1.1.0 through repository tooling. The generator
  now synchronizes `Version.h`, `idf_component.yml`, and `Doxyfile` from
  `library.json`.

## Mutation Effect Table

| Effect | Unresolved? | Evidence/transition |
| --- | --- | --- |
| `NONE` | No | Validation, lifecycle, capability, or existing-uncertainty guard rejected before an effectful frame. |
| `NO_EFFECT` | No | Definite NACK/failure before a complete effectful request transferred. |
| `ACKNOWLEDGED` | Yes unless target verification succeeds | Request was definitely accepted but later STOP, completion, observation, or target policy did not prove final state. |
| `INDETERMINATE` | Yes | Complete PEC transferred but the final acceptance response was ambiguous. |
| `VERIFIED` | No | Immediate/deferred readback or action observation matched the retained intent. |
| `RESYNCHRONIZED` | No | Later coherent readback established actual state but did not prove original intent. |
| `OPERATOR_ACKNOWLEDGED` | No | Only the documented ambiguous auto-adjust not-running case was explicitly accepted without I/O. |

The first uncertainty cause remains until explicit reconciliation. Unrelated
reads, probe, health success, recover, end, and begin do not clear it.

## Exact Public Additions

```cpp
enum class MutationTarget : uint8_t {
  NONE = 0,
  RAW_CUSTOM_BYTE = 1,
  PART_NAME = 2,
  BUS_ADDRESS = 3,
  GLOBAL_INTERVAL = 4,
  CO2_INTERVAL_FACTOR = 5,
  CO2_FILTER = 6,
  OPERATING_MODE = 7,
  AUTO_ADJUST = 8,
  CO2_OFFSET = 9,
  CO2_GAIN = 10,
};

enum class MutationEffect : uint8_t {
  NONE = 0,
  NO_EFFECT = 1,
  ACKNOWLEDGED = 2,
  INDETERMINATE = 3,
  VERIFIED = 4,
  RESYNCHRONIZED = 5,
  OPERATOR_ACKNOWLEDGED = 6,
};
```

`MutationDiagnostic`, `mutationDiagnostic()`,
`acknowledgeAutoAdjustUncertainty()`, and
`SettingsSnapshot::mutation` were added. `Err` appends:

```cpp
PERSISTENT_STATE_UNCERTAIN = 18
```

`OperationKind` appends without renumbering earlier values:

```cpp
CUSTOM_BLOCK_WRITE_VERIFY = 15
RESYNC_PERSISTENT_CONFIG = 16
AUTO_ADJUST_MAINTENANCE = 17
BUS_ADDRESS_CHANGE = 18
```

The full `MutationDiagnostic` field contract is documented in the public
header and 1.1.0 release notes.

## Timing Completion

New formulas use the existing terms `R`, `P`, `W`, `N`, and `I`:

| Kind | Formula |
| --- | --- |
| `CUSTOM_BLOCK_WRITE_VERIFY` | `elementCount*(W + P + R)`, count 1..16 |
| `RESYNC_PERSISTENT_CONFIG` | `9*P + 27*R`, count exactly 1 |
| `AUTO_ADJUST_MAINTENANCE` | `W + 2*P + 2*R`, count exactly 1 |
| `BUS_ADDRESS_CHANGE` | `W`, count exactly 1 |

The fixed resync bound covers every complete supported setting and every
target-specific reconciliation. Health counters are documented as tracked
transfer counters. `tick()` remains a caller-supplied timestamp store.

## Files And Areas Changed

- Public contracts and implementation:
  `include/EE871/EE871.h`, `include/EE871/Status.h`, `src/EE871.cpp`.
- Native fake/tests: `test/support/FakeE2Transport.h`,
  `test/test_basic.cpp`.
- Examples and contracts: Arduino and native ESP-IDF bring-up sources/adapters,
  `tools/check_cli_contract.py`, `tools/check_idf_example_contract.py`.
- Timing/release tooling: `tools/check_public_timing_contract.py`,
  `scripts/generate_version.py`.
- Version metadata: `library.json`, generated `include/EE871/Version.h`,
  `idf_component.yml`, `Doxyfile`.
- Maintained docs: `README.md`, `CHANGELOG.md`, `AGENTS.md`, timing bounds,
  hardware matrix, hardening report, documentation index, 1.1.0 release notes,
  and this handoff.

Historical release notes and historical hardware artifact results were not
rewritten.

## Validation

| Command | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS |
| `python tools/check_public_timing_contract.py` | PASS |
| `python tools/check_cli_contract.py` | PASS |
| `python tools/check_idf_example_contract.py` | PASS |
| `python scripts/generate_version.py check` | PASS at version 1.1.0 |
| `python -m platformio test -e native` | PASS, 91/91 in 1.543 s on the final candidate |
| `python -m platformio run -e ex_bringup_s3` | PASS, 14.106 s |
| `python -m platformio run -e ex_bringup_s2` | PASS, 13.422 s |
| `git diff --check` | PASS; line-ending conversion warnings only |
| Pure ESP-IDF S3/S2 builds | NOT RUN; `idf.py` unavailable on `PATH` |

## Audit Closure

| Audit finding | Software disposition |
| --- | --- |
| P0-1 optional absent startup | Closed by strict/optional begin policy and retained startup evidence. |
| P0-2 offline latch | Closed by bus-silent normal operations and explicit recovery. |
| P0-3 identity/capability proof | Closed by atomic group/subgroup/CO2/capability validation. |
| P0-4 checked CO2 procedure | Closed by value-first/status-second checked result APIs. |
| P0-5 write/pointer completion | Closed by explicit completion deadlines and ordered pointer reads. |
| P0-6 cooperative long waits | Closed by optional task-context `delayMs`/`yield` slicing. |
| P1-1 safe limits/public WCET | Closed by centralized validation, operation bounds, exhaustive map, and source audit. |
| P1-2 persistent single-byte uncertainty | Closed by the unified mutation diagnostic, guard, and target resync. |
| P1-3 fault precision | Closed by `BUS_STUCK`, precise cleanup precedence, `VERIFY_MISMATCH`, semantic identity statuses, and mutation effects. |
| P1-4 optional-setting capability guards | Closed by validated cached support checks before typed reads or writes perform bus I/O. |
| P1-5 transfer/sample counter ambiguity | Closed in public documentation; no false sample counter was added. |
| P1-6 native fake coverage | Closed in software by 91/91 native tests covering lifecycle, timing, checked samples, mutation/resync, address, auto-adjust, and an exhaustive public bus failure matrix. |

Software closure does not close electrical qualification, consuming-firmware
composition, or immutable dependency gates.

## Remaining Firmware-Owned Work

- Publish and verify an immutable 1.1.0 dependency only after separate release
  authorization.
- Keep production E2/EE871 use exclusive to Co2Control.
- Ensure TunnelMonitor and every other non-Co2Control production profile
  excludes the dependency, sources, pins, runtime, data, health, settings, and
  commands.
- Choose/review E2 pins, pull-ups, level shifting, wiring, and board authority.
- Implement one externally serialized E2 owner with bounded queues/deadlines
  and application-owned retry/cadence policy.
- Admit only checked CO2 results to durable firmware data.

No firmware task, pin, queue, schema, storage, Cloud field, or product policy
was added to this library.

## Hardware, External Validation, And Release

No Prompt 04 HIL, physical sensor, waveform, calibration, bus-address,
auto-adjust, power-cycle, network, Cloud, or long-run validation was performed.
All new physical cases remain `NOT RUN` in the hardware matrix. Historical
ESP32-S3 evidence remains associated with its original 0.3.0/1.0.0-era
artifacts and is not recast as 1.1.0 evidence.

No tag, hosted release, or immutable remote dependency was created. Committing
and synchronizing the authorized series branch does not create that release.
Pure ESP-IDF local validation must be reported truthfully according to actual
`idf.py` availability.
