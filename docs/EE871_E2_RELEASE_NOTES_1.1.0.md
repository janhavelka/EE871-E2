# EE871-E2 Release Notes 1.1.0

Date prepared: 2026-07-28

Release status: source candidate; tag and hosted release not created

## Summary

EE871-E2 1.1.0 completes the reusable-library hardening sequence for precise
protocol timing, explicit lifecycle/recovery, checked CO2 samples, and
persistent/maintenance mutation truth. The library remains synchronous,
framework-neutral, fixed-size, and externally serialized. It does not own
GPIO, tasks, queues, pins, schemas, storage, retry cadence, or product policy.

This source is suitable for downstream firmware evaluation after an immutable
1.1.0 tag or release is separately authorized and published. A downstream
project must verify the immutable reference; this document is not proof that
one exists.

## Public Additions

Append-only status codes preserve values 0 through 14:

| `Err` | Value |
| --- | ---: |
| `VERIFY_MISMATCH` | 15 |
| `OFFLINE` | 16 |
| `CO2_SENSOR_ERROR` | 17 |
| `PERSISTENT_STATE_UNCERTAIN` | 18 |

Lifecycle and identity additions:

- `BeginPolicy`, `DeviceIdentity`, and `CapabilitySnapshot`;
- cache-only identity/capability accessors;
- explicit, bus-silent-offline lifecycle and raw `probe()` behavior;
- full identity/capability `recover()`.

Checked-sample additions:

- `Co2ValueKind`, `Co2SensorError`, and `Co2ReadResult`;
- `readCo2AverageSample()` and `readCo2FastSample()`;
- cache-only `hasCo2OffsetGain()` and `hasCo2AdjustmentPoints()`.

Mutation additions:

- `MutationTarget` values `NONE=0` through `CO2_GAIN=10`;
- `MutationEffect` values `NONE=0` through
  `OPERATOR_ACKNOWLEDGED=6`;
- `MutationDiagnostic`, `mutationDiagnostic()`, and
  `SettingsSnapshot::mutation`;
- cache-only `acknowledgeAutoAdjustUncertainty()`.

The source-compatible `persistentConfigDirty()` and
`persistentConfigDirtyError()` APIs remain. They now mirror the one mutation
diagnostic, including explicit maintenance uncertainty.

Operation timing kinds remain stable from 0 through 14 and append:

| `OperationKind` | Value |
| --- | ---: |
| `CUSTOM_BLOCK_WRITE_VERIFY` | 15 |
| `RESYNC_PERSISTENT_CONFIG` | 16 |
| `AUTO_ADJUST_MAINTENANCE` | 17 |
| `BUS_ADDRESS_CHANGE` | 18 |

## Mutation Evidence And Recovery

All effectful custom-memory APIs share one admission, frame-completion,
effect-classification, and fixed-size intent path.

| Effect | Meaning |
| --- | --- |
| `NONE` | Validation, lifecycle, capability, or uncertainty guard rejected before an effectful frame. |
| `NO_EFFECT` | Definite rejection or failure before a complete effectful request transferred. |
| `ACKNOWLEDGED` | Request was definitely accepted, but later STOP, completion, or verification did not prove final state. |
| `INDETERMINATE` | Complete PEC transferred but the final acceptance response was ambiguous. |
| `VERIFIED` | Target-specific readback or action-status evidence matched the request. |
| `RESYNCHRONIZED` | Later coherent readback established actual state but did not prove the original request took effect. |
| `OPERATOR_ACKNOWLEDGED` | The narrow auto-adjust procedure accepted an irreducibly ambiguous not-running observation. |

While `MutationDiagnostic::unresolved` is true, all further effectful APIs fail
before bus I/O with `PERSISTENT_STATE_UNCERTAIN`. Normal sampling, diagnostics,
probe, recovery, bus reset/inspection, cache access, and explicit
`resyncPersistentConfig()` remain available. The library never automatically
replays retained mutation intent.

`resyncPersistentConfig()` reconciles the exact target and records exact
requested, acknowledged, observed, and matched element counts. The no-target
path performs a capability-aware full coherence read and skips unsupported
settings.

Bus-address writes deliberately remain unresolved after acknowledgement.
Applications must end the session, perform their authorized physical power
procedure if required, configure the explicit candidate address, begin a new
session, and resync. The driver does not probe all addresses or guess
activation timing.

Auto-adjust is an explicit, non-cancellable, non-replayable maintenance action.
The driver observes status before and after writing. A later clear status
cannot prove whether an ambiguous request completed or never started; only the
target-specific cache-only acknowledgement can settle that historical case.

Unresolved evidence survives `end()`, failed/repeated `begin()`, and a later
successful `begin()` on the same object. Destroying the object or losing RAM
loses the evidence; applications that need restart persistence must own it
outside the library.

## Protocol, Lifecycle, And Sample Behavior

- E2 transfer waits use explicit bit, byte, write-completion, and interval-pair
  deadlines.
- START, STOP, final ACK, cleanup, NACK, PEC, and readback mismatch failures
  remain distinguishable.
- Optional startup accepts only definite clean absence. Ordinary operations
  are line-silent while offline, and only explicit recovery restores READY.
- Identity and all capability bytes publish atomically after complete
  validation.
- Raw MV3/MV4 reads remain unchanged.
- Checked sample procedures read value first and side-effecting status second,
  preserve every attempted/result field, and keep sensor-domain failures
  separate from transport health.
- Typed optional-setting reads and writes require validated cached
  capabilities before bus I/O, including calibration offset/gain.
- Raw custom writes cannot bypass typed pair, address, calibration,
  auto-adjust, or read-only-address safety.
- Multi-byte verification reads every requested target element before
  reporting the first mismatch. A sampled final ACK or NACK remains definite
  mutation evidence even if the remainder of that bounded phase times out.
- Signed custom-memory values use explicit two's-complement decoding, and
  typed mutation arguments are validated before optional-capability checks.
- The release/version tool is product-neutral and contains no
  TunnelMonitor-specific dependency generation.

## Timing And Examples

The timing calculator covers operation kinds 0 through 18. The exhaustive
public-method/formula map is
[`EE871_E2_OPERATION_TIMING_BOUNDS.md`](EE871_E2_OPERATION_TIMING_BOUNDS.md).
`tools/check_public_timing_contract.py` verifies every public callable is
classified as BUS or `NO_E2_IO`, every bus method is documented, and every
operation kind is implemented.

Arduino and native ESP-IDF bring-up CLIs retain raw `co2avg`, `co2fast`, and
`status`, and add checked `sampleavg` and `samplefast`. Checked output preserves
step validity and detailed error evidence. Help text states that status and
checked procedures may trigger the next measurement. Example-only adapters
provide task-context `delayMs`/`yield`; the core remains framework-neutral.

## Compatibility

- Existing public enum numeric values are unchanged.
- Existing raw measurement signatures and behavior are unchanged.
- Existing dirty accessors remain source compatible.
- `SettingsSnapshot` gained lifecycle, identity/capability, and mutation data.
  ABI/layout-sensitive consumers must rebuild and must not persist or exchange
  its raw object layout.
- New appended error codes and APIs make this a backward-compatible minor
  release at source level.
- No compatibility shim, parallel mutation state system, or product-specific
  integration was added.

## Validation Evidence

The release candidate requires the following software checks on the final
1.1.0 source state:

```text
python tools/check_core_timing_guard.py
python tools/check_public_timing_contract.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

The exact final commands, test count, and results belong in
[`reports/ee871_prompt_04_release_handoff_20260728.md`](reports/ee871_prompt_04_release_handoff_20260728.md).
Pure ESP-IDF builds must be reported separately if `idf.py` is available.

At the final-candidate checkpoint, the core/public timing, CLI, IDF example,
version-metadata, and diff checks pass, and the native suite passes 91/91.
Arduino ESP32-S3 and ESP32-S2 PlatformIO builds also pass. Exact command timing
is retained in the handoff. Local pure ESP-IDF builds were not run because
`idf.py` was unavailable on `PATH`.

No 1.1.0 HIL, physical-sensor, waveform, calibration, address-change,
auto-adjust, network, or long-run validation is claimed by these release notes.
Historical 0.3.0/1.0.0 ESP32-S3 evidence remains recorded with its original
firmware version and provenance in the hardware matrix.

## Known Limitations

- ESP32-S2 and native ESP-IDF physical HIL remain unrecorded.
- Physical stuck-line/fault-jig and power-cycle persistence coverage remains
  incomplete.
- Bus-address activation timing is deliberately not guessed; the application
  owns the authorized power and candidate-address procedure.
- Auto-adjust completion history can remain inherently ambiguous after a clear
  status.
- Mutation evidence is RAM-local to the driver object.
- Warm-up, freshness, cadence, plausibility, retry, and product health policy
  remain application-owned.

## Release Checklist

- [x] Set `library.json` to `1.1.0`.
- [x] Run `python scripts/generate_version.py sync` and verify
  `Version.h`, `idf_component.yml`, and `Doxyfile`.
- [x] Complete every available software validation command above on the final
  source; record unavailable `idf.py` honestly.
- [x] Review the hardware matrix; leave unexecuted physical cases `NOT RUN`.
- [x] Review the Prompt 04 handoff and P0/P1 closure audit.
- [x] Confirm the working tree and intended release commit.
- [x] Commit and push the source candidate on the already authorized series branch.
- [ ] Obtain separate authorization to create/publish a tag or hosted release.
- [ ] Create and publish immutable tag `v1.1.0`.
- [ ] Verify downstream firmware pins that immutable tag or commit.

The checklist remains intentionally incomplete until the separately authorized
immutable tag, hosted release, and downstream pin verification occur.
