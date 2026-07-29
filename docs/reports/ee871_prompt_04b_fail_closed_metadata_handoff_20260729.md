# EE871 Prompt 04B Fail-Closed Metadata Handoff

Date: 2026-07-29

## Baseline And Scope

- Repository: `EE871-E2`
- Branch: `feature/ee871-hardening-series`
- Baseline commit: `1bffb9ef4565ab8a848c845088fc5e37c65c5faf`
- Manifest version: `1.1.0`
- Starting state included the uncommitted Prompt 04A completion-budget
  correction and its documentation. Those changes were preserved.
- Scope remained the synchronous, framework-neutral library, native fake,
  diagnostic examples, contract checks, and current documentation. No firmware
  tasks, queues, pins, schemas, or product policy were added.

## Superseded Lifecycle Decision

Prompt 02's clean-NACK absence shortcut was removed. The E2 specification
allows a present slave to NACK when measurement has priority, so STOP cleanup
cannot convert NACK into physical-absence evidence.

| Begin result | `REQUIRE_PRESENT` | `ALLOW_ABSENT` |
| --- | --- | --- |
| Compatible identity and capabilities | `READY` | `READY` |
| E2 NACK, including clean cleanup | Preserve `NACK`; `UNINIT` | Preserve `NACK`; `UNINIT` |
| Authoritative `DEVICE_NOT_FOUND` | Reject; `UNINIT` | May initialize latched `OFFLINE` |
| Transport/protocol failure | Preserve exact error; `UNINIT` | Preserve exact error; `UNINIT` |
| Semantic identity/capability incompatibility | `NOT_SUPPORTED`; `UNINIT` | `NOT_SUPPORTED`; `UNINIT` |

The current GPIO E2 path has no authoritative `DEVICE_NOT_FOUND` source.
Native tests therefore prove the practical behavior: both policies preserve
NACK, perform one identity attempt, issue no capability pointer write, perform
no long completion delay, publish no cache, and leave health counters at zero.

## Capability Validation

The seven-byte capability read remains one atomic candidate. One shared
validator applies these reserved-zero masks before publication:

| Address | Meaning | Reserved-zero mask |
| --- | --- | --- |
| `0x03` | Custom adjustment support | `0xF0` |
| `0x04` | Adjustment-point support | `0xF0` |
| `0x05` | General adjustment-time support | `0xFE` |
| `0x06` | Per-measurement adjustment-time support | `0xF0` |
| `0x07` | Operating functions | `0x08` |
| `0x08` | Operating-mode support | `0xFC` |
| `0x09` | Special features | `0xFE` |

Reserved-bit failure returns `NOT_SUPPORTED` with
`detail = (address << 8) | raw`. It never blanket-rejects `0x55`;
`0x07 == 0x55` is valid. Direct typed reads of `0x07..0x09` use the same
validator and leave caller output unchanged on failure.

Recovery from READY, DEGRADED, or OFFLINE now treats semantic identity or
capability incompatibility consistently: live identity/capability claims are
cleared and the initialized driver is latched OFFLINE. Successful underlying
tracked transfers remain transport successes; semantic rejection does not
increment transport failures. `probe()` stays identity-only, raw,
health-neutral, and cache-neutral. E2 version byte `0x02` stays diagnostic and
does not gate lifecycle.

## Persisted-Value Validation

Small private validators are reused by typed reads, typed write admission,
post-write observation, unresolved-target resync, and full supported-state
resync:

| Target | Accepted value | Invalid result |
| --- | --- | --- |
| Bus address `0xC0` | `0..7` | `OUT_OF_RANGE`, raw detail |
| Global interval `0xC6/0xC7` | `150..36000` deciseconds | `OUT_OF_RANGE`, assembled detail |
| CO2 factor `0xCB` | `-128..-1`, `1..127` | Zero is `OUT_OF_RANGE` |
| Operating mode `0xD8` | bits 0/1 only, and every set bit advertised by `0x08` | Reserved bit: `OUT_OF_RANGE`; unadvertised defined bit: `NOT_SUPPORTED` |
| Auto-adjust `0xD9` | bit0 only | Reserved bit: `OUT_OF_RANGE` |
| CO2 filter `0xD3` | Opaque byte | No invented range |

Typed reads publish only after validation and otherwise retain caller output.
Semantic-invalid post-write observations remain acknowledged/unresolved,
preserve the first mutation cause, and retain raw observation evidence.
Semantic-invalid resync cannot report `RESYNCHRONIZED`.

Mutation admission precedence is:

1. initialization/offline/unresolved guard;
2. supplied value validation;
3. mutation intent creation;
4. E2 I/O.

This keeps lifecycle and uncertainty errors authoritative while ensuring
invalid values on a ready/resolved driver are bus-silent and create no
mutation diagnostic.

## Protected `customWrite()` Dispatch

| Address | Owner |
| --- | --- |
| `0xC0` | Typed bus-address procedure |
| `0xC6/0xC7` | Reject; paired interval API required |
| `0xCB` | Typed signed-factor procedure; zero rejected |
| `0xD3` | Typed opaque-filter procedure |
| `0xD8` | Typed mode procedure |
| `0xD9` | Value 1 uses typed auto-adjust; other values reject |
| `0x58..0x5B` | Reject; paired calibration API required |
| Documented read-only identity/capability/serial/error/pointer bytes | Reject |
| Other writable bytes | Expert `RAW_CUSTOM_BYTE` procedure |

The expert fallback remains explicit and verified; it was not broadened.
Doxygen and both diagnostic CLI help surfaces state that authoritative vendor
address/restoration semantics are required. Automated HIL restoration remains
typed and never replays the 256-byte forensic image.

## Public Compatibility

- Existing enum numeric values and public method signatures are unchanged.
- `BeginPolicy::ALLOW_ABSENT` retains numeric value 1 but has the corrected
  practical NACK behavior.
- New command-table masks and
  `cmd::makeCapabilityValidationDetail()` are additive.
- No version metadata or generated `Version.h` was changed.

If `v1.1.0` has already been published externally, these behavioral fixes
require an authorized patch release rather than rewriting that tag. If it has
not been published, release/version selection still requires explicit
authorization. At report drafting, the implementation had not been committed
or pushed; an authorized series commit may follow. No tag, release, publication,
or manifest bump is implied by that repository sync.

## Validation Performed

- `python -m platformio test -e native`: PASS, 104/104.
- `python -m platformio run -e ex_bringup_s2`: PASS.
- `python -m platformio run -e ex_bringup_s3`: PASS.
- `python tools/check_cli_contract.py`: PASS.
- `python tools/check_idf_example_contract.py`: PASS.
- `python tools/check_core_timing_guard.py`: PASS.
- `python tools/check_public_timing_contract.py`: PASS.
- `python scripts/generate_version.py check`: PASS.
- `doxygen Doxyfile`: PASS.

`idf.py` was not available on `PATH`, so a native ESP-IDF build was not run.
No HIL, physical sensor, waveform, power-cycle, calibration, address,
auto-adjust, stuck-line, network, or long-run result is claimed.

## Post-Implementation Audit

The complete Prompt 04B diff was audited again before progression. That audit:

- corrected the public `DEVICE_NOT_FOUND` enum documentation so it cannot be
  read as a synonym for ordinary E2 non-response/NACK;
- replaced the last active lifecycle-guideline reference to a "cleanly
  terminated absence" with authoritative accepted `DEVICE_NOT_FOUND`; and
- added explicit production-path coverage proving
  `customWrite(0xD8, reservedBits)` reaches the typed operating-mode validator
  and remains bus-silent;
- added explicit OFFLINE-invalid-input precedence and two-byte interval
  post-write semantic-validation coverage;
- made the public D3 and both CLI help contracts explicit that filter values
  are opaque/vendor-defined and that successful equality verification is not
  semantic qualification; and
- removed a later README statement that incorrectly described protected
  `reg write` dispatch as arbitrary custom-memory access.

## Remaining Work

Prompt 04C remains the owner of HIL runner preflight/evidence correction and
must be completed before live HIL. The corrected library behavior itself has
native and firmware-build coverage; physical qualification remains pending.
