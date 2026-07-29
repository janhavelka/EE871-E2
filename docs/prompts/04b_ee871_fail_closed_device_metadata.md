# Prompt 04B: EE871 Fail-Closed Metadata and Persisted-Value Correction

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `EE871-E2`. Prompt 04A must be complete and passing.

This prompt fixes confirmed lifecycle, capability, and persisted-value defects
before HIL. Keep the library general-purpose,
synchronous, framework-neutral, and externally serialized. Do not add owner
tasks, queues, pins, retry cadence, warm-up policy, schemas, logging, or
product-specific behavior.

Prefer shared private validators and the existing mutation owner. Do not add a
second capability cache, persistent state machine, compatibility shim, or
generic register framework.

## Authoritative Corrections

This prompt supersedes Prompt 02's rule that a clean identity `NACK` is
definite absence. A present E2 slave may NACK during measurement when
measurement priority is active. A clean STOP proves transaction cleanup, not
absence.

Preserve public enum numeric values and source-compatible signatures where
possible, but remove the incorrect behavior. Do not retain a second legacy
path.

## Read Before Editing

Read completely:

- `AGENTS.md`;
- `docs/prompts/README.md`;
- Prompts 02-04A and their handoffs;
- all public headers under `include/EE871/`;
- `src/EE871.cpp`;
- all native fakes/tests;
- Arduino and ESP-IDF diagnostic CLI implementations and contract checks;
- `README.md`, `CHANGELOG.md`, current release notes and validation matrices;
- `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/pdf-extracted-md/E2_interface_specification_v4_1.md`;
- `docs/pdf-extracted-md/E2_interface_utilising_AN0105.md`;
- `docs/pdf-extracted-md/EE871_E2_interface_addendum.md`;
- `docs/pdf-extracted-md/EE871_E2_CO2_interface_AN1611-1.md`.

Inspect the working tree and record branch, commit, version, dirty state, and
baseline validation. Preserve unrelated work.

## Objective

Implement one coherent fail-closed correction:

1. never convert a transient measurement-time NACK into definite absence;
2. reject reserved capability bits before publishing any capability;
3. reuse small typed-value validators for reads, writes, and resync;
4. reject undefined interval-factor zero;
5. preserve precise status, cache atomicity, recovery, and mutation evidence.

## Lifecycle: NACK Is Not Definite Absence

Keep `BeginPolicy` and its numeric values.

`BeginPolicy::REQUIRE_PRESENT` remains strict.

`BeginPolicy::ALLOW_ABSENT` may accept only a genuinely definite
`Err::DEVICE_NOT_FOUND` supplied by an authoritative presence mechanism. The
current GPIO E2 transaction path must not translate control-byte `NACK` into
`DEVICE_NOT_FOUND`.

For both begin policies, identity `NACK` must:

- return `Err::NACK` precisely;
- leave the driver `UNINIT`;
- publish no identity or capabilities;
- preserve normal cleanup evidence;
- perform no hidden retry or long wait.

The application/firmware owner decides retry cadence and whether repeated
NACKs mean operational absence. Do not add retry counts, sleeps, probe loops,
or an `ABSENT` state to the library.

If no current core path can produce definite `DEVICE_NOT_FOUND`, retain the
policy contract for source compatibility but document that current GPIO E2
absence normally appears as NACK and therefore is not accepted by one-shot
`begin()`. Do not add a test hook or new presence abstraction only to
manufacture `DEVICE_NOT_FOUND`.

This deliberately changes the practical `ALLOW_ABSENT` behavior: on the
current GPIO transport, ordinary physical absence normally leaves the driver
`UNINIT` with `NACK` instead of initialized `OFFLINE`. State this compatibility
consequence plainly. The owner must retry `begin()`; `recover()` is not a
substitute for initialization.

Update downstream prompt/guideline text that currently assumes an accepted
NACK creates an initialized offline module. It must instead keep the module
uninitialized and let its existing owner retry policy call `begin()` later.
Do not implement that downstream firmware in this repository.

## Atomic Capability Validation

Validate the existing contiguous capability bytes `0x03..0x09` as local
candidates before setting `CapabilitySnapshot::valid` or publishing any field.
Do not add another capability cache.

Custom address `0x02` reports the E2 specification version used during product
development. The supplied documents do not define a compatibility table that
proves every value other than 4 is incompatible with the documented
`0x03..0x09` layout. Therefore:

- keep the existing `readE2SpecVersion()` as explicit diagnostic evidence;
- do not add the version to `CapabilitySnapshot`;
- do not reject `begin()`, `probe()`, or `recover()` solely because the
  version is not 4;
- do not invent forward/backward compatibility policy.

If accepted project decisions or additional authoritative device
documentation explicitly establish a version gate before implementation,
report the exact conflict and stop this portion rather than silently widening
the prompt.

Require:

```text
0x03 reserved-zero mask: 0xF0
0x04 reserved-zero mask: 0xF0
0x05 reserved-zero mask: 0xFE
0x06 reserved-zero mask: 0xF0
0x07 reserved-zero mask: 0x08
0x08 reserved-zero mask: 0xFC
0x09 reserved-zero mask: 0xFE
```

Put named `static constexpr` masks in `CommandTable.h` and use one private
candidate validator. Equivalent naming is allowed.

Return `Err::NOT_SUPPORTED` for:

- any reserved capability bit set.

Do not blanket-reject raw byte `0x55`: for example, `0x07 == 0x55` is a valid
combination under its defined mask. Address-specific reserved-bit masks, not a
value-only sentinel heuristic, own validity. `0xFF` is naturally rejected by
every listed reserved mask.

Use a deterministic detail value. For reserved-bit failure, encode or
otherwise report both the custom address and raw byte without heap strings.
Document the encoding if used.

Do not use `DEVICE_NOT_FOUND` for a responding incompatible device. Do not
guess capability meaning beyond the documented bits.

All begin/recover candidates remain atomic:

- no partial live cache;
- no capability helper returns true before the complete validated snapshot;
- recovery from an offline session remains latched offline on incompatibility;
- semantic identity/capability incompatibility during recovery clears live
  claims and latches `OFFLINE` even if recovery was entered from READY or
  DEGRADED;
- diagnostic probe remains identity-only and cache/health neutral.

Apply the same address-specific validation to direct typed capability reads:
`readOperatingFunctions()`, `readOperatingModeSupport()`, and
`readSpecialFeatures()`. Assign caller outputs only after validation.

## Shared Typed-Value Validation

Create small private pure validators, with equivalent names allowed:

```cpp
Status _validateBusAddressValue(uint8_t value) const;
Status _validateIntervalValue(uint16_t value) const;
Status _validateIntervalFactorValue(int8_t value) const;
Status _validateOperatingModeValue(uint8_t value) const;
Status _validateAutoAdjustRaw(uint8_t value) const;
```

Reuse them from typed reads, typed writes, unresolved mutation resync, and full
persistent coherence reads. Do not duplicate ranges in four switches.

Preserve the existing error-precedence and mutation ownership for every typed
write:

1. run the common initialized/online/unresolved admission guard;
2. validate the requested value;
3. call `_beginMutation()`;
4. perform E2 I/O.

Thus an uninitialized, offline, or already-unresolved driver keeps its existing
precondition status. On a ready, resolved driver, invalid input is rejected
before mutation state or line activity.

Validation of a successfully transferred but semantically invalid stored value
is not a transport failure. Return the semantic status from the typed
procedure, but do not pass it through `_updateHealth()` or increment transport
failure counters. The underlying successful E2 transfer remains the tracked
transport event.

Required behavior:

### Bus address

- `0..7` is valid.
- A typed read that observes `>7` leaves its output unchanged and returns
  `OUT_OF_RANGE` with the observed byte in `Status::detail`.
- A write rejects `>7` before bus I/O.

### Global interval

- `150..36000` deciseconds is valid.
- A typed read leaves its output unchanged and returns `OUT_OF_RANGE` with the
  observed word in `Status::detail` when invalid.
- After the common lifecycle/offline/unresolved admission guard succeeds, a
  write rejects invalid values before `_beginMutation()` and before E2 I/O.

### CO2 interval factor

- positive values are multipliers;
- negative values are divisors;
- zero is undefined by the supplied E2 specification and must return
  `OUT_OF_RANGE` before `_beginMutation()` or E2 I/O after the common admission
  guard succeeds;
- a typed read leaves its output unchanged and returns `OUT_OF_RANGE` with
  zero in `Status::detail`.

Do not invent narrower limits for nonzero `int8_t` values.

### Operating mode

- only bits 0 and 1 are defined;
- reserved bits produce `OUT_OF_RANGE`;
- a set defined bit that the validated capability snapshot does not advertise
  produces `NOT_SUPPORTED`;
- a typed read leaves its output unchanged and reports the observed byte in
  `Status::detail` on semantic error;
- reads/writes remain capability-gated before I/O where the whole feature is
  absent.

### Auto-adjust

- only bit 0 is defined at `0xD9`;
- any reserved bit produces `OUT_OF_RANGE`;
- `readAutoAdjustStatus()` does not report a corrupt value as idle;
- `startAutoAdjust()` validates the pre-read byte and performs no write if
  reserved bits are set;
- `startAutoAdjust()` validates the post-write observation before declaring
  the action verified;
- if an acknowledged post-write observation has reserved bits, preserve the
  raw value in `Status::detail`, return `OUT_OF_RANGE`, and keep the mutation
  acknowledged and unresolved;
- full and unresolved auto-adjust resync validate every D9 observation before
  resolving uncertainty;
- bit 0 already set remains `BUSY`;
- the existing non-cancellable uncertainty procedure is otherwise unchanged.

For the `bool` status output, do not overwrite the caller's output on invalid
raw data. Raw forensic evidence remains available through `customRead(0xD9)`.

For any semantic-invalid observation during unresolved resync, return the
precise semantic error, retain the original mutation cause, keep
`unresolved=true`, and do not set `RESYNCHRONIZED`.

### CO2 filter

The supplied generic E2 specification says filter values are product-specific,
and the repository does not contain an authoritative EE871 numeric table.
Do not invent a valid range in the core library. Keep the typed byte API
capability-gated and document the value as opaque/vendor-defined. Prompt 04C
will disable automatic filter-write HIL until authoritative values exist.

## Existing Raw Custom-Write Boundary

Do not broaden this corrective prompt into removing the explicit
`customWrite()` expert API or its `RAW_CUSTOM_BYTE` diagnostic. It is an
intentional, named, effectful API with verification and uncertainty tracking,
not a normal sampling path.

Retain Prompt 04's protected-address dispatch. Ensure specifically that
`customWrite(0xCB, 0)` routes through the corrected typed factor validator and
is rejected before I/O. Do not create any new raw-write use in examples or HIL.

Update Doxygen and CLI help to state plainly that an otherwise-unclassified
raw custom write is an expert maintenance operation whose address semantics
must come from authoritative device documentation. The HIL runner must never
replay the 256-byte baseline or use raw writes for restoration. Do not invent
an unsafe flag, magic token, or second raw API.

## Native Tests

Add or repair tests proving:

1. `ALLOW_ABSENT` no longer accepts a clean identity NACK;
2. strict and optional begin both preserve precise measurement-time NACK and
   remain `UNINIT`;
3. definite `DEVICE_NOT_FOUND`, if an authoritative injectable path exists,
   remains the only accepted optional absence;
4. no hidden retry/delay occurs on NACK;
5. every reserved mask bit at `0x03..0x09` fails atomically;
6. reserved-bit-invalid capability patterns cannot enable a typed write, while
   mask-valid `0x07 == 0x55` remains accepted;
7. direct typed capability reads apply the same masks and leave outputs
   unchanged on failure;
8. begin/recover failure publishes no partial capability cache;
9. diagnostic E2-version reads remain available without becoming lifecycle
   compatibility policy;
10. invalid observed address, interval, factor zero, mode, and D9 reserved bits
    return the required precise error;
11. typed reads leave outputs unchanged and preserve invalid raw evidence in
    `Status::detail`;
12. on an initialized, ready, resolved driver, each typed write rejects invalid
    input before `_beginMutation()` and line I/O while existing lifecycle and
    uncertainty precedence remains unchanged;
13. auto-adjust reserved pre-state performs zero write frames;
14. read/write/post-write/resync use the same validators;
15. semantic invalidity after successful transfers does not increment
    transport failures or degrade a transport-healthy driver;
16. semantic-invalid unresolved resync preserves the original cause, keeps
    `unresolved=true`, and does not report `RESYNCHRONIZED`;
17. every documented typed dispatch address reaches exactly one typed owner;
18. no raw custom mutation can bypass pair, calibration, address, mode, factor,
    or auto-adjust rules;
19. existing enum values and public-header source compatibility remain stable;
20. existing Arduino and ESP-IDF CLI parity/help checks remain passing without
    adding new commands.

Tests must exercise production paths. Do not reproduce the validators inside
the fake.

## Documentation and Follow-On Prompt Consistency

Update:

- public Doxygen;
- `README.md`;
- `CHANGELOG.md` under `Unreleased`;
- protocol/register-map and timing-bound docs;
- current active hardening guidance;
- examples and CLI help;
- Prompt 02's active NACK/absence guidance;
- downstream Prompts 07 and 08 wherever they assume `ALLOW_ABSENT` accepts
  NACK or initializes an offline driver.

Document:

- NACK is transient/ambiguous and application retry policy owns absence;
- the practical `ALLOW_ABSENT` compatibility consequence;
- capability reserved masks and atomic publication;
- E2 version is diagnostic evidence, not an invented compatibility gate;
- typed reads fail closed while preserving stated raw evidence;
- filter values remain opaque and are not automatically qualified;
- raw custom writes remain explicit expert maintenance operations and are
  excluded from automated HIL restoration.

Do not claim previously recorded HIL evidence covered these corrections.
Do not rewrite versioned historical release notes or earlier handoffs. Add the
new correction to `CHANGELOG.md` under `Unreleased` and a new handoff.

Do not edit generated `Version.h`, tag, or release unless explicitly
authorized. Record any required patch-release decision.

## Validation

Run at least:

```text
python -m platformio test -e native
python -m platformio run -e ex_bringup_s2
python -m platformio run -e ex_bringup_s3
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/check_core_timing_guard.py
python tools/check_public_timing_contract.py
```

Run the current documented ESP-IDF build if its toolchain is available.
Otherwise report the exact limitation. Do not run live HIL.

## Handoff

Create a report under `docs/reports/` containing:

- baseline/final state;
- superseded decisions removed;
- lifecycle and capability validation tables;
- persisted-value validation table;
- corrected typed validation and protected `customWrite()` dispatch table;
- public compatibility impact;
- tests/builds actually run;
- intentional filter-write limitation;
- remaining HIL.

## Acceptance Criteria

- A measurement-time NACK can never become accepted absence.
- Reserved capability data cannot publish or enable maintenance APIs.
- Capability publication remains atomic.
- Invalid persisted values fail closed with precise status.
- Interval-factor zero cannot be written.
- Corrupt D9 reserved bits cannot be reported idle or followed by a write.
- Existing raw writes remain explicit expert maintenance operations and are
  never used by automated HIL restoration.
- All supported mutations retain one mutation/effect owner.
- No retry cadence, firmware policy, async mechanism, or hidden recovery was
  added.
- Native tests, contract checks, and both ESP32 example builds pass.
- No HIL result is claimed.
