# Prompt 04C: EE871 HIL Runner Preflight and Evidence Correction

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `EE871-E2`. Prompts 04A and 04B and their corrective handoffs must
be complete and passing.

This prompt corrects confirmed false-pass, false-review, destructive-preflight,
fault-evidence, and artifact-integrity defects in the repository HIL runner.
Do not redesign the core library, invent hardware results, add firmware tasks,
or broaden the diagnostic CLI unless a real assertion cannot consume evidence
already emitted.

Keep quick/default and complete-safe plans non-destructive. Persistent,
calibration, address, auto-adjust, power-cycle, unplug/replug, and stuck-line
plans remain separate explicit opt-ins.

Prefer a few direct validators and state invariants over a rule engine,
workflow DSL, replay system, or generic test framework.

## Read Before Editing

Read completely:

- `AGENTS.md`;
- `docs/prompts/README.md`;
- Prompts 03, 04, 04A, and 04B and their handoffs;
- `tools/ee871_hil_runner.py`;
- `test/test_hil_runner_parser.py`;
- `docs/EE871_E2_HIL_RUNNER.md`;
- checked-sample, capability, health, timing, and mutation contracts in the
  public headers;
- Arduino and ESP-IDF diagnostic CLI implementations, especially:
  `samplefast`, `sampleavg`, `features`, `caps`, `dirty`, `drv`, `levels`,
  `buscheck`, `libreset`, `reg dump`, and all typed maintenance commands;
- current CLI/parity/source-audit tools;
- the local E2/EE871 datasheet sections governing D9, capability bits,
  interval factor, filter, and held-low behavior.

Record branch, commit, version, dirty state, parser baseline, CLI contract
baseline, and firmware build baseline. Preserve unrelated changes.

## Objective

Reuse the existing `CommandSpec`, parser, state, checkpoint, and summary model
to fix all confirmed runner defects:

1. an accidental D9 auto-adjust action must never be ignored;
2. definite command failures must be `FAIL`, not operator review;
3. no destructive command may pass incomplete/inconsistent preflight;
4. checked-sample validation must honor the detailed-error capability;
5. stuck-line evidence must prove the selected fault and real health change;
6. baseline timestamps and values must remain immutable;
7. hazardous metadata must be meaningful;
8. unsupported filter writes and undefined factor zero must not be offered.

Do not add automatic resync, recovery, retry, restoration replay, address scan,
or write-anything-from-baseline behavior.

## 1. D9 Is an Action Register, Not Generally Volatile

Remove `0xD9` from the global custom-memory volatile/read-dependent set.

The general comparison exceptions remain only the documented values whose
reads may legitimately move independently of the selected mutation:

```python
CUSTOM_MEMORY_VOLATILE_ADDRESSES = frozenset({0xC1, 0xFE, 0xFF})
```

An unexpected D9 change during interval, factor, mode, part-name, calibration,
address, or final-restoration comparison is a hard failure.

Allow D9 only when the explicitly authorized selected target is
`AUTO_ADJUST` at `0xD9..0xD9`. Use the existing target allowlist passed to the
one `custom_memory_diff()` helper. Do not add command-name exceptions
throughout the runner.

The dedicated auto-adjust plan remains:

- one-shot;
- non-replayable;
- non-cancellable;
- non-restorable;
- forensic after the action.

Never restore D9 from the recorded baseline.

## 2. Definitive Failures Take Precedence Over Missing Success Text

Refactor `classify_response()` once so a parsed non-OK status or validator
failure cannot be hidden by a missing success-only output token.

Use this order:

1. operator-only step;
2. command timeout;
3. empty response;
4. semantic validators;
5. expected success token;
6. unresolved parser/review notes;
7. pass.

Equivalent ordering is acceptable if it preserves these results:

- `Status: TIMEOUT`, `NACK`, `BUS_STUCK`, `OFFLINE`, or another rejected
  status is `FAIL` when success was required, even if the normal value/heading
  was not printed;
- any parsed non-OK status is `FAIL` even when a `CommandSpec` has no semantic
  status validator, except for a named intentional non-OK contract;
- the existing intentional non-OK contracts remain valid:
  - optional `NOT_SUPPORTED`;
  - address `PERSISTENT_STATE_UNCERTAIN`;
  - allowed auto-adjust-start uncertainty;
  - expected unplug failure;
  - exact stuck-line `BUS_STUCK`;
- missing/unparseable evidence with no definite failure may remain
  `OPERATOR_REVIEW_REQUIRED`.

Do not special-case every command separately.

## 3. Complete Destructive Baseline Admission

Before sending any destructive command, require:

- a successful complete baseline group;
- an immutable complete 256-byte custom-memory snapshot;
- fresh `dirty` evidence after the latest mutation epoch;
- `persistentConfigDirty == false`;
- `resyncNeeded == false`;
- `mutation.unresolved == false` explicitly;
- a successfully parsed typed restore baseline for the selected target;
- recorded persistent bus address equal to operator-supplied
  `--device-address`;
- capability-aware auto-adjust idle evidence.

Store the command-line address in a dedicated immutable state key such as:

```python
configured_device_address
```

Do not reuse `expected_device_address`, candidate address, or another state key
that changes after a destructive address command. Compare the semantic
baseline `addr` result with the configured value and latch baseline failure on
mismatch.

Auto-adjust preflight is capability-aware. Use the runner's actual parsed
`special_features` state key and documented auto-adjust capability bit
`0x01`:

- if `special_features & 0x01` advertises auto-adjust, `autoadj` must parse and report
  idle;
- if auto-adjust is not advertised, a clean `NOT_SUPPORTED` result satisfies
  this check;
- if running, missing, corrupt, or contradictory, block every unrelated
  destructive plan.

Enforce these conditions in both baseline validators and the final
pre-transmission admission guard. A stale/missing validator must not allow a
write.

Never invoke `resync`, acknowledge uncertainty, or clear state automatically
to make preflight pass.

Add read-only firmware/E2 version evidence to the maintenance baseline if it
is not already guaranteed by the preceding safe plan. Parse and record the
public `readE2SpecVersion()`/CLI `e2spec` result. The version is diagnostic
evidence only: do not add a cached capability field, runner compatibility
table, or lifecycle gate.

For each explicitly authorized restorable persistence row, retain the existing
simple sequence and make its evidence complete:

1. before any write, record the complete 256-byte custom-memory image plus all
   supported typed values: part name, bus address, global interval, CO2
   interval factor, CO2 filter, operating mode, CO2 offset, CO2 gain,
   calibration points, and auto-adjust state;
2. record a clean mutation diagnostic and the exact typed baseline for the
   selected target;
3. issue the selected test value through its typed command only;
4. verify typed readback, exact mutation evidence, and a complete post-test
   memory diff limited to the selected target plus documented volatile bytes;
5. only while every preceding result is successful, resolved, and
   internally consistent, restore the exact recorded typed baseline through
   the same typed owner;
6. verify restored typed readback, clean/resolved mutation evidence, and a
   final complete memory comparison with the original image.

Record an explicit `NOT_SUPPORTED` result for unavailable typed values rather
than inventing a default. Never restore from raw dump bytes. If the test write,
readback, mutation evidence, or diff fails or becomes uncertain, stop the
automatic sequence, preserve all evidence, and require operator diagnosis;
do not send a “best effort” restoration write. This is the safe answer to a
failed write because the runner cannot assume which address actually changed.

Address change remains its existing explicit candidate/activation/rebegin/
resync workflow and requires a separately authorized restoration workflow; it
must never scan. Calibration uses typed offset/gain owners and dedicated
authority. Auto-adjust remains intentionally non-restorable. Do not turn the
full dump into a replay engine.

## 4. Checked Samples With and Without Detailed Error-Code Capability

Ensure `features` and `caps` are captured before checked-sample validation in
the complete-safe plan. Use cached runner state rather than re-reading features
inside each validator.

For a clean CO2 status, retain the current requirements:

- value and status reads attempted and successful;
- valid ppm/status evidence;
- no detailed error read;
- no sensor error;
- top-level `OK`.

For status bit 3 set and error-code capability advertised, require:

- detailed error read attempted and successful;
- valid raw code;
- correct enum mapping for codes 1, 200, 201, 202 or `UNKNOWN`;
- top-level `CO2_SENSOR_ERROR`;
- top-level detail equal to the detailed code.

For status bit 3 set and error-code capability not advertised, the structurally
correct library evidence is:

- detailed error read not attempted;
- detailed validity false and no raw detail code;
- sensor error `UNKNOWN` with enum value 255;
- top-level `CO2_SENSOR_ERROR`;
- top-level detail equal to the status byte.

A coherent sensor-domain error still fails a healthy-bench HIL row. Report one
truthful sensor-fault reason; do not mislabel it as a parser/transport failure.
Keep the pre/post `drv` evidence proving the checked procedure did not invent a
transport failure.

## 5. Precise Stuck-Line Evidence

For the applied fixture:

- SDA-low requires SDA low and SCL high;
- SCL-low requires SCL low and SDA high.

Both low is not the selected fault and must fail.

With an already-held-low selected line, require exact `BUS_STUCK` from:

- `buscheck`;
- tracked `status`;
- `libreset`.

Do not accept generic `TIMEOUT` for these rows. A later waveform-level timing
test is separate.

For each SDA-low and SCL-low plan, use this order:

1. released `levels`;
2. pre-fault `drv`;
3. operator applies the reviewed open-drain/current-limited jig;
4. in-fault `levels`;
5. raw `buscheck`;
6. tracked failing `status`;
7. in-fault `drv` while the jig is still applied;
8. raw `libreset`;
9. operator releases the jig;
10. released `levels`;
11. explicit `recover`;
12. final `drv`.

The in-fault health validator must compare with the pre-fault snapshot and
require:

- total transport failures increased;
- consecutive failures is at least one;
- state/online invariants are coherent:
  - `DEGRADED` implies online;
  - `OFFLINE` implies not online;
  - `READY` or `UNINIT` is invalid for the in-fault row.

Do not assume one fixed offline threshold if the runner does not capture it.
Raw `buscheck` and `libreset` remain health-neutral; do not change core health
ownership in this prompt.

After release/recovery require READY, online, and zero consecutive failures.
Operator steps remain review-required, and waveform proof still requires
external equipment.

## 6. Mutation-Diagnostic Consistency

Strengthen the existing clean-state validator so these fields must agree:

```text
persistentConfigDirty == false
resyncNeeded == false
mutation.unresolved == false
```

Require the mutation fields to parse, not merely exist. Reject contradictory
combinations as `FAIL`. The final write-admission guard must independently
require the explicit false values.

Where the CLI emits dirty-error/cause fields, validate the normal resolved
contract without inventing history:

- legacy dirty error is `OK` when unresolved is false;
- historical mutation target/effect may remain;
- no validator clears or rewrites device state.

## 7. Preserve the Original Baseline Capture Time

Record a state value such as:

```python
baseline_custom_memory_captured_utc
```

exactly once when the first complete 256-byte baseline is accepted. Reuse that
value in `custom_memory_baseline.json`, later checkpoint metadata, and final
summaries. Keep `custom_memory_baseline.hex` as stable raw bytes; do not invent
metadata inside the hex representation.

Later checkpoint writes may update a separate `updated_utc`. They must not:

- relabel an old baseline as newly captured;
- replace baseline bytes or typed baseline values;
- add resume/replay behavior.

## 8. Reject Blank Hazardous Metadata

For every live hazardous plan, trim required metadata and reject:

- empty strings;
- whitespace-only strings;
- case-insensitive `unspecified`.

Apply this to:

- board;
- target name;
- operator;
- sensor ID;
- fixture ID;
- electrical authority;
- power procedure where currently required.

Keep dry-run behavior unchanged. Use one small helper; do not invent external
identity verification.

## 9. Unsupported Filter Writes and Factor Zero

The repository has no authoritative EE871 numeric filter table. Keep:

- read-only filter baseline;
- full-memory forensic capture;
- the general library typed API.

Remove the effectful `--write-co2-filter` runner option, plan row, restoration
row, help text, tests, and guide instructions. Do not guess a safe alternate,
accept arbitrary `0..255`, or use raw register writes. Record that filter-write
HIL is intentionally unavailable until an authoritative value table and
reviewed restoration procedure exist.

The core correction rejects interval-factor zero. Make the runner reject zero
at argument parsing as well:

```text
valid explicit factor test values: -128..-1 or 1..127
```

Do not narrow other factor values without authority. Factor remains an
explicit optional persistence row; do not choose one automatically.

Remove runner-only dead branches made obsolete by filter-row removal. Do not
clean unrelated code.

## Parser, Planner, and Artifact Tests

Extend `test/test_hil_runner_parser.py` against production runner functions.
At minimum prove:

1. D9 is unexpected globally;
2. D9 is accepted only for explicit auto-adjust target context;
3. an unintended D9 change plus an intended interval/calibration/address
   change is `FAIL`;
4. parsed non-OK status with missing success token is `FAIL`;
5. each intentional accepted non-OK contract remains accepted;
6. missing output without definite failure remains operator review;
7. unresolved or missing mutation state blocks destructive admission even if
   legacy dirty flags say clean;
8. address baseline mismatch blocks the first destructive command;
9. configured address state is not overwritten by candidate/restoration state;
10. advertised/running auto-adjust blocks writes;
11. advertised/idle auto-adjust passes;
12. unadvertised/clean `NOT_SUPPORTED` does not create a false blocker;
13. both supported and unsupported detailed-error checked-sample shapes are
    structurally validated;
14. a real sensor-domain error remains a healthy-bench failure without an
    invented transport failure;
15. SCL-low requires SDA high and SDA-low requires SCL high;
16. `TIMEOUT` is rejected where exact `BUS_STUCK` is required;
17. each stuck-line plan contains pre-fault health, tracked failure, in-fault
    health, release, recovery, and final health in order;
18. in-fault health requires a failure-counter increase and coherent
    state/online values;
19. repeated checkpoints preserve the original baseline timestamp and bytes
    while advancing only update time;
20. empty, whitespace, and placeholder hazardous metadata are rejected;
21. factor zero is rejected;
22. no filter-write command can be selected or built;
23. quick/default and complete-safe plans contain no destructive command;
24. no restoration command is sent after any failure or uncertainty;
25. no runner path replays raw custom memory;
26. a maintenance baseline records the complete image, diagnostic versions,
    and every typed persistent result, including explicit `NOT_SUPPORTED`;
27. a successful reversible row restores through its typed owner and requires
    typed readback, resolved mutation evidence, and full-image equality.

Repair tests that encode old D9, filter, failure-precedence, or permissive
fault behavior. Prefer structured assertions over substring-only tests.

## Documentation and Tooling

Update:

- `docs/EE871_E2_HIL_RUNNER.md`;
- runner help and script version;
- `README.md` runner section;
- `CHANGELOG.md` under `Unreleased`;
- current validation matrix/release notes where they describe runner behavior;
- a corrective handoff under `docs/reports/`.

Update active guidance and `CHANGELOG.md` under `Unreleased`. Do not rewrite
versioned historical release notes, validation results, or earlier prompt
handoffs.

Document:

- D9 target-only handling;
- complete mutation/address/auto-adjust preflight;
- precise stuck-line sequence and in-fault health evidence;
- immutable baseline capture time;
- filter-write HIL deferral;
- nonzero factor values;
- checked samples with and without error-code capability;
- the fact that no HIL was run by this source change.

Do not weaken warnings about level shifting, current limiting, isolated sensor
power, calibration references, address activation, or non-cancellable
auto-adjust.

## Validation

Run at least:

```text
python -m py_compile tools/ee871_hil_runner.py
python test/test_hil_runner_parser.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m platformio test -e native
python -m platformio run -e ex_bringup_s2
python -m platformio run -e ex_bringup_s3
git diff --check
```

Inspect dry-run plans for:

- quick/default;
- complete-safe;
- persistent interval;
- persistent factor;
- operating mode;
- part name;
- calibration;
- address change/restoration;
- auto-adjust;
- unplug/replug;
- stuck SDA;
- stuck SCL;
- power cycle.

Prove from dry-run plan data that safe plans have no destructive command and
that no filter-write plan exists. Use a concrete nonzero factor such as `1`
when building the factor dry run. The one `--include-stuck-line` dry run must
contain and be inspected for both `fault-sda-low` and `fault-scl-low`
sequences.

Run the native ESP-IDF example/contract build if its documented toolchain is
available. Report unavailable tooling honestly.

Do not open a serial port or run live/destructive HIL in this prompt.

## Handoff

Create a concise report under `docs/reports/` containing:

- baseline/final state;
- runner refactors and deleted obsolete branches;
- corrected preflight/state invariant table;
- corrected plan sequences;
- tests/builds/dry-runs actually performed;
- intentionally disabled filter row;
- remaining operator/HIL requirements.

## Acceptance Criteria

- An accidental D9 action cannot be ignored by an unrelated comparison.
- Definitive failures cannot degrade to operator review because success text
  is absent.
- No destructive command is sent without complete immutable baseline,
  resolved mutation state, matching address, and capability-aware idle
  auto-adjust evidence.
- Checked-sample validation matches both legitimate capability branches while
  preserving sensor-versus-transport semantics.
- Stuck-line rows prove the selected line only, exact BUS_STUCK, real in-fault
  health movement, and explicit recovery.
- Baseline bytes, typed baselines, and capture time remain immutable.
- Every reversible successful write is restored and verified through its typed
  owner; failed or uncertain writes stop without unsafe best-effort replay.
- Blank hazardous metadata and factor zero are rejected.
- Filter-write HIL is unavailable without authoritative values.
- Safe plans remain non-destructive.
- No replay, scan, hidden resync/recovery, or new framework was added.
- Tests and docs match production runner behavior.
- No hardware or HIL success is claimed.
