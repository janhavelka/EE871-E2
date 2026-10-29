# Retained branch review

Reviewed on 2026-10-23 against `main` at
`43168fcc60616cb62daf28224fdbf22864849bb9`, after fetching origin and confirming
`main == origin/main` and a clean initial worktree. Main was the newest active
branch. Evidence describes that baseline, before this documentation commit.
No implementation was changed and no branch was merged or deleted.

## HIL-1 disposition after the audit — 2026-10-29

**HIL-1 is implemented in the host runner.** A single maintenance admission
owner requires a complete successful current target baseline and fresh clean
diagnostics before interval, offset or gain writes, including explicit values.
It preserves baselines separately from expected values and latches the first
maintenance failure; later clean or successful rows cannot reopen admission.
Diagnostic parsing and completeness checks use the same bounded current
section. Unexpected or duplicate sections (regardless of capitalization),
misplaced or unordered resync markers, and disagreement between clean flags
and the dirty error also block maintenance.

The real-main host regression
`test_main_failed_interval_baseline_blocks_explicit_serial_write` first
reproduced the unsafe `interval 160` send on the unchanged runner. Coverage in
`test/test_hil_runner_parser.py` also includes
`test_main_dynamic_interval_needs_current_not_niche_baseline`,
`test_main_each_target_requires_complete_successful_baseline`,
`test_main_maintenance_failure_stays_latched_after_clean_diagnostics`, and
`test_main_authorized_write_has_separate_baseline_and_expected_evidence`.
Section and consistency regressions include
`test_main_following_section_cannot_replace_dirty_preflight`,
`test_main_following_section_cannot_supply_missing_resync_after_fields`,
`test_main_mixed_case_duplicate_dirty_headers_cannot_admit_write`,
`test_main_contradictory_clean_error_cannot_admit_later_write`, and
`test_maintenance_resync_requires_complete_unique_after_fields`.
No hardware validation is claimed. A successful requested-value write is not
restoration; uncertain writes require operator-owned reconciliation outside
the runner. CAP-1..5 and branch retirement were not selected by this fix.

The remaining assessment and reproduction describe the reviewed historical
baseline, including its helper names and then-missing admission. Use the tests
above on current HEAD; see the [prompt index](prompts/README.md) for current
per-finding outcomes. Implementation SHA and exact terminal CI evidence belong
in the implementation handoff, not this commit's own contents.

## CAP-1 disposition after the audit — 2026-10-29

**CAP-1 is implemented as an additive checked sample API.**
`readCo2FastSample()` and `readCo2AverageSample()` share one procedure owned by
the existing driver: complete raw value first, status second, and supported
error detail only for a CO2 status error. They use the current raw/typed paths,
preserving protocol, narrow frame retries, low-byte latching and tracked health
ownership. Raw reads still succeed when their transfers succeed, even if the
sensor status would report an error; that behavior was intentional.

Fixed-size `Co2ReadResult` replaces its evidence on each call and records raw
value validity separately from checked validity, with status/error validity,
attempted flags and exact step statuses. Later failures retain earlier
evidence. Appended `CO2_SENSOR_ERROR` separates sensor rejection from transport
failure without renumbering existing errors; unknown raw detail remains
visible. There is no inherited 50,000 ppm limit. Checked success makes no
warm-up, freshness, accuracy, product-model or application-range guarantee.

Manual `co2fastsample` / `co2avgsample` callers in both diagnostic CLIs expose
that contract without changing live automated plans.
`test_checked_samples_reject_sensor_error_while_raw_reads_still_succeed`
demonstrates intentional raw success and checked rejection for the same sensor
error. Native fake regressions cover ordering, boundaries, transfer failures,
unsupported detail, sensor codes,
preconditions, retry decisions, health deltas and result reuse. See the
[prompt index](prompts/README.md) for current regression references and the
implementation handoff for exact local/CI results. No hardware qualification,
persistent sensor writes or branch retirement is claimed.

The remaining decision and tables preserve the historical audit baseline;
their then-missing CAP-1 description is superseded by this disposition.

## CAP-2 disposition after the audit — 2026-10-29

**CAP-2 is implemented as optional cooperative post-write quiet waits.**
Appended `Config::delayMs`, `Config::yield` and `longDelaySliceMs` retain all
earlier positional aggregate fields. Slices default to 10 ms and must be
1..50 ms; invalid settings fail before GPIO traffic. One shared helper requests
exactly the configured quiet wait in positive slices, using the delay callback
or the existing per-millisecond `delayUs(1000)` fallback, then optional yield
after each completed slice. Zero waits, 150/300 ms defaults and 5000 ms limits
remain unchanged.

Arduino and native IDF adapters install task-context callbacks, with IDF tick
rounding that avoids under-waiting. These callbacks never enter protocol
signaling, flash STOP stretching, SCL polling, reads, volatile pointer updates
or the fixed retry pause. Existing health, readback and persistent uncertainty
behavior remain with their current owners. Callbacks are bounded,
application-owned and non-reentrant under external serialization; void callbacks
cannot prove wall-clock adherence or report their own failure. Host tests show
requested waits and placement only, not scheduler/watchdog improvement on a
physical device.
`test_maintenance_wait_fallback_preserves_one_micro_delay_per_millisecond`
records the existing finite fallback on a successful fake write;
`test_maintenance_wait_callbacks_cover_slice_boundaries_and_order` checks the
additive callback path. See the [prompt index](prompts/README.md) for native
regression references and the implementation handoff for exact checks, SHA and
CI evidence.

The remaining decision and tables preserve the historical audit baseline;
their then-missing CAP-2 description is superseded by this disposition.

## Decision

[Executable software prompts](prompts/README.md) now map HIL-1 and CAP-1..5 to
independent implementation scopes, meaningful regression cases and a final
integrated closeout. They are NOT IMPLEMENTED; optional feature prompts remain
distinct from the required HIL admission fix. Hardware-only work is excluded.

There is important unimplemented work in the retained hardening series:
**the HIL runner's fail-closed admission of persistent writes after baseline or
maintenance verification failures**. A read-only host reproduction confirms
that current main can admit an explicitly requested write after a baseline
read fails. This is HIL-1 below, the first selective follow-up.

Several other branch capabilities are absent: checked value/status sample
results, cooperative long-wait callbacks, machine-readable operation timing
bounds, and richer persistent mutation evidence. Those are design/capability
decisions, not automatically defects. Main independently implements much of the
old timing, identity, capability, recovery and dirty-state protection, and has
newer retry and calibration fixes that the retained branches lack.

**Retain `feature/ee871-hardening-series` as reference until HIL-1 is resolved
and the capability decisions below are recorded. Do not merge it wholesale.**
Archive/retire `hardening/ee871-production-api-20260627` after preserving its
tip: its useful ideas are represented by the later series, while its permissive
startup policy masks transport failures. Already-contained branch names may
retire separately. No retirement action is performed by this audit.

## Exact branch inventory

Counts are main-only / retained-tip-only from
`git rev-list --left-right --count <main>...<tip>`. All remote branches were
inspected; `origin/HEAD` is only an alias.

| Retained `origin/` branch | Full tip SHA | Counts | Disposition |
| --- | --- | --- | --- |
| `feature/ee871-hardening-series` | `dae9bea1155db4efb757a37a99ed7b507d718e6c` | 33 / 13 | Unmerged implementation and HIL work; selective review required. |
| `hardening/ee871-production-api-20260627` | `27023a164b4bd8c5a51614fffb071329e1b70867` | 36 / 1 | Unmerged older implementation alternative. |
| `feature/ee871-e2-idf-port` | `58d23fde9ba956dc1619d8a74c09188932351a19` | 63 / 0 | Already contained; native port is not missing. |
| `hardening/ee871-e2-industry-readiness` | `44029164746fc0aa0147e0f08efaf7c2ff619230` | 41 / 0 | Already contained. |
| `debug-example` | `cfca1a9acee48842430957ce0c4e86708d437d52` | 88 / 0 | Already contained. |
| `agent/pioarduino-55-03-311-hil` | `0e6b87bd38f8dbbf8420c0aaa38708ca861004a2` | 23 / 0 | Already contained; local tip matches. |
| `fix/ee871-read-stop-timeout-20260910` | `571d370b33064c491320654a8ed2992b3ecab038` | 14 / 0 | Already contained; local tip matches. |
| `feat/ee871-read-retries-20260911` | `d55c3acee8d452c557e1522f91b26a3126f06a09` | 12 / 0 | Already contained; local tip matches. |

`git cherry` marks all fourteen unique commits `+`. That only means their
patches are not identical to patches on main; the semantic assessment follows.

## Every unique commit and its current relevance

| Commit | Actual change | Current disposition |
| --- | --- | --- |
| `d6100b01723a95b618110dc4696c64ad3977d2c2` | Byte/completion timing, long-wait callbacks, operation timing API and fault tests. | Main has independent strict bit/byte timing and flash STOP budgets. Cooperative waits/timing API remain absent (CAP-2/3). Old volatile-pointer flash allowance must not return. |
| `739beeca9017bc678f026d4b466aa2bb6332b08e` | Follow-up timing and fake-transport boundary corrections. | Compare any selected timing idea with current nominal-byte/deadline tests; not a standalone current fix. |
| `974f7303ee9348e6defa4b0e76962d6eab05d1fb` | Atomic identity/capability snapshots, lifecycle/recovery contracts, optional startup policy. | Main validates full identity and atomic 0x07..0x09 capability cache, latches OFFLINE and requires recovery. Expanded snapshots/policy are optional differences. |
| `432519862430703a056d4ab37247adcf5fbbc1d1` | Lifecycle correction: conservative absence handling and related contracts. | Correctly rejects treating a NACK as authoritative absence. Main's strict initialization is compatible with that principle; do not restore older permissive policy. |
| `82b656ecb2010e92729e2b71ece5fe13c4c2e766` | `Co2ReadResult`, checked MV3/MV4 then status/error procedure and per-step tests. | Missing capability CAP-1. Main intentionally exposes raw values plus separate status and documents application composition. |
| `5df2515a46c604ee5dcbe5da02c7950a5bc3169e` | Corrects checked-sample/status-trigger wording in guidance. | Current README already states the interval/age condition; no runtime patch. |
| `50702f3805f2a24f300fd189e0992b35ed96a714` | Target/effect/intent mutation diagnostics, admission, reconciliation, typed maintenance flows, timing contracts and tests. | Main now has sticky first-error and all-pending-target bitmap resync. Rich intent/effect/admission is still absent (CAP-4), not evidence that dirty tracking is missing. |
| `fed97e657e9b0e3cf27bc6eca27c39fd727c7339` | Follow-up persistent-state and release/version-generator corrections. | Reassess only selected behaviors on main; current generated metadata is maintained separately. Do not replay historical version changes. |
| `1bffb9ef4565ab8a848c845088fc5e37c65c5faf` | Expanded safe/persistent/fault HIL plans, command evidence, typed restore and parser tests. | Some useful runner behavior remains absent. HIL-1 has a concrete current failure case; expanded plans/restoration are optional subsequent work. |
| `d40f70a41b88584c020f9efc16889ffcade69117` | Completion-budget correction and fail-closed capability/mode metadata with tests. | Main has independent completion limits, reserved-bit checks and on-demand calibration guards. Branch timing formulas still assume flash completion for 0x50; do not transplant. |
| `b3c130dc8ecf0b77d7d937d816955884482fcccb` | Complete immutable baseline/preflight and maintenance-failure admission gates. | Important missing mechanism HIL-1. Select minimal applicable behavior; do not import the whole older runner. |
| `9bd174ed395feec33c2288803df7912a996b730f` | Checks baseline contents, fresh idle auto-adjust admission, per-fault evidence and sample status consistency. | Reference for HIL-1 regression cases. Auto-adjust/extra fault workflows only matter if those plans are added. |
| `dae9bea1155db4efb757a37a99ed7b507d718e6c` | Removes stale helpers/stubs, revises runner/CLI/docs, adds workflow checks. | Main has its own cleanup, native IDF workflow and newer CLI/parser repairs. No reason to restore deleted historical documentation or old workflow pins. |
| `27023a164b4bd8c5a51614fffb071329e1b70867` | Older alternate checked samples, stronger identity, long-wait callbacks and `BeginPolicy::AllowAbsent`. | Identity is implemented independently; CAP-1/2 cover remaining useful ideas. Its `isPresenceOrTransportFailure()` accepts NACK, TIMEOUT, BUS_STUCK, E2_ERROR and PEC_MISMATCH as successful OFFLINE startup: do not import that policy. |

Historical prompt reports and test counts at those tips are not current
verification and do not establish hardware behavior on main.

## What is already implemented or deliberately different

Evidence paths/lines below refer to the reviewed main baseline.

- **Timing and completion:** `src/EE871.cpp:314` validates bit period, nominal
  byte time, 25/35 ms limits and separate flash stretch allowance. Ordinary read
  and volatile-pointer STOPs use the ordinary deadline; custom writes use the
  flash deadline. `README.md:248` explains the distinction and callback overhead.
  Tests include `test_begin_validates_generated_clock_period`,
  `test_begin_validates_nominal_byte_deadline`,
  `test_byte_deadline_includes_nominal_phases_after_stretch`, and
  `test_flash_write_stop_stretches_commit_and_verify`.
- **Identity/recovery:** `_validateIdentityRaw()` (`src/EE871.cpp:1244`) checks
  group, subgroup and CO2 support. `_readFeatureFlagsRaw()` installs only valid
  complete flags. Failed recovery clears capabilities and latches OFFLINE;
  ordinary traffic cannot clear it. Main retains the original transport error
  code/detail instead of the branch's new `OFFLINE` code. Tests cover identity,
  malformed flags, atomic recovery, and offline replay. That is a documented
  diagnostic choice, not missing recovery.
- **Calibration and persistent uncertainty:** `_requireCo2AdjustmentSupport()`
  at `src/EE871.cpp:1288` validates 0x03/0x04 on demand. `resyncPersistentConfig()`
  at `:496` checks every pending target and retains uncertainty across recovery
  and end/begin. Single/raw writes mark uncertainty once acceptance is possible;
  final PEC NACK has a definite-rejection case. Auto-adjust requires support,
  valid bits and idle state. Current tests at `test/test_basic.cpp:2129`, `:2204`,
  `:2237`, `:2311`, `:2357`, `:2384` cover these contracts. Resync intentionally
  proves coherent actual state, not requested values or historical calibration
  success; that limitation is already explicit in the public header/README.
- **Framework/port:** native `examples/idf/basic_bringup` and CI for ESP32-S2/S3
  exist. `EE871` is non-copyable/non-movable, framework neutral and externally
  serialized. Broader branch capability caching of 0x03..0x06 is unnecessary
  for current on-demand typed checks.
- **Newer fixes to preserve:** main's `571d370b33064c491320654a8ed2992b3ecab038`
  ordinary STOP correction, `a358f92a6882e00810c775a61f5499d5cff60885` bounded
  read retries, `3d32ac30cc4b228299d3f0feaffd538c031f5276` calibration/uncertain
  write guards, and subsequent CLI capture/parsing fixes are not established by
  the old branch. Any selected port must keep them and their tests.

## HIL-1: persistent writes remain admitted after failed preflight

**P1, confirmed current runner defect for opted-in persistent testing.** It
does not affect default non-persistent plans, and no serial device was opened
to reproduce it.

Failure path:

1. A prior `dirty` row establishes `persistent_config_dirty=False` and
   `resync_needed=False`.
2. The subsequent baseline `interval` read fails with a complete response
   (for example a PEC or transport error), so no restorable baseline is captured.
3. `update_state()` (`tools/ee871_hil_runner.py:1007`) retains the previous clean
   flags and does not latch baseline failure.
4. `maintenance_write_block_reason()` (`:915`) checks only those cached flags.
   An explicit `interval 160`, `offset 10` or gain command is still admitted.
   `main()` (`:1319`) stops the plan only on response-framing timeout, not an
   ordinary failed verification row. Final FAIL classification occurs after
   subsequent commands may already have executed.

A host probe using the actual unchanged runner functions produced:

```text
interval 160 gate= None resolve= ('interval 160', None) baseline= None
offset 10 gate= None resolve= ('offset 10', None) baseline= None
```

The branch's `b3c130d`/`9bd174e` series supplies the missing principle: complete
validated baseline plus latched verification failure before destructive
admission. A full 256-byte image is one historical implementation choice; the
minimum fix should require complete, current, target-specific restorable
baseline/evidence and cannot rely only on a stale clean flag.

Acceptance criteria for a focused fix:

- Missing, failed, malformed or stale baseline blocks **every** persistent write
  for that target, including explicit-value commands; preserve the original
  failing row/reason in the result.
- A failed maintenance write/readback/dirty/resync check latches failure and
  blocks later persistent actions. Do not automatically restore after uncertain
  writes without a separate verified recovery/restoration decision.
- Add host tests for failed interval/offset/gain baseline, good baseline then
  failed readback, stale clean flags, and a successful explicitly authorized
  flow. Assert forbidden commands never reach the serial-send fake.
- Preserve main's bounded serial writes, complete-response checks, status-name
  parsing, retry diagnostics, and non-destructive default plans.
- Decide separately whether to add automatic typed restoration. Current plans
  change values but do not provide the retained branch's full baseline/restore
  campaign; report that boundary explicitly rather than inferring restoration.

Minimal read-only reproduction from the repository root:

```powershell
@'
import importlib.util, sys
from pathlib import Path
p = Path('tools/ee871_hil_runner.py')
s = importlib.util.spec_from_file_location('audit_runner', p)
m = importlib.util.module_from_spec(s)
sys.modules[s.name] = m
s.loader.exec_module(m)
a = m.parse_args(['--dry-run', '--include-persistent-writes',
    '--confirm-persistent-writes', '--maintenance-interval', '160',
    '--write-co2-offset', '10'])
plan = m.maintenance_specs(a)
state = {'persistent_config_dirty': False, 'resync_needed': False}
failed = m.result_row(plan[1], 'interval', m.RESULT_FAIL,
    'read failed', 0.0, '', 'prompt', {})
m.update_state(state, failed)
for step in plan:
    if step.destructive:
        print(step.command, 'gate=', repr(m.maintenance_write_block_reason(step, state)),
              'resolve=', m.resolve_dynamic_command(step, state),
              'baseline=', state.get('measurement_interval_ds'))
'@ | python -
```

## Capability decisions still needed

These are verified branch/main differences. They are not all mandatory repairs.

| ID / priority | Missing branch capability and current boundary | Decision and acceptance criteria |
| --- | --- | --- |
| CAP-1 / P2, P1 if a consumer treats raw read success as valid CO2 | Branch `Co2ReadResult`, `readCo2FastSample()` and `readCo2AverageSample()` perform value then status, optional error-code read, per-step evidence and separate sensor-domain result. Main `src/EE871.cpp:828`/`:832` only returns raw MV3/MV4; README explicitly delegates warm-up, status, range and freshness to application. A successful raw read with CO2 status bit 3 set is therefore not a valid checked sample. | First inspect the real consumer. If it already composes value/status and rejects sensor errors, no API addition is required. Otherwise add a narrow checked helper or fix consumer composition; test low/high/status/error-code failures, unsupported error detail, sensor codes 1/200/201/202/unknown, and no invented freshness. Preserve raw APIs; do not adopt the old hard-coded 50,000 ppm range as a universal product qualification limit. |
| CAP-2 / P2, P1 if maintenance shares a latency-sensitive task | Branch optional `delayMs`, `yield`, `longDelaySliceMs` let long post-write waits cooperate. Main `sleepMs()` at `src/EE871.cpp:306` repeatedly calls `delayUs(1000)`; example callbacks use `delayMicroseconds`/`esp_rom_delay_us`, so the configured 150/300 ms and up-to-5 s waits busy-wait. They are bounded, but not cooperative. | Either isolate maintenance from the latency-sensitive owner, or append optional long-wait callbacks without changing existing Config positional fields. Test bounded slices and default behavior, no yields inside bit/ACK/START/STOP signaling, callback non-reentrancy, and actual scheduler/watchdog behavior. Do not change protocol waits merely to satisfy a timing preference. |
| CAP-3 / P2 | Branch `operationTimingBound()` and `OperationKind` calculate admission bounds; current README gives manual requested-delay formulas instead. | Add only for a concrete owner needing programmatic admission. Recalculate against main's separate flash STOP and ordinary pointer timing, optional 0..3 read retries, calibration guards and 256-target resync. Test maximum valid settings, overflow, unsupported counts and callback-overhead caveats. The branch's 0x50 flash formula is obsolete. |
| CAP-4 / P2, P1 if maintenance requires operation-effect evidence | Branch `MutationDiagnostic` distinguishes no effect/acknowledged/indeterminate/verified/resynchronized, keeps intended target values and gates further mutations. Main exposes sticky error plus all-pending-target coherence, but no requested-versus-observed intent record or blanket mutation admission lock. | Decide whether application-owned expected values and maintenance serialization already satisfy the consumer. If not, port the narrow evidence/admission requirement; preserve main's multi-target uncertainty across recovery/end/begin and do not let later writes erase earlier targets. Test every partial write/PEC/ACK/STOP/readback phase, address activation uncertainty, auto-adjust one-shot semantics and resync limits. Do not import the old large mutation framework without that need. |
| CAP-5 / no automatic implementation recommendation | Older `AllowAbsent` returns successful OFFLINE startup for several transport faults. Later series restricts acceptance to authoritative DEVICE_NOT_FOUND, which raw GPIO E2 responses do not themselves establish. Main fails strict begin and leaves UNINIT. | Keep strict startup. If hot-plug startup is required, application policy can retry begin with a bounded schedule. Any new library policy must distinguish absence from NACK/PEC/timeout/stuck line and retain failure evidence; do not equate these with confirmed absence. |

## Hardware evidence remains separate

[The validation matrix](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) already records
the limits: current standalone CLI/long-soak qualification, electrical waveform
timing, physical fault injection, supported persistent address behavior and
calibration are not all qualified. The retained branch's HIL plans and passing
host tests do not close those rows. Run the selected fixture procedures only
after any HIL-1 fix, recording exact commit/firmware identity, actual settings,
artifacts and restoration. Keep this evidence in the existing matrix.

## Reproduction and retain/retire workflow

```powershell
git status --short --branch
git for-each-ref --format="%(refname:short) %(objectname)" refs/heads refs/remotes/origin
git rev-list --left-right --count 43168fcc60616cb62daf28224fdbf22864849bb9...origin/feature/ee871-hardening-series
git cherry 43168fcc60616cb62daf28224fdbf22864849bb9 origin/feature/ee871-hardening-series
git log --reverse --format="%H %s" 43168fcc60616cb62daf28224fdbf22864849bb9..origin/feature/ee871-hardening-series
git diff origin/feature/ee871-hardening-series 43168fcc60616cb62daf28224fdbf22864849bb9 -- include/EE871/Config.h include/EE871/EE871.h src/EE871.cpp tools/ee871_hil_runner.py
git show b3c130dc8ecf0b77d7d937d816955884482fcccb -- tools/ee871_hil_runner.py test/test_hil_runner_parser.py
git merge-base --is-ancestor origin/feature/ee871-e2-idf-port 43168fcc60616cb62daf28224fdbf22864849bb9
```

1. Start with HIL-1 on current main, writing the failing host regression first;
   use the historical preflight commits as design/test evidence.
2. Record accept/defer/reject for CAP-1..5 against actual consumer requirements.
   An absent public symbol alone is not a reason to add it.
3. Validate each selected behavior using current tests/contracts, then verify
   the exact final implementation commit in existing CI when the change is
   substantial. Preserve current release metadata and supported platforms.
4. Refresh refs before retirement; review any moved tips. Preserve both unique
   tips in named durable remote archive references, verify availability, then
   remove obsolete active branch names only in a separate explicit maintenance
   action. Retain the series until HIL-1 and capability disposition are recorded.

## Checks actually run and future CI

Completed for this audit: ancestry/cherry/diff and every unique-commit inventory;
semantic source/test/workflow review; HIL-1 pure host reproduction; all three
static contracts; and Python tool tests, **68 passed**:

```powershell
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m unittest discover -s test -p "test_*.py"
```

Final `git diff --check`, local Markdown links, and every full commit SHA were
checked and passed.
No native tests, firmware builds, physical operations or remote CI are claimed
for this documentation-only audit. A future HIL-1 fix needs meaningful parser/
serial-send regression tests; a future core/API change also needs native tests
through `.\scripts\pio.cmd test -e native`, both Arduino environments
`ex_bringup_s3`/`ex_bringup_s2`, version checks and native IDF 6.0.1 ESP32-S2/S3
builds from `.github/workflows/ci.yml`. The present workflow installs unpinned
PlatformIO and triggers on main/master pushes and PRs; it has no manual dispatch
trigger. Do not change triggers to avoid checks or infer final-commit coverage
from an old green run.
