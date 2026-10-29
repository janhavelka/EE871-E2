# EE871-E2 software implementation prompts

**Status: SELECTED SOFTWARE COMPLETE.** Prompts 01 through 05 implement HIL-1,
CAP-1, CAP-2, CAP-4 and CAP-3. Prompt 06 closes the selected software scope;
CAP-5 strict startup is preserved and verified. All six prompts were selected;
no selected software scope is deferred. Hardware qualification remains separate.
These executable instructions were prepared on 2026-10-25 after source/test review of clean,
fetched `main` at `d37b586e35e553af2a65197282d6d478e554733b`.
The underlying [retained branch audit](../RETAINED_BRANCH_AUDIT.md) compares
implementation at `43168fcc60616cb62daf28224fdbf22864849bb9` with historical
branches. The intervening commit added that audit, not the missing features.
Re-audit actual HEAD whenever executing a prompt; this snapshot is not a reason
to overwrite subsequent fixes.

## Coverage, scope and order

| Finding | Prompt | Selection | State |
| --- | --- | --- | --- |
| HIL-1: stale clean state admits persistent writes after failed preflight | [01 — Persistent HIL admission](01_persistent_hil_admission.md) | Required defect correction; host-side implementation only. | IMPLEMENTED: current target baseline and fresh clean diagnostics gate every destructive send; campaign failures remain latched. |
| CAP-1: checked value/status sample result | [02 — Checked CO2 samples](02_checked_co2_samples.md) | Selected additive feature; raw APIs retain their existing contract. | IMPLEMENTED: explicit MV3/MV4 then status/optional detail, per-step evidence, manual Arduino/IDF callers; no product-range or freshness claim. |
| CAP-2: cooperative long waits | [03 — Cooperative maintenance waits](03_cooperative_maintenance_waits.md) | Selected additive callbacks; existing finite waits retain their settings. | IMPLEMENTED: optional sliced delay/yield only after persistent transfer, exact requested totals and native/Arduino adapters; no physical scheduler claim. |
| CAP-4: richer persistent operation evidence/admission | [04 — Persistent operation evidence](04_persistent_operation_evidence.md) | Selected fixed-size latest-operation evidence and application admission; existing dirty tracking remains authoritative. | IMPLEMENTED: bounded outer-operation evidence, cache-only CLI diagnostics and explicit CLI/HIL admission; no physical outcome claim. |
| CAP-3: programmatic operation timing bounds | [05 — Requested-delay bounds](05_requested_delay_bounds.md) | Selected additive pure API and real example consumer; current manual bounds were not automatically incorrect. | IMPLEMENTED: checked microsecond sums from explicit Config, bounded targets/counts and bus-silent Arduino/IDF diagnostics. |
| CAP-5: permissive absent startup in older branch | [06 — Software closeout](06_software_closeout.md) | Selected integrated closeout preserves strict startup; no new startup feature. | PRESERVED AND VERIFIED: precise failed begin, atomic recovery and persistent retention; existing tests suffice. |

Prompt 01 regression evidence in `test/test_hil_runner_parser.py`:
`test_main_failed_interval_baseline_blocks_explicit_serial_write` reproduces
the original forbidden send through the real `main()` loop;
`test_main_dynamic_interval_needs_current_not_niche_baseline` rejects stale
cached values; `test_main_each_target_requires_complete_successful_baseline`
covers interval/offset/gain preflight;
`test_main_maintenance_failure_stays_latched_after_clean_diagnostics` covers
continued admission closure; and
`test_main_authorized_write_has_separate_baseline_and_expected_evidence`
covers the authorized success path. Current-section boundaries and duplicate
headers are covered by
`test_main_following_section_cannot_replace_dirty_preflight`,
`test_main_following_section_cannot_supply_missing_resync_after_fields`, and
`test_main_mixed_case_duplicate_dirty_headers_cannot_admit_write`.
`test_main_contradictory_clean_error_cannot_admit_later_write` rejects clean/error
disagreement; `test_maintenance_resync_requires_complete_unique_after_fields`
also checks resync markers are unique, ordered and inside the current section.
These are host tests, not physical qualification. Requested-value success does
not restore the baseline, and uncertain writes require operator-owned
reconciliation outside the runner.

Prompt 02 adds `readCo2FastSample()` / `readCo2AverageSample()` and fixed-size
`Co2ReadResult`, preserving raw methods and their retry/health behavior.
The fake demonstrates raw success with status bit 3 set, then checked rejection
with retained value evidence in
`test_checked_samples_reject_sensor_error_while_raw_reads_still_succeed`.
`test_checked_samples_accept_full_word_range_in_value_then_status_order`
checks both value kinds and boundaries;
`test_checked_samples_keep_sensor_evidence_when_error_detail_transfer_fails`
checks retained status evidence and exact detail failure. Native coverage also
includes value/status failures, unsupported detail, sensor codes, OFFLINE/UNINIT,
result reuse and frame-retry decisions. The Arduino and native IDF CLIs expose
manual `co2fastsample` / `co2avgsample`; automated hardware plans are unchanged.
Checked success means successful requested transfers and clear sensor status,
with application-owned warm-up, freshness, accuracy and range policy.
Current implementation checks and exact CI evidence are recorded in its handoff;
no hardware qualification is claimed.

Prompt 03 appends optional `delayMs`/`yield` callbacks and
`longDelaySliceMs` (default 10 ms, valid 1..50 ms) without moving existing
configuration fields. Quiet waits request exact positive slices, delay before
yield, and keep the legacy per-millisecond microsecond fallback when callbacks
are absent.
`test_maintenance_wait_fallback_preserves_one_micro_delay_per_millisecond`
records a successful write's existing per-millisecond quiet wait;
`test_maintenance_wait_callbacks_cover_slice_boundaries_and_order` checks exact
totals and callback ordering, while
`test_maintenance_wait_callbacks_never_run_during_reads_pointer_retry_or_stretch`
checks the signaling boundary. Native coverage also checks configuration,
failure/dirty behavior and legacy aggregate initialization. Protocol and fixed
retry timing remain separate. Arduino and native IDF adapters install real
task-context callbacks; host recorders do not measure scheduler or watchdog
latency on devices. Exact local/CI evidence belongs in the execution handoff.

Prompt 04 adds cache-only `getLastPersistentOperation()` and
`PersistentOperationResult`, with one outer mutation boundary and 16-byte
requested/observed evidence. The existing bitmap and first dirty error retain
all unresolved targets; core writes remain permitted while dirty. Native
regression `test_interval_partial_write_resync_proves_coherence_not_requested_value`
demonstrates that coherence-only resync can succeed while the original request
remains unverified. `test_final_pec_sample_controls_uncertainty_when_clock_low_fails`
checks that a sampled PEC ACK/NACK remains authoritative after a later failure;
`test_persistent_operation_partial_pairs_and_names_keep_outer_progress` and
`test_persistent_operation_interval_readback_keeps_validated_byte_evidence`
cover partial outer operations and successful earlier read frames.
`test_persistent_operation_raw_interval_retains_outer_request_and_complement_validity`
checks the raw-to-paired interval boundary, while
`test_persistent_operation_diagnostic_is_bus_silent_and_health_neutral_in_every_state`
checks cache-only access. Pending-target, lifecycle, capability, malformed
address/mode and auto-adjust regressions retain the existing guarantees.

Both CLIs provide `operation` and gate maintenance on dirty state, possible
unmatched bytes and possible address effects.
`test_persistent_operation_cli_preserves_evidence_and_gates_without_io`
executes both CLI helpers and compares evidence/admission output;
`test_persistent_cli_contract_rejects_unguarded_mutations_and_bus_calls` checks
all ten guard sites and diagnostic purity. Recovery/resync cannot certify
intent or silently admit another write; no reset/admit or replay command was
added. Prompt 01's host campaign latch remains independent. Runner
integration requires and consumes a fresh open `operation` admission before
each destructive runner send.
`test_main_operation_evidence_blocks_sends_despite_clean_resync` and
`test_main_requires_fresh_operation_admission_for_each_write` exercise the
real host sender and preserve the campaign latch. Exact local/CI results
belong in the execution handoff. Physical
write outcomes, address activation and calibration remain unqualified.

Prompt 05 exposes pure `operationTimingBound()` and `OperationTimingBound` with
integer microsecond sums. Numeric configuration checks are shared with `begin()`;
queries before initialization require no callbacks and do not invoke them.
The explicit operation table covers the current checked samples and quiet-wait
callbacks without speculative APIs. Byte budgets include nominal phases,
volatile pointers retain ordinary STOP, direct writes use flash STOP, eligible
frames include only control-NACK retry costs, and resync reserves all 256 pending
targets plus baseline and conditional support reads. `bounds` in both CLIs
provides the real consumer without performing those operations. Callback and
scheduler costs remain application-owned. See the
[reviewed derivation](../../README.md#programmatic-requested-delay-bounds) and
execution handoff for exact verification and final commit/CI evidence; no
physical timing qualification is implied.

The native regressions in `test/test_basic.cpp` include
`test_requested_delay_bounds_match_independent_operation_examples` for default,
minimum and maximum settings;
`test_requested_delay_primitives_reach_their_hand_derived_bounds` for constructed
maximum requested-delay paths;
`test_requested_delay_retry_bounds_cover_last_success_exhaustion_and_high_latch`
and `test_requested_delay_bounds_cover_failed_cleanup_veto_and_later_timeout`
for retry and failure accounting;
`test_requested_delay_resync_bound_covers_no_targets_and_all_256_targets` for
full resync coverage; and
`test_requested_delay_queries_preserve_state_config_health_and_persistent_evidence`
for repeated query purity. `test/test_cli_contract.py` adds
`test_requested_delay_formatters_use_real_calculator_without_io`, which compiles
and executes both actual CLI formatters, and
`test_requested_delay_contract_rejects_bus_calls_and_wall_time_claims`, which
checks the diagnostic boundary. Exact local and final-commit CI results are
recorded below.

Prompt 06 reviewed the complete selected diff and current startup, transport,
sample, wait, evidence, timing and sender-admission owners. No concrete runtime
regression or redundant implementation owner was found; no source change or
duplicate startup test was needed. Failed strict `begin()` from UNINIT preserves
its precise status, UNINIT and an empty capability cache while retaining
persistent uncertainty. An active-session call returns `ALREADY_INITIALIZED`
without changing that session.
Failed recovery clears capabilities and latches OFFLINE; only complete identity
and capability verification restores READY. Recovery does not verify uncertain
writes, and resync establishes current coherence only.

Existing registered native tests provide the CAP-5 evidence:
`test_begin_validates_full_ee871_identity_and_co2_capability`,
`test_feature_cache_failure_fails_begin_closed`,
`test_malformed_feature_flags_fail_begin_and_recovery_closed`,
`test_failed_recover_clears_capabilities_and_latches_from_ready_or_degraded`,
`test_failed_recovery_is_atomic_and_keeps_offline_latched`,
`test_recovery_refreshes_feature_cache_atomically`,
`test_offline_replay_marks_message_and_preserves_original_diagnostics`, and
`test_pending_single_write_survives_offline_recovery_and_reinitialization`.
Raw read behavior and existing Config positional fields/error values are
preserved; new fields and `CO2_SENSOR_ERROR` append. No permissive startup,
hidden reset/retry or new framework was introduced. Application-owned bounded
hot-plug retries remain outside the driver.

## Implementation and verification record — 2026-10-29

All selected changes remain on the existing `main` branch. The preparation
baseline is `d37b586e35e553af2a65197282d6d478e554733b`; these commits identify
the implemented scopes:

| Prompt / finding | Implementation commit(s) | Verified outcome |
| --- | --- | --- |
| 01 / HIL-1 | `b41c46aa7b12071e769a2e7b49cfcf99dccacd7d`, `9df386a6696135abdad470495dbb406d8d63446f` | Actual destructive-send admission fixed, including inconsistent diagnostic sections. |
| 02 / CAP-1 | `46d01ab7a915eb32f7a59946446cc5dbd3c95f1d` | Explicit checked sample API and both CLI callers. |
| 03 / CAP-2 | `934a4d673a194122b609afa20ce62cdfa179f7c5` | Optional cooperative quiet waits and both transport adapters. |
| 04 / CAP-4 | `83af3cf8351e344e2c0be133324cc70d13f5de6d` | Latest-operation evidence and CLI/HIL maintenance admission. |
| 05 / CAP-3 | `5c920d00b7065a906b8fae7f4bf751a42796e955` | Pure requested-delay bounds and both CLI consumers. |
| 06 / CAP-5 and integration | Audited combined implementation `5c920d00b7065a906b8fae7f4bf751a42796e955` | Strict startup preserved and verified; documentation-only closeout. |

The combined implementation at
`5c920d00b7065a906b8fae7f4bf751a42796e955` passed these actual local checks:

- `python -m unittest discover -s test -p "test_*.py"`: 100 passed.
- `.\scripts\pio.cmd test -e native`: 134 passed.
- `python tools/check_core_timing_guard.py`,
  `python tools/check_cli_contract.py`,
  `python tools/check_idf_example_contract.py`, and
  `python scripts/generate_version.py check`: passed.
- `.\scripts\pio.cmd run -e ex_bringup_s3`,
  `.\scripts\pio.cmd run -e ex_bringup_s2`, and
  `.\scripts\pio.cmd run -e compat_tunnelmonitor_s3`: passed.
- `doxygen Doxyfile` and `git diff --check`: passed.

[CI run 40915629506](https://github.com/janhavelka/EE871-E2/actions/runs/40915629506)
completed successfully at that exact implementation SHA: native tests, library
validation, both Arduino builds and both native-IDF 6.0.1 builds (ESP32-S3/S2).
Local native-IDF builds were **NOT RUN** because the SDK was unavailable; static
checks are not substituted for those builds. Prompt 06 reuses the unchanged
implementation's test/build results and reconfirmed this terminal CI evidence.
Focused Doxygen, local links/anchors, source/test references, fenced blocks and
whitespace checks passed for this documentation-only closeout. Its later
documentation commit is distinct and identified in the final handoff.
Hardware access, physical timing/write/calibration qualification, branch
retirement and a new release remain outside this closeout.

## Original selection and execution rules

For the required defect fix and closeout, execute **01, then 06**. For the optional full
software feature set, execute **01 → 02 → 03 → 04 → 05 → 06**. Prompt 05 must
describe the operations actually present after any selected optional work.
Prompts 02, 03 and 04 are independent feature decisions and may be omitted;
prompt 05 must also work without them. Prompt 04 depends on 01 only if extending
runner admission; its core evidence work must not wait for an unrelated feature.
All work stays on the same existing branch for the whole selected prompt set.

Selecting a feature prompt is authorization to implement its stated software
scope through tests, documentation and publication. Do not stop at another plan
or ask again whether that selected feature is wanted. If the same capability
has already landed, verify its contract and close the finding with evidence
instead of building a duplicate. Unselected optional prompts remain deferred;
their absence does not block HIL-1 closure or imply a current defect.

Hardware qualification, serial-port access, flashing, real persistent writes,
calibration, waveform capture, physical faults, field soak and branch retirement
are excluded from every prompt. Existing hardware evidence remains in the
[validation matrix](../EE871_E2_HARDWARE_VALIDATION_MATRIX.md); do not turn those
rows into software implementation requirements or claim host tests close them.

## Preparation code audit supporting the prompts

The following observations describe the preparation baseline above. HIL-1 and
CAP-1 through CAP-4 are superseded by the implementation dispositions and
regression evidence in the coverage table. The strict-startup boundary remains
current and was preserved and verified by Prompt 06.

- HIL `main()` continues after a prompt-framed failure and stops only on a
  framing timeout. `update_state()` keeps older parsed values and clean flags;
  `maintenance_write_block_reason()` checks only dirty/resync flags. Explicit
  interval/offset/gain writes can therefore pass after their baseline fails.
  Default same-value interval rewriting is blocked when there is no prior
  interval, but an earlier niche read can supply a stale one. Command-line and
  runtime confirmation remain necessary and do not repair failed preflight.
- Raw `readCo2Fast()`/`readCo2Average()` call `readU16()` without status. Existing
  README explicitly assigns value/status composition and product validity to
  the caller. A checked helper is an additive contract, not a silent raw change.
- `sleepMs()` calls `delayUs(1000)` for each millisecond. Config has no long-wait
  callback; the Arduino/IDF adapters use busy-delay callbacks. Current separate
  ordinary/flash STOP policy and fixed read-retry pause must survive any change.
- `_pendingPersistentReads` plus the first dirty error already retains all
  uncertain targets across recovery and end/begin. `resyncPersistentConfig()`
  proves current readability/coherence, not successful application of original
  intent. A richer latest-operation record must not replace that protection.
- No `operationTimingBound()` API is present. Existing read-retry, calibration,
  flash STOP and pending-target resync paths make old branch formulas obsolete.
- `begin()` is strict; identity/feature errors leave UNINIT with an empty cache.
  `recover()` atomically refreshes capabilities and latches failure OFFLINE.
  The old permissive startup patch is not appropriate to restore.

Read source symbols/test names in each prompt rather than relying on historical
line numbers. Retained branch code is a source of design/test cases, never a
replacement for current main. Prefer a small refactor of the existing owner,
delete obsolete overlapping paths and keep a single decision point. Do not add
frameworks, scheduler engines, registries, speculative APIs or wrapper layers
that only hide inconsistent state ownership.

## Repository and verification contract

Each prompt repeats the necessary execution rules so it can be used on its own.
Read [AGENTS.md](../../AGENTS.md) and
[CI](../../.github/workflows/ci.yml) first. Fetch and inspect the branch/upstream;
fast-forward only when safe. Preserve concurrent work. The lead performs shared
Git mutations. Stage only owned changes, commit and push meaningful completed
chunks, verify upstream synchronization and never force-push. The generic fork
example in CONTRIBUTING does not override AGENTS' existing-branch rule.

Windows PlatformIO commands must use `scripts\pio.cmd`; do not install another
Core. CI currently uses unpinned PlatformIO on Ubuntu, Arduino pioarduino
55.03.311, and native ESP-IDF 6.0.1 for ESP32-S2/S3. The compatibility environment
uses pioarduino 54.03.20. Recheck actual pins/commands before implementation.
Workflow triggers are push/PR on main/master; no `workflow_dispatch` exists.
Use existing push/PR runs for substantial changes. Verify the exact final
implementation commit's terminal jobs; neither an earlier green SHA nor a
static contract check proves a native-IDF build. Do not weaken checks or alter
triggers to evade them. Small documentation-only dispositions require focused
local checks, not an extra remote CI dispatch/wait.

Common commands, selecting the appropriate subset described in each prompt:

```powershell
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m unittest discover -s test -p "test_*.py"
python scripts/generate_version.py check
.\scripts\pio.cmd test -e native
.\scripts\pio.cmd run -e ex_bringup_s3
.\scripts\pio.cmd run -e ex_bringup_s2
.\scripts\pio.cmd run -e compat_tunnelmonitor_s3
doxygen Doxyfile
git diff --check
```

Native IDF commands, in an IDF 6.0.1 shell, are
`idf.py -C examples/idf/basic_bringup set-target esp32s3 build` and the same
command with `esp32s2`. If unavailable locally, record NOT RUN and use the exact
implementation commit's existing CI jobs. Tests must assert externally visible
behavior, bus/serial effects, boundedness and error evidence, not private-state
shape or formulas copied verbatim from implementation. Native tests added to
`test/test_basic.cpp` must be registered in its Unity runner.

The current manifest has no explicit package allow-list, and Doxygen enumerates
specific inputs rather than all of docs. Prompt sources need no dependency,
manifest, workflow or version change. Links to this source-only prompt set must
remain valid in GitHub; do not expand the public API documentation build merely
to render implementation instructions. Review package contents before a future
release rather than quietly changing package policy here.

## Traceable closeout

For each executed prompt, update its status row with implemented / verified
already present / deliberately preserved, plus a concise rationale and relevant
test symbols. Record the implementation SHA, commands/results and exact CI run
SHA/URL in the final handoff; do not put a self-referential commit SHA into the
same commit. Update the audit's finding disposition without rewriting its
historical baseline. Keep permanent behavior in README/public headers and an
Unreleased changelog entry; avoid generating another narrative report. If CI
fails, fix the cause, rerun relevant checks and verify the new final SHA.

Prompt preparation checks passed on 2026-10-25: all three static contracts,
generated version consistency, Doxygen, local Markdown links, referenced commit
objects, fenced blocks and whitespace. Doxygen exposed existing relative links
to the source-only retained audit; those and the new prompt-index links use
GitHub source URLs without expanding inputs or weakening warnings. These checks
validate the instructions/documents only; all implementation statuses were
NOT IMPLEMENTED at preparation. Native/firmware builds, hardware runs and remote
CI were not run for this prompt-drafting change.
