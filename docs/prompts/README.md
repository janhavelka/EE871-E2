# EE871-E2 software implementation prompts

**Status: PARTIALLY IMPLEMENTED.** Prompt 01 implements HIL-1; prompts 02..06
remain NOT IMPLEMENTED and were not selected with this fix. These executable
instructions were prepared on 2026-10-25 after source/test review of clean,
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
| CAP-1: checked value/status sample result | [02 — Checked CO2 samples](02_checked_co2_samples.md) | Optional additive feature; invoking this prompt selects the feature. Raw API absence is not a bug. | NOT IMPLEMENTED |
| CAP-2: cooperative long waits | [03 — Cooperative maintenance waits](03_cooperative_maintenance_waits.md) | Optional additive callbacks; invoking this prompt selects the feature. Current waits are bounded. | NOT IMPLEMENTED |
| CAP-4: richer persistent operation evidence/admission | [04 — Persistent operation evidence](04_persistent_operation_evidence.md) | Optional fixed-size evidence and application admission; invoking it selects the feature. Current dirty tracking already exists. | NOT IMPLEMENTED |
| CAP-3: programmatic operation timing bounds | [05 — Requested-delay bounds](05_requested_delay_bounds.md) | Optional additive pure API and real example consumer; invoking it selects the feature. Current manual bounds are not automatically incorrect. | NOT IMPLEMENTED |
| CAP-5: permissive absent startup in older branch | [06 — Software closeout](06_software_closeout.md) | Integrated closeout preserves strict startup; no new feature or mandatory duplicate test batch. | NOT IMPLEMENTED |

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

The following observations describe the preparation baseline above. HIL-1 is
superseded by the implementation disposition and regression evidence in the
coverage table; the optional capability observations remain unchanged.

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
