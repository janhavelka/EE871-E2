# Prompt 06 — Close the selected software work and preserve strict startup

**Findings:** selected HIL-1/CAP-1..4 work and CAP-5 disposition.
**Status: NOT IMPLEMENTED.** This is the final integrated closeout prompt, not
a new feature request. CAP-5 means preserve the existing strict startup policy;
no startup defect or missing API is presumed. Do not add tests that only repeat
already-covered implementation, and do not add `AllowAbsent` or a new startup
framework. Complete any concrete regression discovered in the selected changes
and publish an accurate software closeout. Hardware work is excluded.

## Establish what actually changed

Read AGENTS.md, docs/prompts/README.md, docs/RETAINED_BRANCH_AUDIT.md and
.github/workflows/ci.yml. Fetch and inspect the current branch/upstream,
fast-forward only safely and preserve unrelated work. Remain on the existing
prompt-set branch; the lead owns shared Git mutations. Preparation baseline
was `d37b586e35e553af2a65197282d6d478e554733b`; compare it with actual HEAD and
the implementation commits recorded for each selected prompt.

List selected prompts and their implemented/verified/deferred state. HIL-1
requires the real sender-admission correction or evidence it landed elsewhere.
Optional capabilities are not required just because their prompt files exist.
Do not retroactively claim a feature was selected, or mark a scope deferred
when its invoking task explicitly requested implementation.

Inspect current `begin`, `_resetStoppedState`, `_validateIdentityRaw`,
`_readFeatureFlagsRaw`, `probe`, `_recoverTracked`, `_offlineStatus`,
`_readControlByteTracked`, `_writeCommandTracked`, `end` and
`resyncPersistentConfig` in src/EE871.cpp. Read Config/Status/public contracts
and existing tests in test/test_basic.cpp:

- `test_begin_validates_full_ee871_identity_and_co2_capability`;
- `test_feature_cache_failure_fails_begin_closed`;
- `test_malformed_feature_flags_fail_begin_and_recovery_closed`;
- `test_failed_recovery_is_atomic_and_keeps_offline_latched`;
- `test_recovery_refreshes_feature_cache_atomically`;
- `test_offline_replay_marks_message_and_preserves_original_diagnostics`;
- lifecycle/pending-target retention and retry-exclusion tests.

Historical CAP-5 comparison: `git show`
`27023a164b4bd8c5a51614fffb071329e1b70867` for `BeginPolicy::AllowAbsent`,
`isPresenceOrTransportFailure()` and `begin()`, then
`432519862430703a056d4ab37247adcf5fbbc1d1` and final historical tip
`dae9bea1155db4efb757a37a99ed7b507d718e6c`. The oldest branch accepts NACK,
TIMEOUT, BUS_STUCK, E2_ERROR and PEC_MISMATCH as successful OFFLINE startup.
The later branch only accepts authoritative DEVICE_NOT_FOUND; raw GPIO E2
transactions do not establish that absence distinction. Current main rightly
retains strict begin and reserves DEVICE_NOT_FOUND for compatibility.

## Integrate without inventing a new workstream

Verify these invariants against the selected implementation's actual call paths:

| Boundary | Required invariant |
| --- | --- |
| Failed strict begin | Precise failure returned, UNINIT, no partly installed capability cache; persistent dirty evidence survives. |
| NACK/PEC/timeout/stuck line/wrong identity | Never reclassified as confirmed absence or hidden by successful startup. |
| Failed recovery from READY/DEGRADED/OFFLINE | Capability cache invalidated, OFFLINE latched, original failure evidence retained; normal traffic cannot clear it. |
| Successful recovery | Full identity/capability verification before READY; no claim uncertain persistent writes were verified. |
| New checked samples, waits, timing query or operation evidence | Reuse existing owner/transport contracts; no private startup bypass or hidden reset/retry. |
| HIL failure admission | Prompt-framed baseline failure blocks actual destructive sends even with old clean flags or stale values; final FAIL alone is insufficient. |
| Compatibility | Raw reads unchanged; Config positional fields preserved; enum values append-only; managed synchronous/framework-neutral core remains. |
| Documentation | Every optional API limit is explicit, and host evidence is never labeled hardware qualification. |

Existing test coverage may fully satisfy startup verification. Reuse it. If
the selected changes introduce a genuine uncovered failure, first reproduce
the smallest observable regression using the existing fake, then refactor the
responsible existing owner and add only the necessary regression. Do not create
a redundant no-op code change or duplicate test batch to make CAP-5 look newly
implemented. If unchanged and covered, record **preserved and verified**, citing
the source/test evidence and actual checked SHA.

Any startup retry for a hot-plug application remains application-owned and
bounded. This prompt does not authorize new recurring retries, GPIO ownership,
absence guessing or modifications to an unrelated consuming repository.

Review the complete selected diff for layered workarounds: duplicated validation,
parallel maintenance-state owners, competing evidence caches, copied protocol
loops, stale formula tables or dead adapters. Simplify only where directly
introduced or made obsolete by this selected work. Preserve established public
contracts and remove redundant paths rather than layering another wrapper.

## Verify the final implementation honestly

Use prior recorded results when they verify the exact unchanged final source
tree; do not rerun unchanged suites just for this closeout. New source/tests,
failed checks or unresolved concerns require relevant reruns. Applicable commands:

```powershell
python -m unittest discover -s test -p "test_*.py"
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
.\scripts\pio.cmd test -e native
.\scripts\pio.cmd run -e ex_bringup_s3
.\scripts\pio.cmd run -e ex_bringup_s2
.\scripts\pio.cmd run -e compat_tunnelmonitor_s3
doxygen Doxyfile
git diff --check
```

Windows uses the managed wrapper, not another Core install. Native IDF 6.0.1
builds use `idf.py -C examples/idf/basic_bringup set-target esp32s3 build` and
esp32s2. Report unavailable local builds as NOT RUN, and use actual existing
CI jobs for exact-commit build evidence. For substantial selected changes or
new integration fixes, verify every required job at the final implementation
SHA through existing push/PR runs. There is no manual dispatch trigger; never
modify triggers or weaken checks. If this step only changes closeout prose,
focused docs/link/Doxygen/whitespace verification is enough; clearly distinguish
the green implementation SHA from the later documentation SHA.

## Finish and publish

Update the prompt index with per-finding outcomes and test symbols, and audit
dispositions without changing its historical baseline claims. CAP-5 should say
strict startup preserved, not missing feature implemented. Optional unselected
features stay NOT IMPLEMENTED/deferred with their selection boundary intact.
Keep lasting usage/compatibility rules in README/public headers and release
changes in the Unreleased changelog. Do not generate another bulky campaign
report, tag a release, delete branches or change hardware qualification rows.

Stage only owned changes, commit/push completed work and confirm upstream
synchronization without force-push. Final handoff must identify implementation
and documentation commits, selected fixes/features, explicit deferrals,
actual checks, exact terminal CI SHA/URL, and limitations. Any reproduced
selected-scope defect must be resolved and verified; do not stop at a list of
follow-up plans while reporting the prompt complete.
