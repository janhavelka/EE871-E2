# Prompt 01 — Fail closed before persistent HIL commands

**Finding:** HIL-1. **Status: NOT IMPLEMENTED.** Required software defect fix.
Implement this prompt end to end in EE871-E2. Do not open serial ports, flash,
run hardware HIL or perform real persistent writes; use host fakes only.

## Start from current source

Read AGENTS.md fully, docs/prompts/README.md, the HIL-1 section of
docs/RETAINED_BRANCH_AUDIT.md, docs/EE871_E2_HIL_RUNNER.md and
.github/workflows/ci.yml. Fetch and inspect the existing branch/upstream,
fast-forward only when safe, preserve unrelated work and use the same branch
for this whole prompt set. If subagents share the checkout, the lead owns Git
mutations. This prompt was prepared at
`d37b586e35e553af2a65197282d6d478e554733b`; re-audit current HEAD first.

Inspect tools/ee871_hil_runner.py: `maintenance_specs`, `build_plan`,
`confirm_persistent_runtime`, `resolve_dynamic_command`,
`maintenance_write_block_reason`, `run_serial_command`, `update_state`,
`record_persistent_write_expectation`, `classify_response` and `main`.
Read test/test_hil_runner_parser.py, especially
`test_maintenance_writes_are_gated_by_clean_dirty_state`,
`test_persistent_readbacks_compare_expected_values`,
`test_prompt_framed_value_nack_completes_without_timeout`, and the fake serial
patterns used by synchronization/short-write tests. Also inspect typed driver
write behavior to understand that an ordinary read failure need not set dirty
or reach the OFFLINE threshold.

Use `git show` for the runner/tests at historical commits
`b3c130dc8ecf0b77d7d937d816955884482fcccb` and
`9bd174ed395feec33c2288803df7912a996b730f`. They contain complete-baseline and
failure-latch ideas. Do not cherry-pick the older runner; it lacks newer current
parsing, serial and diagnostic fixes and has a much broader campaign scope.

## Reproduce before fixing

Build a host regression through the real `main()` control flow or the existing
command execution loop with mocked serial/open/input/artifact boundaries.
Model successful initial synchronization and default safe-prefix commands,
the exact persistent opt-in flags, successful runtime confirmation, a fresh
clean dirty row, then a complete prompt-framed failing interval baseline.
Select an explicit `--maintenance-interval 160`. Assert no persistent command
is sent afterward; current code sends it because cached clean flags survive.
Do not bypass runtime confirmation in the production path to simplify testing;
provide the exact confirmed input in the fake. The final run already becoming
FAIL does not prevent an earlier unsafe send, so assert the actual send list.

Also demonstrate the distinct default-value case: without any prior parsed
interval, `interval <current>` is skipped; with a successful earlier niche
interval followed by failed current preflight, stale state must not substitute
for a current baseline. Repeat explicit offset/gain cases after their own
baseline failure. No physical fault or live sensor is needed.

## Implement a small coherent admission model

Refactor the existing plan/state ownership so there is one authoritative
preflight/admission decision immediately before every destructive send. Make
target identity, successful current baseline, expected value and latched
maintenance failure explicit. Prefer a small existing dataclass/local state
record with clear lifetimes over adding loosely coupled flags in several
functions. Remove obsolete checks/state that the new owner supersedes.

- Capture a target-specific baseline only from a complete successful response
  in the current maintenance preflight; do not accept values merely present in
  the global parsed-state dictionary. Preserve it separately from later
  observations/expected write values, and do not overwrite it after mutation.
- Require fresh clean diagnostics and the selected target's successful baseline
  before **all** interval/offset/gain writes, including explicitly supplied
  values. A malformed, failed, missing or stale baseline is a hard admission
  failure with an actionable recorded reason.
- Latch the first maintenance baseline/write/readback/dirty/resync failure for
  the selected campaign. Later successful diagnostic rows must not reopen
  destructive admission. Read-only diagnostics may continue where framing is
  still synchronized; framing timeout must retain its current stop behavior.
- Keep opt-in flags, exact runtime confirmation, value range checks, bounded
  serial writes, complete current-response validation and no-write defaults.
  Do not turn a failed original row into a successful result because subsequent
  writes were skipped. Preserve final nonzero verdict semantics and evidence.
- Do not introduce full-memory snapshots, address-changing plans, auto-adjust,
  new fault workflows, generic execution engines or automatic restoration in
  this fix. Existing typed baseline coverage is sufficient for this scope.
  State explicitly that a successful requested-value write is not restoration;
  uncertain writes require operator-owned reconciliation outside this runner.

## Required failure matrix and acceptance

| Scenario | Observable requirement |
| --- | --- |
| Complete clean baseline + authorized valid write | Exactly one planned write, matching readback, correct expected/baseline separation. |
| Missing/failed/malformed interval, offset or gain baseline | No destructive send for that campaign; retain original failure and cause. |
| Prior niche value + failed current baseline | Stale value never authorizes either dynamic or explicit write. |
| Failed write, mismatched readback, dirty result or failed resync | Later persistent actions remain blocked despite later clean/read successes. |
| Short serial write or framing timeout | Current termination/response association remains intact. |
| Default, extended and niche plans without persistent opt-in | No new persistent writes or confirmation prompts. |
| Dry run / skipped rows / user abort | Never hardware PASS; no live serial access; honest incomplete/failure verdict. |
| Confirmations absent/wrong and out-of-range arguments | Existing pre-execution rejection remains; no command reaches the sender. |

Add focused tests to the existing Python suite and update the old clean-only
gate test so it tests the new public contract rather than asserting that two
booleans are sufficient. Keep a real loop/serial-send regression in addition
to pure admission unit tests. Do not mock the admission function under test.

## Verify, document and publish

Run from the repository root:

```powershell
python -m unittest discover -s test -p "test_*.py"
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/check_core_timing_guard.py
python scripts/generate_version.py check
git diff --check
```

Run Doxygen if changing its guide inputs. No firmware build/hardware run is
needed merely to test this host defect locally; this persistent-action
admission change is substantive, so verify the exact final implementation
commit through the existing push/PR CI jobs. There is no workflow_dispatch;
do not alter triggers or weaken checks. Fix failures and reverify the new SHA.

Update the runner guide, Unreleased changelog, audit HIL-1 disposition and
prompt-index status with exact behavioral limits and test symbols. Record
actual commands/results and final implementation SHA/terminal CI URL in the
handoff. Do not claim physical validation. Stage only your changes, commit and
push the completed fix, then confirm branch/upstream synchronization without
force-push. Do not stop after a proposal or a regression test that still fails.
