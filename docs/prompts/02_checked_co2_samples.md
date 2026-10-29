# Prompt 02 — Add explicit checked CO2 samples

**Finding:** CAP-1. **Status: IMPLEMENTED.** Selected additive capability.
Invoking this prompt selects implementation of checked sample APIs and their
native/Arduino diagnostic callers. The current raw API is intentionally raw;
do not describe its lack of status checking as a regression. Complete the
selected software feature rather than asking again whether to add it. If an
equivalent feature already exists, verify and use it instead of duplicating it.
No hardware tests, serial access or physical calibration are in scope.

Implemented on 2026-10-29 with explicit `readCo2FastSample()` and
`readCo2AverageSample()` APIs, fixed-size per-step evidence, appended
`CO2_SENSOR_ERROR`, and manual Arduino/native IDF diagnostics. Raw APIs,
frame-level retry rules and transport-health ownership are preserved. Checked
success means successful requested transfers and clear sensor status; the
application still owns warm-up, freshness, accuracy and range. The execution
handoff records actual commands/results, final SHA and exact terminal CI URL;
no hardware qualification or release is claimed. The instructions below remain
the selected scope and acceptance contract.

## Inspect and reproduce the actual contract gap

Read AGENTS.md, docs/prompts/README.md, audit CAP-1, README sampling/health
contracts, the bundled protocol guide and .github/workflows/ci.yml. Fetch and
inspect the existing branch/upstream, fast-forward only safely, preserve
concurrent work and keep this prompt set on that branch. The lead owns shared
Git mutations. Prepared source baseline:
`d37b586e35e553af2a65197282d6d478e554733b`; re-audit current HEAD.

Inspect include/EE871/EE871.h and Status.h; in src/EE871.cpp inspect
`readU16`, `readCo2Fast`, `readCo2Average`, `readStatus`, `readErrorCode`,
`_readControlByteTracked`, `_updateHealth` and feature predicates. Read
test/support/FakeE2Transport.h and tests
`test_measurement_reads_cover_values_boundaries_and_high_byte_failure`,
`test_retry_never_replays_identity_custom_reads_or_writes`,
`test_high_byte_retry_preserves_latch_and_sticky_low_event`, and current
support/status tests in test/test_basic.cpp. Inspect both CLI command handlers,
tools/check_cli_contract.py, tools/check_idf_example_contract.py and the runner
parser before adding any output/command surface.

Historical reference: `git show` commit
`82b656ecb2010e92729e2b71ece5fe13c4c2e766` and final series tip
`dae9bea1155db4efb757a37a99ed7b507d718e6c` for `Co2ReadResult`,
`_readCo2Sample` and corresponding tests. Review trigger wording correction
`5df2515a46c604ee5dcbe5da02c7950a5bc3169e`. Do not import the old hard-coded
50,000 ppm limit or replace current raw/retry behavior.

Use the fake to show a successful MV3/MV4 raw value while status bit 3 is set.
Raw success is expected; the new checked API must retain that value as evidence
but reject it as a clean checked sample. Add a failing behavioral test for the
new API, not a test that falsely demands raw `readCo2Fast()` start reading status.

## Implement one small checked procedure

Add fixed-size, documented result evidence and two explicit checked entry
points, using existing raw/typed methods for all protocol and transport work.
Use the existing class as owner. Share the common MV3/MV4 procedure; do not
duplicate the two-byte, PEC, health, retry or capability logic. Avoid a generic
sampling framework, hidden scheduler, new cache service or heap allocation.

The procedure is value low byte, high byte, then status, then optional error
detail only when status reports a CO2 error and the device advertises it.
Short-circuit after a failed value read; do not perform a side-effecting status
read to improve partial evidence. If a later step fails, preserve successful
earlier evidence and exact code/detail for the failing step. Use explicit
attempted/valid fields so default OK status never proves an unattempted step ran.

- Expose raw value, status byte/validity, optional error code/validity and each
  attempted step's Status. Keep names/semantics small and unambiguous.
- Append any needed sensor-domain error code after existing Err values; do not
  renumber them or repurpose BUSY/E2_ERROR as an unrelated sensor status result.
  Unknown sensor error codes remain visible, not silently successful.
- Sensor-domain failures must not add a transport-health failure. Existing
  tracked reads still count their real transfers and retry outcomes exactly
  once per final frame; a helper adds no extra aggregate health event.
- A clean checked result means successful requested transfers and clear sensor
  status. It does not certify warm-up, freshness, concentration accuracy,
  application range or the board's product model. Do not add a guessed universal
  ppm limit. If an application needs a narrower range, show explicit caller
  validation after the helper and preserve the distinction in documentation.
- Keep existing raw methods unchanged. Document that status can trigger the
  next measurement only under the existing interval/age conditions, and that
  value-before-status ordering does not prove a fresh conversion.
- Add narrow explicit diagnostic callers in both Arduino and native IDF CLIs
  with matching command/help/output semantics, so the new API has a real user.
  Reuse each example's existing parser; do not share Arduino implementation
  into IDF. Update contract/parser tests only where those outputs are consumed.
  Do not add the new commands to live automated plans as hidden side effects.

## Required behavior matrix

| Scenario | Expected observation |
| --- | --- |
| Clean MV3/MV4, values 0 and 65535 | Exact raw word retained; clean sensor status gives checked success without invented product-range policy. |
| Low-byte or high-byte failure | Exact transfer error; no status/error-code call after value failure; no partial valid ppm. |
| Status NACK/PEC/timeout/STOP failure | Successful ppm evidence retained, checked success false, exact status failure recorded. |
| Status bit 3 with supported error detail | Codes 1, 200, 201, 202 and unknown values preserved; sensor-domain failure independent of transport health. |
| Status error with no error-code capability | Sensor-domain failure; no unsupported custom read; unattempted detail is explicit. |
| Error-detail read failure | Preserve sensor-status evidence and transport error separately; never mark ppm checked-valid. |
| UNINIT, OFFLINE and invalid invocation | Precise current precondition behavior; no unauthorized bus traffic. |
| Retry count 0 and 3, high-byte retry, veto/cleanup failure | Reuse existing narrow retry rules, low-byte latch and sticky diagnostics; no helper-level replay/reset. |
| Same result object reused after success/failure | No stale attempted/valid fields survive into a new result. |

Register native tests in the existing Unity runner. Prefer test callbacks that
assert transaction order and health deltas over tests that mirror private fields.

## Verification and closeout

```powershell
.\scripts\pio.cmd test -e native
python -m unittest discover -s test -p "test_*.py"
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
.\scripts\pio.cmd run -e ex_bringup_s3
.\scripts\pio.cmd run -e ex_bringup_s2
.\scripts\pio.cmd run -e compat_tunnelmonitor_s3
doxygen Doxyfile
git diff --check
```

Windows uses the managed PlatformIO wrapper; never install another Core.
Build native IDF 6.0.1 for both targets locally when available using
`idf.py -C examples/idf/basic_bringup set-target esp32s3 build` and `esp32s2`.
Report unavailable builds as NOT RUN and verify both jobs in existing CI.
This public/API integration change requires terminal CI for the exact final
implementation commit via existing push/PR triggers. No manual dispatch trigger
exists; do not change it. Correct failures and reverify after source changes.

Update public Doxygen, README sampling examples, both CLI guides as needed,
Unreleased changelog, CAP-1 disposition and prompt status. State the deliberately
limited validity claim and actual tests; no hardware qualification. Stage only
your work, commit and push, verify synchronization, and provide the final SHA,
CI URL and concise compatibility/behavior evidence. No automatic release/tag or
unrelated version rewrite is part of this prompt.
