# Prompt 05 — Expose programmatic requested-delay bounds

**Finding:** CAP-3. **Status: IMPLEMENTED in the selected software scope.** Selected additive capability.
Invoking this prompt selects a pure operation-bound API and a real diagnostic
consumer. Current manual bounds are not automatically wrong. Implement against
the operations actually present on current HEAD, after any selected sample,
long-wait or mutation prompts; do not require unselected optional APIs.
No serial access, hardware timing capture or real protocol transactions are
authorized for validation. A requested-delay bound is not measured wall time.

Implementation disposition: static `operationTimingBound()` accepts explicit
`Config`, an `OperationKind`, output `OperationTimingBound`, and bounded target/
count arguments. It returns an integer microsecond requested-delay sum, shares
numeric configuration validation with `begin()`, and requires no installed
callbacks for calculation. `bounds` in both Arduino and native IDF diagnostics
is the real bus-silent consumer. The [README derivation](../../README.md#programmatic-requested-delay-bounds)
records primitive and compound formulas, including current checked samples,
quiet waits, interval-byte special handling and all 256 possible resync targets.
Exact test, build and final-commit CI results belong in the execution handoff;
no hardware qualification is claimed. The instructions below retain the
selected scope and acceptance criteria.

## Re-audit the operation graph

Read AGENTS.md, docs/prompts/README.md, audit CAP-3, README timing/retry/resync
sections, protocol timing references and .github/workflows/ci.yml. Fetch and
inspect the existing branch/upstream, fast-forward only safely, preserve
unrelated work and remain on the same prompt-set branch. The lead owns shared
Git mutations. Prepared baseline:
`d37b586e35e553af2a65197282d6d478e554733b`; do not freeze formulas to it.

Inspect Config/CommandTable/EE871.h and src/EE871.cpp helpers `nominalBitTimeUs`,
`requireByteBudget`, `waitSclHigh`, `e2Start`, `e2Stop`, `finishWithStop`,
`sleepMs`, `_readControlByteRaw`, `_readControlByteTracked`, `_writeCommandRaw`,
`_busResetRaw`, `_validateIdentityRaw`, `_readFeatureFlagsRaw`,
`_requireCo2AdjustmentSupport` and all public compound operations. Enumerate
actual transfer/retry/delay paths, including failure cleanup, instead of merely
copying README constants. Inspect current timing/retry tests and fake elapsed
time/transfer counters before deciding what additional instrumentation is needed.

Historical sources: `git show`
`d6100b01723a95b618110dc4696c64ad3977d2c2` and
`dae9bea1155db4efb757a37a99ed7b507d718e6c` for `OperationKind`,
`OperationTimingBound`, `_calculateOperationTimingBound` and timing tests.
Those formulas treat volatile 0x50 pointer writes as flash-completion work and
predate main's read retries/calibration/resync fixes. They are test/design
references, not valid current formulas to transplant.

## Define a small pure API and prove its boundary

Use the existing driver/config vocabulary. Provide an operation selector,
bounded element count where applicable, Status-returning checked calculation,
and an unambiguous unit/type for the result. It must work before `begin()` from
explicit Config without GPIO callbacks being invoked. A convenience overload
using the active validated config is optional only if it avoids duplicate
validation/calculation and has a real caller.

Refactor shared configuration checks into one pure existing-owner helper when
needed; do not create competing validation logic or call `begin()` to validate.
Preserve every accepted current configuration, including zero post-write waits,
and reject invalid selectors/counts/overflow before publishing output.
Use checked wide arithmetic and round only at the documented final unit
boundary. Unsupported operation/count pairs return a precise status, not zero
or a deceptively small number.

The contract must say **sum of maximum requested protocol waits/delays under
the configured callback contract**. GPIO callback execution time, optional
callback overhead, scheduler preemption and latency are not bounded by this
core. Do not advertise hard wall-clock admission unless the caller supplies
and owns those additional bounds. No sleep, retry, reset, hardware access or
health/cache mutation is permitted during calculation.

Cover the concrete operation classes already exposed:

- ordinary control read, raw MV3/MV4 pair, volatile pointer update, custom block
  read (address/range/count bounded), verified custom byte write;
- interval pair, offset/gain pair with on-demand support read, fixed part-name
  write, address write, mode/factor/filter, auto-adjust preflight and request;
- strict begin/probe/recover including identity/capability loads and possible
  bounded reset, standalone reset, and persistent resync;
- checked sample methods only if Prompt 02 or an equivalent is present, including
  optional error-code path; long-delay callback requested slices if present;
- bus-silent status/configuration diagnostics either documented as zero or
  intentionally excluded by the public operation-selector contract.

Use a small explicit operation table/switch with shared primitive costs. Do not
invent a generalized protocol interpreter or add selectors for speculative
future APIs. Give the new API a real caller in existing CLI timing/diagnostic
output for both Arduino and native IDF, without performing the operations.

### Accounting that must remain correct

An ordinary read and volatile pointer write use the ordinary STOP allowance.
Direct custom writes use the separate flash STOP allowance. Reset has nine
releases plus STOP and can encounter pending flash. Byte budgets include
nominal phases; failure cleanup can still require STOP. Optional MV3/MV4/status
control-NACK retries have at most three extra frames and fixed 1-ms pauses;
identity/custom/writes never gain those retries. High-byte retry preserves the
low latch and is not a whole-pair retry. Capability reads add their actual
pointer/read costs. Resync may include every one of 256 pending targets,
conditional support reads, idle auto-adjust checks and baseline fields.

For unknown pre-init capabilities use a conservative documented path covering
every supported branch. Do not expose internal pending bitmap or assume one
target when declaring a resync worst case. The table must account for all
currently selected optional features without forcing them into the driver.

## Meaningful verification matrix

First capture independent hand-derived primitive/compound examples from current
code/spec contracts, then add tests. The native fake's requested-delay total is
useful independent execution evidence, but a single successful run is not proof
of the maximum. Exercise constructed worst/failure/retry paths too.

| Cases | Assertions |
| --- | --- |
| Default config ordinary read and pointer write | Match independently derived current costs; pointer gets no flash delay. |
| Bit/byte/flash/write-delay minimum and maximum accepted settings | No overflow/underestimate, validation agrees with begin, pure calculation. |
| Retry 0..3, exhaustion, cleanup/veto and success on last retry | Bound covers each eligible executed path; unsupported operations never include invented retries. |
| Block counts 0, 1, maximum and maximum+1; invalid selector | Precise rejection or correct bounded total; output unchanged on failure. |
| Calibrated pair/part name/interval/auto-adjust | Support, post-write quiet waits, verification and failure cleanup included. |
| Resync no targets and all 256 targets with conditional support | Conservative advertised bound covers both; no target loss hidden by formula. |
| Strict begin/probe/recover, initially non-idle bus | Correct reset/identity/capability budget and no callback from calculator itself. |
| Optional checked helper/long-wait callback/evidence features present or absent | No build dependency on unselected feature; accurate current operation composition. |
| Config/driver health snapshot before/after repeated queries | Identical state and zero GPIO/transport calls. |

Register Unity tests and update README formulas from the same reviewed
derivation. Do not generate a test by pasting the implementation expression;
use independent constants, callback traces and boundary cases.

## Verify and publish

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

Use the managed Windows wrapper, no additional Core. Native-IDF 6.0.1 commands:
`idf.py -C examples/idf/basic_bringup set-target esp32s3 build`, then esp32s2.
Unavailable local SDK/build is NOT RUN, not a static-check pass. This public API
and example integration needs the existing push/PR CI's terminal results at the
exact final implementation SHA, including both native-IDF targets. No manual
dispatch exists; do not change triggers or weaken checks. Fix and reverify any
new final implementation commit.

Update public contracts, README timing examples, CLI help/contract tests,
Unreleased changelog, CAP-3 disposition and prompt status. Explicitly state
requested-delay versus caller-owned wall-time limits and actual tests. Stage
only owned work, commit/push, verify upstream synchronization without force and
return final SHA, CI URL and remaining software limitations. Do not claim
physical timing qualification or add a release/tag.
