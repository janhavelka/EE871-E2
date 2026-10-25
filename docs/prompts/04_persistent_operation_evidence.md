# Prompt 04 — Add bounded persistent-operation evidence

**Finding:** CAP-4. **Status: NOT IMPLEMENTED.** Optional additive capability.
Invoking this prompt selects a small latest-operation evidence contract plus
explicit application maintenance admission. Main already retains persistent
uncertainty; this is not a repair for nonexistent dirty tracking. Complete the
selected software scope; no hardware access, real writes, calibration, address
changes, automatic restore or additional HIL campaign types are authorized.

## Re-audit ownership and existing guarantees

Read AGENTS.md, docs/prompts/README.md, audit CAP-4, README persistent-write
contracts, protocol/runner guides and .github/workflows/ci.yml. Fetch and inspect
the current branch/upstream, fast-forward only safely, preserve concurrent work
and keep the prompt set on its existing branch. The lead owns shared Git
mutations. Prepared baseline:
`d37b586e35e553af2a65197282d6d478e554733b`; actual HEAD may include other prompts.

Inspect include/EE871/EE871.h and Config/Status; in src/EE871.cpp inspect
`_markPersistentConfigDirty`, `_clearPersistentConfigDirty`, `_resetStoppedState`,
`_writeCommandRaw`, `_writeCommandTracked`, `_customWriteDirect`, `customWrite`,
`writeMeasurementInterval`, `writePartName`, `writeCo2Offset`, `writeCo2Gain`,
`writeBusAddress`, `startAutoAdjust`, and `resyncPersistentConfig`.
The existing pending bitmap and first error are the owner of unresolved state.
Read test/support/FakeE2Transport.h and native tests for single/raw uncertainty,
interval low/high failure, multiple pending targets, end/begin/recovery retention,
lost capability, address/mode validation and uncertain auto-adjust resync.

Historical design/tests: `git show`
`50702f3805f2a24f300fd189e0992b35ed96a714`,
`fed97e657e9b0e3cf27bc6eca27c39fd727c7339`, and
`d40f70a41b88584c020f9efc16889ffcade69117` (EE871.h/src/test).
Compare with current fixes from
`3d32ac30cc4b228299d3f0feaffd538c031f5276`. Do not import the large historical
MutationIntent/MutationDiagnostic framework or replace current multi-target
tracking with one historical target.

Reproduce with the native fake: request an interval change, fail after the low
byte may have applied, recover communication, then read coherent values which
differ from the requested interval. Existing resync may correctly clear dirty
because it proves coherence only. Show why a caller needs separate operation
intent/effect evidence; do not assert that coherence-only resync is a bug.

## Design and implement one precise evidence owner

Add a fixed-size latest persistent-operation result accessible without bus I/O,
or equivalent additive result overloads that preserve existing signatures.
Choose one simple consistent public form and use it across existing typed
mutation methods; do not create both parallel forms without a caller.

Minimum information: operation/target address range, whether an effectful
attempt occurred, requested and observed evidence with explicit validity,
definite no-effect versus possible/acknowledged effect versus verified result,
and original error. For multi-byte values use fixed capacity sufficient for
the actual largest supported typed target (part name is 16 bytes); do not add
generic allocation, arbitrary command objects or an unbounded history.

Make the outer public mutation the one operation boundary. Internal byte-write
helpers accumulate local evidence without overwriting it as separate user
operations. Centralize phase classification in the existing transport helper
which knows whether PEC/ACK/STOP was observed; do not infer physical acceptance
merely from a Status name in each wrapper. Remove duplicate bookkeeping when
refactoring those paths.

- Preserve every original code/detail and current tracking cadence. Before PEC
  starts, definite rejection can mean no effect; once it starts, failure is
  uncertain unless a final PEC NACK was definitely observed. Low-byte success
  followed by high-byte failure remains a partial operation.
- Successful matching readback verifies only what was read. An acknowledged
  write is not automatically verified. Observed coherent-but-different values
  must not become proof that the requested values applied.
- Keep the existing all-pending-target bitmap, sticky first error and lifecycle
  retention as the sole unresolved-state owner. Latest-operation evidence must
  never erase an earlier unresolved target. Document its bounded history scope;
  the application retains baseline/intent needed across multiple operations.
- `recover()` restores communication, not intent verification.
  `resyncPersistentConfig()` remains current-state coherence, not retrospective
  success. Preserve all-target success before clearing dirty, conditional
  capability checks, and BUSY while an uncertain adjustment is running.
- Address writes do not establish when the active address changes; preserve
  uncertainty and require an explicit application-owned candidate-session
  decision. Auto-adjust is an irreversible one-shot request: idle later does
  not prove it ran or succeeded, and no helper may replay it automatically.
- Admission belongs at the explicit application maintenance boundary. Use the
  current CLI/HIL admission owner (and Prompt 01 if runner work is selected) to
  block further opted-in mutations after unresolved evidence. Do not silently
  change all existing raw/typed core writes to reject whenever dirty: current
  compatibility supports accumulating multiple pending targets, and changing
  that contract requires separately chosen API policy, not a hidden side effect.
- Provide one real cache-only diagnostic caller in both existing CLIs and any
  narrowly needed runner parser integration. Preserve command parsing/framing
  and runtime confirmation; no automatic restore, reset, resync or new write
  command may be hidden behind the diagnostic.

If clean implementation needs a small helper for the repeated typed pair-write
logic, put it under the existing driver and make its specific target/commit
rules explicit. Do not create a generic transaction engine to handle two pairs.

## Failure matrix and acceptance

| Scenario | Required evidence and invariant |
| --- | --- |
| Precondition/capability/range rejection | No attempted effect; no I/O and no new dirty target. |
| Control/address/data NACK before possible acceptance | Definite no-effect classification where phase evidence proves it. |
| PEC transmission timeout, final ACK uncertainty, STOP failure | Possible effect; precise original error retained and target pending. |
| Definite final PEC NACK | No new uncertainty for a clean single-byte write; prior uncertainty retained. |
| Low succeeds/high fails; partial part-name write | Whole outer operation evidence identifies partial target/progress, not just last nested byte. |
| Matching and mismatching readback | Verified only on match; observed/expected evidence remains distinguishable on mismatch. |
| Two different failed operations | Latest record scoped correctly; both targets survive in existing pending bitmap. |
| Recover, end/begin, successful unrelated write | No loss of earlier pending state or original dirty cause. |
| Resync to readable but different values | Coherence can recover; no claim the original requested operation succeeded. |
| Unsupported pending calibration/part-name or malformed mode/address | Resync stays unsuccessful/dirty with accurate reason. |
| Auto-adjust already running / uncertain start | No duplicate request; BUSY gating preserved; later idle is not calibration-success evidence. |

Use existing phase injection controls, extend only missing observations and
register tests in Unity. Ensure diagnostics are bus-silent and do not alter
health, retry counters or driver state. If Prompt 01 is present, assert later
destructive serial commands are not sent after uncertainty.

## Verify and close out

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

Use the Windows wrapper; do not install Core. Native-IDF 6.0.1 commands are
`idf.py -C examples/idf/basic_bringup set-target esp32s3 build` and esp32s2.
If locally unavailable report NOT RUN; verify both CI jobs. Persistent behavior
and API integration are substantial: verify the exact final implementation
commit's terminal existing push/PR CI, fix failures and reverify the new SHA.
No workflow_dispatch exists; do not alter workflow triggers or weaken checks.

Update public docs, README/runner limits, Unreleased changelog, CAP-4 disposition
and index status with meaningful test symbols and current compatibility choices.
Do not mark HIL-1 fixed merely because the driver exposes more evidence; its
sender admission still needs Prompt 01. Stage only owned changes, commit/push,
confirm synchronization without force, and return exact SHA/CI/actual checks.
Physical proof of write/calibration outcomes remains outside this prompt.
