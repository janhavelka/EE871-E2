# Prompt 04C Post-Completion Audit

Date: 2026-07-29

Branch: `feature/ee871-hardening-series`

Published Prompt 04C commit audited:
`b3c130dc8ecf0b77d7d937d816955884482fcccb`

Prompt 04C baseline:
`d40f70a41b88584c020f9efc16889ffcade69117`

Library version: `1.1.0` (unchanged)

Runner version: `2.2` -> `2.3`

Hardware/HIL status: not run

## Outcome

The complete Prompt 04C diff, resulting runner, diagnostic command producers,
contract checks, tests, active guidance, and applicable local datasheets were
audited. Confirmed runner/evidence defects were corrected in a separate
post-completion change. No core library source, public API, pin, task, product,
schema, or firmware integration boundary changed.

Prompt 04C's intended architecture remains intact: the runner operates the
repository diagnostic CLI, safe plans are non-destructive, persistent writes
restore only through typed owners, raw memory is forensic evidence only, and
hazardous physical procedures remain explicit operator/HIL work.

## Corrections

- Checked-sample validation now enforces the library contract precisely:
  clean samples require `ppmValid=true`; CO2 sensor-error samples require
  `ppmValid=false`; attempted and unattempted error steps require coherent
  status/detail evidence; and public status names must match their exact
  numeric enum values.
- A coherent CO2 sensor error produces one sensor-domain failure reason. It is
  not accompanied by a misleading invalid-ppm failure or reclassified as a
  transport fault.
- Every destructive admission now validates the actual immutable custom-memory
  snapshot as exactly 256 integer bytes in addition to its completeness flag
  and nonblank capture time.
- Auto-adjust requires a freshly successful idle `autoadj` row immediately
  before operator authorization and the one-shot command. A failed, malformed,
  missing, or running result blocks the action even when the earlier baseline
  was idle.
- Each stuck-line apply step now requires released-line and READY health
  evidence from its own fault group. Pre-fault counters are group-scoped and
  cleared after a failed capture, preventing SDA evidence from being reused
  for SCL or vice versa.
- Arduino and native ESP-IDF diagnostic text no longer claims an undocumented
  five-minute auto-adjust duration. It identifies duration as device-defined;
  D9 remains the observable running state. Both command-contract checks enforce
  this.

## Audit Coverage Added

Runner tests now directly cover:

- non-OK generic command failure without command-specific validators;
- exact public status name/code consistency;
- both supported and unsupported checked-sample sensor-error shapes and their
  contradictory step evidence;
- all required hazardous metadata fields;
- malformed and incomplete baseline images;
- restoration blocking after failed readback, mutation, or memory-diff rows;
- complete typed-baseline result capture, including explicit unsupported
  results;
- complete reversible interval-factor flow through test write, typed readback,
  mutation evidence, target-only image difference, typed restoration, restored
  readback, and final image equality;
- separate SDA-low and SCL-low line selection, both-low rejection, apply-jig
  admission, and health-snapshot group scoping;
- every generated plan variant and the absence of raw `reg write` commands.

## Final Invariants

| Area | Audited result |
| --- | --- |
| Core library | Unchanged; remains synchronous, bounded, framework-neutral, externally serialized, explicitly recoverable, and product-neutral. |
| Offline behavior | Existing 104-test production coverage passes; normal offline calls remain bus-silent and recovery remains explicit. |
| Identity/capabilities | Existing atomic fail-closed production coverage passes. |
| Checked samples | Runner evidence now matches clean and sensor-error production contracts exactly. |
| Persistent uncertainty | Typed mutation evidence remains observable; failed/uncertain operations block subsequent destructive writes and unsafe restore. |
| Baseline | Actual 256-byte image, typed values, address match, capability-aware auto-adjust state, and immutable timestamp are required. |
| Auto-adjust | Exactly one non-replayable action, gated by fresh idle evidence and operator authority; no invented duration. |
| Stuck lines | Selected line only, exact `BUS_STUCK`, matching-group health movement, fixture release, and explicit recovery. |
| Safe plans | Quick/default and complete-safe contain no destructive commands. |
| Filter | Write HIL remains unavailable pending authoritative values and a reviewed restoration procedure. |
| Raw replay/scan | No raw memory replay, address scan, hidden resync/recovery, or new framework was added. |
| Product isolation | No production firmware integration was added and the library remains product-neutral. A read-only sibling-repository scan found no TunnelMonitor EE871 dependency, E2 implementation, pins, runtime, data, settings, or commands, but did find a declaration-only `SystemResource::E2Bus` health enum reservation. |

## Validation Performed

| Command/check | Result |
| --- | --- |
| `python -m py_compile tools/ee871_hil_runner.py test/test_hil_runner_parser.py` | PASS |
| `python test/test_hil_runner_parser.py` | PASS, 73/73 |
| `python tools/check_cli_contract.py` | PASS |
| `python tools/check_idf_example_contract.py` | PASS |
| `python tools/check_core_timing_guard.py` | PASS |
| `python tools/check_public_timing_contract.py` | PASS |
| `python scripts/generate_version.py check` | PASS |
| `python -m platformio test -e native` | PASS, 104/104 |
| `python -m platformio run -e ex_bringup_s2` | PASS |
| `python -m platformio run -e ex_bringup_s3` | PASS |
| `doxygen Doxyfile` | PASS |
| Runner `--version` and `--help` | PASS; version 2.3 |
| Structured dry-plan inspection | PASS, 12/12 plan variants |
| Safe-plan and raw-write assertions | PASS; no safe destructive command and no `reg write` plan |
| `git diff --check` | PASS; line-ending conversion warnings only |
| Native ESP-IDF example build | NOT RUN; `idf.py` is unavailable on `PATH` |

The structured plan inspection covered quick/default, complete-safe,
persistent interval, nonzero factor `1`, operating mode, part name,
calibration offset/gain, address change/restoration, auto-adjust,
unplug/replug, both SDA-low and SCL-low sequences, and power cycle.

## Datasheet Check And Remaining Work

The local E2/EE871 documents were rechecked for status/error semantics, custom
memory addresses, capability bits, D9 one-shot behavior, write completion, and
clock-stretch timing. They do not define a fixed five-minute auto-adjust
duration. No new library protocol defect was found in this audit.

No serial port was opened and no live, destructive, calibration, address,
auto-adjust, unplug, stuck-line, power-cycle, waveform, network, or field HIL
was run. Those validations still require the documented dedicated hardware,
electrical safeguards, restoration authority, calibration references, and raw
evidence capture. This audit does not claim product-firmware integration,
hardware qualification, release, or HIL success.

The strict product-isolation wording is not completely satisfied by the
current sibling `TunnelMonitor-node` checkout:
`include/TunnelMonitor/contracts/Health.h` contains the declaration-only
`SystemResource::E2Bus = 12`, with one numeric-contract test. No EE871 or E2
implementation/use was found. That enum was introduced by unrelated
TunnelMonitor work and removing an append-only firmware contract is outside
Prompt 04C's library/runner scope. It was therefore not changed here; it must
be resolved deliberately in the firmware repository if the rule requires even
reserved declaration-only health identifiers to be absent.
