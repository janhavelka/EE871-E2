# Prompt 03 — Make optional long maintenance waits cooperative

**Finding:** CAP-2. **Status: NOT IMPLEMENTED.** Optional additive capability.
Invoking this prompt selects long-wait callback support and example adapters.
Current delays are finite; the selected improvement is scheduler cooperation
during post-write quiet waits. Do not invent an unboundedness bug or introduce
an asynchronous driver engine. Complete the software change without hardware
tests, serial access, scheduler measurements on devices or real writes.

## Audit the current boundaries first

Read AGENTS.md, docs/prompts/README.md, audit CAP-2, README timing contracts,
docs/IDF_PORT.md, and .github/workflows/ci.yml. Fetch and inspect the existing
branch/upstream, fast-forward only safely, preserve unrelated changes and stay
on the existing prompt-set branch. Shared Git mutations belong to the lead.
Prepared baseline is `d37b586e35e553af2a65197282d6d478e554733b`; use actual HEAD.

Inspect include/EE871/Config.h; `sleepMs`, `begin`, `_customWriteDirect`,
`writeMeasurementInterval`, `_readControlByteTracked`, `waitSclHigh`,
`e2Start`, `e2Stop`, `writeByte`, `readByte`, `readAck`, `sendAck` in
src/EE871.cpp. Inspect examples/common/E2Transport.h,
examples/idf/common/E2GpioTransport.h and both CLI configuration builders.
Read FakeE2Transport, nominal/deadline tests and
`test_flash_write_stop_stretches_commit_and_verify` in test/test_basic.cpp.

Historical callback designs are in commits
`d6100b01723a95b618110dc4696c64ad3977d2c2`,
`739beeca9017bc678f026d4b466aa2bb6332b08e`, and
`50702f3805f2a24f300fd189e0992b35ed96a714` (Config, sleep helper, adapters).
Review with `git show`; do not restore their old pointer-write flash timing or
configuration normalization. Current zero write-delay values are accepted and
used by host tests; silently increasing them is not this prompt's scope.

Instrument a native fake successful persistent write with a nonzero configured
quiet wait. Show that current `sleepMs` requests one `delayUs(1000)` per
millisecond and that there is no separate cooperative callback. This establishes
the feature boundary; do not call it a failed protocol transaction.

## Implement the smallest useful callback contract

Extend the existing Config with optional task-context long-delay/yield support,
appending fields after all current members, including flashStretchTimeoutUs,
readNackRetries and allowReadRetry. Preserve existing positional aggregate
initializers and default behavior. Use one shared post-write wait helper and
remove duplicated or obsolete long-wait code it replaces.

- The long-delay callback receives a bounded positive millisecond slice and
  must wait at least that amount while remaining bounded/non-reentrant. Use a
  small documented finite default slice. If configurable, validate zero and
  maximum explicitly; never create a zero-progress loop.
- The optional yield callback is invoked only at documented long-wait slice
  boundaries. If both exist, define the order. No new callback is required for
  existing users; fallback uses the existing bounded microsecond delays.
- Sum requested slice delays exactly to the configured quiet wait. Preserve
  zero waits, final partial slices, existing max-5000-ms settings and the
  separate 150/300-ms default configuration values.
- Do not yield, schedule, change ownership or stretch nominal phases inside
  START/STOP/bit/byte/ACK/PEC signaling or SCL-release polling. A flash STOP
  stretch is protocol signaling, not a post-write quiet wait.
- Keep readNackRetries' fixed 1000-us pause and veto points unchanged. Keep
  ordinary reads/volatile 0x50 pointer STOPs on the ordinary timeout and direct
  flash writes/reset on their current separate allowance.
- Callbacks are application-owned, task-context, externally serialized and
  cannot call the same driver recursively. A void callback contract cannot
  prove wall-clock adherence or independently detect callback failures; state
  that limitation instead of inventing cancellation or watchdog guarantees.
- Add real example adapters: Arduino can use its normal task delay/yield;
  native IDF can use FreeRTOS waits with rounding that never under-waits the
  requested milliseconds. Core must contain no framework headers or delay
  calls. Do not share Arduino code into IDF or add a new service abstraction.

Do not add timeout APIs, hidden retry/reset, persistent-state changes or a
background owner. Prompt 05 may later account for the resulting requested
delays and callback-overhead exclusions; this feature does not depend on it.

## Native tests and acceptance

Use bounded callback counters/recorders owned by the existing fake or test
fixture; no production test doubles. Register every new Unity test.

| Scenario | Assertion |
| --- | --- |
| Optional callbacks absent | Existing byte sequence/status/health/dirty semantics and fallback requested waits unchanged. |
| Delay callback alone, yield alone, both | Documented selection/order; only post-write waits invoke them. |
| Wait 0, 1, exact slice, partial final slice, 5000 ms | Exact total requested wait, finite callback counts, no zero-progress calls. |
| Invalid slice/configuration | Rejected before GPIO traffic; output/stored active session contracts preserved. |
| Read, raw pointer update, retry pause, protocol stretch | No long-wait callback/yield invoked inside those paths. |
| Persistent transfer failure before quiet wait | No unnecessary quiet wait or verify traffic; original status and uncertainty retained. |
| Quiet wait followed by readback failure | Current dirty tracking and exact transport/verify result preserved. |
| Legacy positional Config initialization | Compile-only compatibility case still binds every existing field correctly. |

A recorder showing callbacks in the right place is software evidence only;
do not claim a real scheduler/watchdog latency improvement was measured.

## Verification, documentation and publication

```powershell
.\scripts\pio.cmd test -e native
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m unittest discover -s test -p "test_*.py"
python scripts/generate_version.py check
.\scripts\pio.cmd run -e ex_bringup_s3
.\scripts\pio.cmd run -e ex_bringup_s2
.\scripts\pio.cmd run -e compat_tunnelmonitor_s3
doxygen Doxyfile
git diff --check
```

Use scripts/pio.cmd on Windows; do not install another Core. Native IDF 6.0.1
commands are `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`
and the same for esp32s2. Mark missing local SDK checks NOT RUN and require the
exact final implementation commit's corresponding CI jobs to finish green.
This API/adapter change is substantial: verify the existing push/PR CI run for
that SHA, fix confirmed failures, do not modify triggers or weaken checks.

Update Config/public docs, timing/IDF guide, relevant examples, Unreleased
changelog, audit CAP-2 and prompt status. Mention preserved defaults, bounded
callback assumptions and no physical qualification. Stage only owned changes,
commit and push the complete feature, verify synchronization without force and
return final SHA, checks/limitations and terminal CI URL. Do not silently bump
release metadata or tag a release.
