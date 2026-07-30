# COM20 Post-Power-Cycle Stability/Stale Follow-Up

Date: 2026-07-30

## Evidence Classification And Limitations

The strict 625-second capture in `post_power_cycle_stability_20260730.txt`
started its first
sample approximately 66 seconds after MCU boot, as inferred from its final MCU
timestamps. It therefore characterizes 10-minute post-power-cycle stability,
not the EE871's immediate documented 5-10 second warm-up window. A separate
operator-confirmed sensor/MCU power cycle captured the post-COM-reappearance
transition afterward; its timing limitations and clean raw PASS are retained in
`immediate_warmup_attempts_20260730.md` and
`immediate_warmup_final_20260730.txt`.

The strict validator required all 63 scheduled CLI commands to return OK.
One `co2fast` operation returned a bounded `NACK` at t=330 seconds; the other
62 scheduled CLI commands succeeded. The immediately following `co2avg` and
`status` operations also succeeded, subsequent scheduled samples succeeded,
and final state was READY with zero consecutive failures. The recorded result
therefore remains `FAIL`.

The raw log's phrase "10-second status age" is imprecise. The documented
condition concerns the age of the last measurement when status is read, not
the age of a previous status read. The sequence below is retained only as an
observational timing/sample-evolution check and does not prove internal
freshness behavior.

## Follow-Up

The values below are a manually normalized summary of interactive serial output
observed immediately after the strict capture. No raw follow-up serial
transcript was retained, so these values are secondary evidence rather than a
raw artifact.

Focused reproduction was run immediately afterward without resetting the MCU or
clearing lifetime health counters:

- Initial state: READY, consecutive failures 0, total success 114, total
  failures 1, dirty clean.
- `stress_mix 1000`: 1000/1000, zero errors, +1875 tracked successes,
  +0 failures, 12.777 seconds.
- `stress 1000`: 1000/1000, zero errors, +2000 tracked successes,
  +0 failures, 12.941 seconds.
- Selftest: 27/27, zero failures/skips.
- Final state: READY, consecutive failures 0, total success 4061, total
  failures 1, dirty clean.

The isolated NACK was not reproduced in the next 2,000 stress operations. No
automatic retry was added to the core: the library returned the precise bounded
transport error, tracked it, and returned to READY on the next successful
operation. Application retry cadence remains application-owned.

The status/stale characterization within the strict capture completed:

- t=600.031 s: status OK, `0x00`.
- t=607.046 s: averaged CO2 OK, `634 ppm`, seven seconds after status.
- t=618.062 s: averaged CO2 OK, `634 ppm`, after an additional 11 seconds.
- t=618.078 s: status OK, `0x00`.
- t=625.093 s: averaged CO2 OK, `626 ppm`, seven seconds after status.

These values demonstrate bounded, successful command timing and observable
sample evolution. They do not prove sample freshness internals, CO2 accuracy,
or calibration accuracy.
