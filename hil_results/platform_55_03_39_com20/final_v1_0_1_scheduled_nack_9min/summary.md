# EE871-E2 Soak Summary

- Final verdict: `PASS`
- Status: `COMPLETE`
- Start UTC: `2026-07-31T07:04:12Z`
- End UTC: `2026-07-31T07:13:16Z`
- Requested duration: `0.15 h`
- Elapsed: `543.594 s`
- Sample cycles: `108`
- Serial reconnects: `0`
- Commands: PASS `564`, SCHEDULED_CONTROL_NACK_RECOVERED `2`, FAIL `0`, REVIEW `0`, SKIP `0`
- Transport counter regressions: `0`
- Scheduled control-byte NACKs recovered by one bounded retry: `2`
- Raw MV3 observed range: `541..584 ppm` (first `554`, last `563`)
- Raw MV4 observed range: `556..572 ppm` (first `558`, last `563`)

This soak validates bounded transport, status/health behavior, persistent-state cleanliness, and counter monotonicity for the recorded bench interval. It does not validate CO2 accuracy or calibration.

## Retention

The compact JSON retains all command/result rows and both NACK/retry pairs,
including abnormal-response excerpts. The duplicate ordinary-PASS serial text
was removed during the 2026-07-31 evidence condensation.
