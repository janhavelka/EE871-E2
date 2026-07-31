# EE871-E2 Overnight Soak Summary

- Final verdict: `FAIL`
- Status: `COMPLETE`
- Start UTC: `2026-07-30T17:31:26Z`
- End UTC: `2026-07-31T01:31:35Z`
- Requested duration: `8.0 h`
- Elapsed: `28808.844 s`
- Sample cycles: `480`
- Serial reconnects: `29`
- Classified commands: PASS `2376`, FAIL `40`, SKIP `0`
- Original runner labels: FAIL `29`, REVIEW `11`; the 11 complete MV3
  control-byte NACKs are failures after correcting the old validator order.
- Transport counter regressions: `0`
- Raw MV3 observed range: `524..716 ppm` (first `675`, last `570`)
- Raw MV4 observed range: `536..702 ppm` (first `646`, last `555`)

This soak validates bounded transport, status/health behavior, persistent-state cleanliness, and reset/counter continuity for the recorded bench interval. It does not validate CO2 accuracy or calibration.

## Retention

The raw transcript is retained because it contains the unique 29 incomplete
HWCDC replies and 11 fully framed NACKs. Its SHA-256 is
`42986B55CDBCDE688806421AC925B91A2FAC01C67F011F18C76EE4B50727CB15`.
The compact JSON keeps the complete command/result ledger and excerpts for
abnormal rows while omitting redundant PASS excerpts.
