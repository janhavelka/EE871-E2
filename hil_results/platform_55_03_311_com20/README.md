# pioarduino 55.03.311 COM20 Evidence

Hardware: ESP32-S3 revision 0.2, 4 MB embedded flash, 2 MB embedded QSPI
PSRAM, EE871-E2 on DATA GPIO6 / CLOCK GPIO7. Pull-up values, level-shifter
implementation, supply voltage, cable length, and ambient conditions were not
independently measured.

Runtime stack: pioarduino `platform-espressif32` `55.03.311`,
Arduino-ESP32 `3.3.11`, ESP-IDF `5.5.5`, EE871 library `1.0.1`.

## Authoritative Result

`full_hil_final_exact_commit/ee871_20260731T094932Z/`

- Firmware source: clean commit `3bce89e`.
- Result: 184 PASS / 0 FAIL / 0 SKIP / 0 review.
- Safe self-test: 26 PASS / 0 FAIL / 1 unsupported-mode SKIP.
- Repeated CO2 stress: 500/500.
- Mixed protocol stress: 500/500.
- Address scan: only address 0 accepted as a device; its frame had valid PEC.
- Library control-byte test: 9/9 through the production driver path.
- Final state: READY, online, zero consecutive and total transport failures,
  3,109 tracked successes, persistent state clean.

The Markdown report retains the complete command ledger. The structured JSON
retains metadata and parsed final state. Redundant raw serial for this ordinary
PASS run was removed after verification.

## Retained Diagnostic Attempts

`full_hil_exact_commit/ee871_20260731T093632Z/`

- Clean firmware `a25978c`: 183 PASS / 1 review.
- The example address scanner counted ACK samples at addresses 1-7 even though
  all seven returned `0xFF` with invalid PEC. Address 0 returned a valid frame.
- Fix: discovery now requires ACK plus valid PEC. Malformed responses remain
  visible but are not reported as devices.

`full_hil_scanner_fix_exact_commit/ee871_20260731T094431Z/`

- Clean firmware `c1a9b24`: 183 PASS / 1 review.
- The corrected scan found only address 0. The example-only `libtest` duplicate
  raw bit-banger then produced eight PEC errors while the production driver
  completed 3,100 tracked transfers with zero failures.
- Fix: `libtest` now uses the production driver's bounded,
  clock-stretch-aware, PEC-validating `readControlByte()` path.

Raw serial is retained for these two distinct negative diagnostics. Their JSON
summaries retain bounded failure excerpts. They are evidence leading to the
fixes, not passing qualification results.

## Claim Boundary

This evidence covers the automated non-persistent safe, extended, niche,
trace/sniffer, and stress command groups on this board. It does not prove CO2
accuracy, calibration, electrical margins, ESP32-S2 hardware, pure ESP-IDF
hardware, physical fault injection on this platform pin, or long-soak
stability.
