# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-07-30T10:59:33Z`
- port: `COM20`
- baud: `115200`
- dry_run: `False`
- board: `ESP32-S3_4MB_2MB-QSPI-PSRAM`
- target_name: `ex_bringup_s3-pioarduino-54.03.20-hil-fixes`
- operator: `Codex-automated-HIL`
- expected_device_address: `0`
- git_branch: `main`
- git_commit: `1fbe7d814b06`
- git_worktree: `dirty`

## Counts

- PASS: `25`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `0`

## Parsed State

```json
{
  "firmware_build": "Jul 30 2026 12:57:22",
  "library_version": "1.0.0",
  "library_full": "1.0.0 (1fbe7d8, 2026-07-30 12:57:20, dirty)",
  "library_build": "2026-07-30 12:57:20",
  "library_commit": "1fbe7d8",
  "library_git_status": "dirty",
  "last_selftest": {
    "pass": 27,
    "fail": 0,
    "skip": 0
  },
  "driver_state": "READY",
  "online": true,
  "consecutive_failures": 0,
  "total_success": 174,
  "total_failures": 0,
  "persistent_config_dirty": false,
  "resync_needed": false,
  "last_stress": {
    "kind": "stress",
    "total": 50,
    "success": 50,
    "errors": 0
  },
  "measurement_interval_ds": 150,
  "expected_measurement_interval_ds": 150,
  "co2_offset_ppm": 0,
  "expected_co2_offset_ppm": 0,
  "co2_gain": 32768,
  "expected_co2_gain": 32768
}
```

## Commands

| # | Command | Group | Result | Elapsed s | Reason |
| --- | --- | --- | --- | --- | --- |
| 1 | `version` | `safe` | `PASS` | `0.0` |  |
| 2 | `help` | `safe` | `PASS` | `0.016` |  |
| 3 | `probe` | `safe` | `PASS` | `0.016` |  |
| 4 | `read` | `safe` | `PASS` | `0.016` |  |
| 5 | `selftest` | `safe` | `PASS` | `0.531` |  |
| 6 | `drv` | `safe` | `PASS` | `0.0` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.641` |  |
| 9 | `drv` | `safe` | `PASS` | `0.0` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 11 | `dirty` | `maintenance` | `PASS` | `0.0` |  |
| 12 | `interval` | `maintenance` | `PASS` | `0.031` |  |
| 13 | `interval 150` | `maintenance` | `PASS` | `0.344` |  |
| 14 | `interval` | `maintenance` | `PASS` | `0.031` |  |
| 15 | `dirty` | `maintenance` | `PASS` | `0.0` |  |
| 16 | `resync` | `maintenance` | `PASS` | `0.203` |  |
| 17 | `dirty` | `maintenance` | `PASS` | `0.0` |  |
| 18 | `offset` | `maintenance-calibration` | `PASS` | `0.031` |  |
| 19 | `offset 0` | `maintenance-calibration` | `PASS` | `0.343` |  |
| 20 | `offset` | `maintenance-calibration` | `PASS` | `0.031` |  |
| 21 | `dirty` | `maintenance-calibration` | `PASS` | `0.0` |  |
| 22 | `gain` | `maintenance-calibration` | `PASS` | `0.031` |  |
| 23 | `gain 32768` | `maintenance-calibration` | `PASS` | `0.344` |  |
| 24 | `gain` | `maintenance-calibration` | `PASS` | `0.031` |  |
| 25 | `dirty` | `maintenance-calibration` | `PASS` | `0.0` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
