# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-07-30T10:59:54Z`
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

- PASS: `33`
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
  "total_success": 1418,
  "total_failures": 0,
  "persistent_config_dirty": false,
  "resync_needed": false,
  "last_stress": {
    "kind": "stress",
    "total": 500,
    "success": 500,
    "errors": 0
  }
}
```

## Commands

| # | Command | Group | Result | Elapsed s | Reason |
| --- | --- | --- | --- | --- | --- |
| 1 | `version` | `safe` | `PASS` | `0.0` |  |
| 2 | `help` | `safe` | `PASS` | `0.016` |  |
| 3 | `probe` | `safe` | `PASS` | `0.016` |  |
| 4 | `read` | `safe` | `PASS` | `0.016` |  |
| 5 | `selftest` | `safe` | `PASS` | `0.515` |  |
| 6 | `drv` | `safe` | `PASS` | `0.0` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.641` |  |
| 9 | `drv` | `safe` | `PASS` | `0.0` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 11 | `stress 500` | `extended` | `PASS` | `6.469` |  |
| 12 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 13 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 14 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 15 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 16 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 17 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 18 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 19 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 20 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 21 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 22 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 23 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 24 | `selftest` | `extended-cycle` | `PASS` | `0.515` |  |
| 25 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 26 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 27 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 28 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 29 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 30 | `selftest` | `extended-cycle` | `PASS` | `0.515` |  |
| 31 | `recover` | `extended` | `PASS` | `0.016` |  |
| 32 | `drv` | `extended` | `PASS` | `0.0` |  |
| 33 | `dirty` | `extended` | `PASS` | `0.0` |  |

## Retention

This command ledger is the retained evidence. The duplicate generated raw
transcript and JSON were removed during the 2026-07-31 evidence condensation.
