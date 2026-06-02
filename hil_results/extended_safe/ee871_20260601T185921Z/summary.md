# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-06-01T18:59:21Z`
- port: `COM17`
- baud: `115200`
- dry_run: `False`
- board: `unspecified`
- target_name: `unspecified`
- operator: `unspecified`
- expected_device_address: `0`
- git_branch: `hardening/ee871-e2-industry-readiness`
- git_commit: `84a46b694c8b`
- git_worktree: `dirty`

## Counts

- PASS: `33`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `0`

## Parsed State

```json
{
  "firmware_build": "Jun  1 2026 20:57:04",
  "library_version": "0.3.0",
  "library_full": "0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)",
  "library_build": "2026-06-01 20:57:01",
  "library_commit": "84a46b6",
  "library_git_status": "clean",
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
| 1 | `version` | `safe` | `PASS` | `0.001` |  |
| 2 | `help` | `safe` | `PASS` | `0.012` |  |
| 3 | `probe` | `safe` | `PASS` | `0.013` |  |
| 4 | `read` | `safe` | `PASS` | `0.013` |  |
| 5 | `selftest` | `safe` | `PASS` | `0.506` |  |
| 6 | `drv` | `safe` | `PASS` | `0.001` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.001` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.658` |  |
| 9 | `drv` | `safe` | `PASS` | `0.001` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.001` |  |
| 11 | `stress 500` | `extended` | `PASS` | `6.483` |  |
| 12 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 13 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 14 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 15 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 16 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 17 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 18 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 19 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 20 | `read` | `extended-read-loop` | `PASS` | `0.013` |  |
| 21 | `read` | `extended-read-loop` | `PASS` | `0.014` |  |
| 22 | `probe` | `extended-cycle` | `PASS` | `0.013` |  |
| 23 | `read` | `extended-cycle` | `PASS` | `0.013` |  |
| 24 | `selftest` | `extended-cycle` | `PASS` | `0.506` |  |
| 25 | `probe` | `extended-cycle` | `PASS` | `0.014` |  |
| 26 | `read` | `extended-cycle` | `PASS` | `0.013` |  |
| 27 | `selftest` | `extended-cycle` | `PASS` | `0.506` |  |
| 28 | `probe` | `extended-cycle` | `PASS` | `0.014` |  |
| 29 | `read` | `extended-cycle` | `PASS` | `0.014` |  |
| 30 | `selftest` | `extended-cycle` | `PASS` | `0.506` |  |
| 31 | `recover` | `extended` | `PASS` | `0.016` |  |
| 32 | `drv` | `extended` | `PASS` | `0.001` |  |
| 33 | `dirty` | `extended` | `PASS` | `0.001` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
