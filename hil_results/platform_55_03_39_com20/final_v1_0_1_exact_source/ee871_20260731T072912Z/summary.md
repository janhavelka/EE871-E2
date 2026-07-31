# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-07-31T07:29:12Z`
- port: `COM20`
- baud: `115200`
- dry_run: `False`
- board: `ESP32-S3-PSRAM`
- target_name: `ex_bringup_s3-final-source`
- operator: `Codex`
- expected_device_address: `0`
- git_branch: `main`
- git_commit: `2ee66cf06de9`
- git_worktree: `dirty`

## Counts

- PASS: `144`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `0`

## Parsed State

```json
{
  "firmware_build": "Jul 31 2026 09:28:51",
  "library_version": "1.0.1",
  "library_full": "1.0.1 (2ee66cf, 2026-07-31 09:28:49, dirty)",
  "library_build": "2026-07-31 09:28:49",
  "library_commit": "2ee66cf",
  "library_git_status": "dirty",
  "last_selftest": {
    "pass": 26,
    "fail": 0,
    "skip": 1
  },
  "driver_state": "READY",
  "online": true,
  "consecutive_failures": 0,
  "total_success": 2094,
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
| 3 | `probe` | `safe` | `PASS` | `0.015` |  |
| 4 | `read` | `safe` | `PASS` | `0.016` |  |
| 5 | `selftest` | `safe` | `PASS` | `0.484` |  |
| 6 | `drv` | `safe` | `PASS` | `0.0` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.641` |  |
| 9 | `drv` | `safe` | `PASS` | `0.0` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 11 | `stress 500` | `extended` | `PASS` | `6.469` |  |
| 12 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 13 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 14 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 15 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 16 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 17 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 18 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 19 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 20 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 21 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 22 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 23 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 24 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 25 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 26 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 27 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 28 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 29 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 30 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 31 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 32 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 33 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 34 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 35 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 36 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 37 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 38 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 39 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 40 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 41 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 42 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 43 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 44 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 45 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 46 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 47 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 48 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 49 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 50 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 51 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 52 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 53 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 54 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 55 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 56 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 57 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 58 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 59 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 60 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 61 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 62 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 63 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 64 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 65 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 66 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 67 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 68 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 69 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 70 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 71 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 72 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 73 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 74 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 75 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 76 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 77 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 78 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 79 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 80 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 81 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 82 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 83 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 84 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 85 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 86 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 87 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 88 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 89 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 90 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 91 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 92 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 93 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 94 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 95 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 96 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 97 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 98 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 99 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 100 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 101 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 102 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 103 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 104 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 105 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 106 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 107 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 108 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 109 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 110 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 111 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 112 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 113 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 114 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 115 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 116 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 117 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 118 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 119 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 120 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 121 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 122 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 123 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 124 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 125 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 126 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 127 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 128 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 129 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 130 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 131 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 132 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 133 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 134 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 135 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 136 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 137 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 138 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 139 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 140 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 141 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 142 | `recover` | `extended` | `PASS` | `0.016` |  |
| 143 | `drv` | `extended` | `PASS` | `0.0` |  |
| 144 | `dirty` | `extended` | `PASS` | `0.0` |  |

## Retention

This command ledger is the retained evidence. The duplicate generated raw
transcript and JSON were removed during the 2026-07-31 evidence condensation.
