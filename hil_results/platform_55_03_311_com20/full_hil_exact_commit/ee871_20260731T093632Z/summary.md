# EE871-E2 HIL Summary

Final verdict: `OPERATOR_REVIEW_REQUIRED`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-07-31T09:36:32Z`
- port: `COM20`
- baud: `115200`
- dry_run: `False`
- board: `ESP32-S3-4MB-2MB-QSPI-PSRAM`
- target_name: `ex_bringup_s3`
- operator: `Codex`
- expected_device_address: `0`
- git_branch: `agent/pioarduino-55-03-311-hil`
- git_commit: `a25978cff96f`
- git_worktree: `clean`

## Counts

- PASS: `183`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `1`

## Parsed State

```json
{
  "firmware_build": "Jul 31 2026 11:35:32",
  "arduino_esp32_version": "3.3.11",
  "esp_idf_version": "v5.5.5",
  "library_version": "1.0.1",
  "library_full": "1.0.1 (a25978c, 2026-07-31 11:35:30, clean)",
  "library_build": "2026-07-31 11:35:30",
  "library_commit": "a25978c",
  "library_git_status": "clean",
  "last_selftest": {
    "pass": 26,
    "fail": 0,
    "skip": 1
  },
  "driver_state": "READY",
  "online": true,
  "consecutive_failures": 0,
  "total_success": 3100,
  "total_failures": 0,
  "persistent_config_dirty": false,
  "resync_needed": false,
  "last_stress": {
    "kind": "stress_mix",
    "total": 500,
    "success": 500,
    "errors": 0
  },
  "device_address": 0,
  "measurement_interval_ds": 150
}
```

## Commands

| # | Command | Group | Result | Elapsed s | Reason |
| --- | --- | --- | --- | --- | --- |
| 1 | `version` | `safe` | `PASS` | `0.0` |  |
| 2 | `help` | `safe` | `PASS` | `0.0` |  |
| 3 | `probe` | `safe` | `PASS` | `0.0` |  |
| 4 | `read` | `safe` | `PASS` | `0.016` |  |
| 5 | `selftest` | `safe` | `PASS` | `0.5` |  |
| 6 | `drv` | `safe` | `PASS` | `0.0` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.015` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.641` |  |
| 9 | `drv` | `safe` | `PASS` | `0.0` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 11 | `stress 500` | `extended` | `PASS` | `6.422` |  |
| 12 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 13 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 14 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 15 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 16 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 17 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 18 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 19 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 20 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 21 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 22 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 23 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 24 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 25 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 26 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 27 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 28 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 29 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 30 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 31 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 32 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 33 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 34 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 35 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 36 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 37 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 38 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 39 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 40 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 41 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 42 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 43 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 44 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 45 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 46 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 47 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 48 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 49 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 50 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 51 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 52 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 53 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 54 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 55 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 56 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 57 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 58 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 59 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 60 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 61 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 62 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 63 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 64 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 65 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 66 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 67 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 68 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 69 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 70 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 71 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 72 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 73 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 74 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 75 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 76 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 77 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 78 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 79 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 80 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 81 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 82 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 83 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 84 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 85 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 86 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 87 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 88 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 89 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 90 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 91 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 92 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 93 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 94 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 95 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 96 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 97 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 98 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 99 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 100 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 101 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 102 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 103 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 104 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 105 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 106 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 107 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 108 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 109 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 110 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 111 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 112 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 113 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 114 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 115 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 116 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 117 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 118 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 119 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 120 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 121 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 122 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 123 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 124 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 125 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 126 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 127 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 128 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 129 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 130 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 131 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 132 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 133 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 134 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 135 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 136 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 137 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 138 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 139 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 140 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 141 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 142 | `recover` | `extended` | `PASS` | `0.016` |  |
| 143 | `drv` | `extended` | `PASS` | `0.016` |  |
| 144 | `dirty` | `extended` | `PASS` | `0.0` |  |
| 145 | `id` | `niche-identity` | `PASS` | `0.031` |  |
| 146 | `status` | `niche-identity` | `PASS` | `0.0` |  |
| 147 | `co2fast` | `niche-identity` | `PASS` | `0.016` |  |
| 148 | `co2avg` | `niche-identity` | `PASS` | `0.016` |  |
| 149 | `error` | `niche-identity` | `PASS` | `0.016` |  |
| 150 | `fw` | `niche-identity` | `PASS` | `0.031` |  |
| 151 | `e2spec` | `niche-identity` | `PASS` | `0.016` |  |
| 152 | `features` | `niche-identity` | `PASS` | `0.047` |  |
| 153 | `caps` | `niche-identity` | `PASS` | `0.016` |  |
| 154 | `serial` | `niche-identity` | `PASS` | `0.109` |  |
| 155 | `partname` | `niche-identity` | `PASS` | `0.11` |  |
| 156 | `addr` | `niche-config` | `PASS` | `0.015` |  |
| 157 | `interval` | `niche-config` | `PASS` | `0.015` |  |
| 158 | `factor` | `niche-config` | `PASS` | `0.015` |  |
| 159 | `filter` | `niche-config` | `PASS` | `0.016` |  |
| 160 | `mode` | `niche-guards` | `PASS` | `0.0` |  |
| 161 | `addr 8` | `niche-guards` | `PASS` | `0.0` |  |
| 162 | `interval 149` | `niche-guards` | `PASS` | `0.0` |  |
| 163 | `interval 36001` | `niche-guards` | `PASS` | `0.0` |  |
| 164 | `mode 4` | `niche-guards` | `PASS` | `0.0` |  |
| 165 | `drv` | `niche-guards` | `PASS` | `0.0` |  |
| 166 | `dirty` | `niche-guards` | `PASS` | `0.0` |  |
| 167 | `buscheck` | `niche-bus` | `PASS` | `0.0` |  |
| 168 | `levels` | `niche-bus` | `PASS` | `0.0` |  |
| 169 | `clocktest` | `niche-bus` | `PASS` | `0.0` |  |
| 170 | `scan` | `niche-bus` | `OPERATOR_REVIEW_REQUIRED` | `0.953` | expected output token missing |
| 171 | `timing` | `niche-bus` | `PASS` | `0.547` |  |
| 172 | `libtest` | `niche-bus` | `PASS` | `0.235` |  |
| 173 | `diag` | `niche-bus` | `PASS` | `2.766` |  |
| 174 | `trace clear` | `niche-trace` | `PASS` | `0.0` |  |
| 175 | `verbose 1` | `niche-trace` | `PASS` | `0.0` |  |
| 176 | `status` | `niche-trace` | `PASS` | `0.016` |  |
| 177 | `trace stats` | `niche-trace` | `PASS` | `0.0` |  |
| 178 | `verbose 0` | `niche-trace` | `PASS` | `0.0` |  |
| 179 | `sniff` | `niche-trace` | `PASS` | `0.0` |  |
| 180 | `status` | `niche-trace` | `PASS` | `0.0` |  |
| 181 | `sniff` | `niche-trace` | `PASS` | `0.0` |  |
| 182 | `stress_mix 500` | `niche-stress` | `PASS` | `6.391` |  |
| 183 | `drv` | `niche-final` | `PASS` | `0.0` |  |
| 184 | `dirty` | `niche-final` | `PASS` | `0.0` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
