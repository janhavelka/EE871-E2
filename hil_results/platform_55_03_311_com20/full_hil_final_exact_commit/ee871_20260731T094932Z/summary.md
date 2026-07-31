# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-07-31T09:49:32Z`
- port: `COM20`
- baud: `115200`
- dry_run: `False`
- board: `ESP32-S3-4MB-2MB-QSPI-PSRAM`
- target_name: `ex_bringup_s3`
- operator: `Codex`
- expected_device_address: `0`
- git_branch: `agent/pioarduino-55-03-311-hil`
- git_commit: `3bce89eee9cd`
- git_worktree: `clean`

## Counts

- PASS: `184`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `0`

## Parsed State

```json
{
  "firmware_build": "Jul 31 2026 11:48:48",
  "arduino_esp32_version": "3.3.11",
  "esp_idf_version": "v5.5.5",
  "library_version": "1.0.1",
  "library_full": "1.0.1 (3bce89e, 2026-07-31 11:48:47, clean)",
  "library_build": "2026-07-31 11:48:47",
  "library_commit": "3bce89e",
  "library_git_status": "clean",
  "last_selftest": {
    "pass": 26,
    "fail": 0,
    "skip": 1
  },
  "driver_state": "READY",
  "online": true,
  "consecutive_failures": 0,
  "total_success": 3109,
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
| 5 | `selftest` | `safe` | `PASS` | `0.515` |  |
| 6 | `drv` | `safe` | `PASS` | `0.0` |  |
| 7 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 8 | `stress 50` | `safe` | `PASS` | `0.641` |  |
| 9 | `drv` | `safe` | `PASS` | `0.0` |  |
| 10 | `dirty` | `safe` | `PASS` | `0.0` |  |
| 11 | `stress 500` | `extended` | `PASS` | `6.391` |  |
| 12 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 13 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 14 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 15 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 16 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 17 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 18 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 19 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 20 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 21 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 22 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 23 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 24 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 25 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 26 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 27 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 28 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 29 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 30 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 31 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 32 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 33 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 34 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 35 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 36 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 37 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 38 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 39 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 40 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 41 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 42 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 43 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 44 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 45 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 46 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 47 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 48 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 49 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 50 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 51 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 52 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 53 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 54 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 55 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 56 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 57 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 58 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 59 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 60 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 61 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 62 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 63 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 64 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 65 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 66 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 67 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 68 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 69 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 70 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 71 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 72 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 73 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 74 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 75 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 76 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 77 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 78 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 79 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 80 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 81 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 82 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 83 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 84 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 85 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 86 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 87 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 88 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 89 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 90 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 91 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 92 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 93 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 94 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 95 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 96 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 97 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 98 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 99 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 100 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 101 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 102 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 103 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 104 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 105 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 106 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 107 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 108 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 109 | `read` | `extended-read-loop` | `PASS` | `0.016` |  |
| 110 | `read` | `extended-read-loop` | `PASS` | `0.015` |  |
| 111 | `read` | `extended-read-loop` | `PASS` | `0.0` |  |
| 112 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 113 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 114 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 115 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 116 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 117 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 118 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 119 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 120 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 121 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 122 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 123 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 124 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 125 | `read` | `extended-cycle` | `PASS` | `0.015` |  |
| 126 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 127 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 128 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 129 | `selftest` | `extended-cycle` | `PASS` | `0.485` |  |
| 130 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 131 | `read` | `extended-cycle` | `PASS` | `0.0` |  |
| 132 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 133 | `probe` | `extended-cycle` | `PASS` | `0.015` |  |
| 134 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 135 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 136 | `probe` | `extended-cycle` | `PASS` | `0.016` |  |
| 137 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 138 | `selftest` | `extended-cycle` | `PASS` | `0.5` |  |
| 139 | `probe` | `extended-cycle` | `PASS` | `0.0` |  |
| 140 | `read` | `extended-cycle` | `PASS` | `0.016` |  |
| 141 | `selftest` | `extended-cycle` | `PASS` | `0.484` |  |
| 142 | `recover` | `extended` | `PASS` | `0.016` |  |
| 143 | `drv` | `extended` | `PASS` | `0.0` |  |
| 144 | `dirty` | `extended` | `PASS` | `0.0` |  |
| 145 | `id` | `niche-identity` | `PASS` | `0.031` |  |
| 146 | `status` | `niche-identity` | `PASS` | `0.015` |  |
| 147 | `co2fast` | `niche-identity` | `PASS` | `0.016` |  |
| 148 | `co2avg` | `niche-identity` | `PASS` | `0.0` |  |
| 149 | `error` | `niche-identity` | `PASS` | `0.015` |  |
| 150 | `fw` | `niche-identity` | `PASS` | `0.031` |  |
| 151 | `e2spec` | `niche-identity` | `PASS` | `0.015` |  |
| 152 | `features` | `niche-identity` | `PASS` | `0.047` |  |
| 153 | `caps` | `niche-identity` | `PASS` | `0.0` |  |
| 154 | `serial` | `niche-identity` | `PASS` | `0.109` |  |
| 155 | `partname` | `niche-identity` | `PASS` | `0.125` |  |
| 156 | `addr` | `niche-config` | `PASS` | `0.015` |  |
| 157 | `interval` | `niche-config` | `PASS` | `0.031` |  |
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
| 170 | `scan` | `niche-bus` | `PASS` | `1.234` |  |
| 171 | `timing` | `niche-bus` | `PASS` | `0.547` |  |
| 172 | `libtest` | `niche-bus` | `PASS` | `0.062` |  |
| 173 | `diag` | `niche-bus` | `PASS` | `2.781` |  |
| 174 | `trace clear` | `niche-trace` | `PASS` | `0.0` |  |
| 175 | `verbose 1` | `niche-trace` | `PASS` | `0.0` |  |
| 176 | `status` | `niche-trace` | `PASS` | `0.015` |  |
| 177 | `trace stats` | `niche-trace` | `PASS` | `0.0` |  |
| 178 | `verbose 0` | `niche-trace` | `PASS` | `0.0` |  |
| 179 | `sniff` | `niche-trace` | `PASS` | `0.0` |  |
| 180 | `status` | `niche-trace` | `PASS` | `0.015` |  |
| 181 | `sniff` | `niche-trace` | `PASS` | `0.0` |  |
| 182 | `stress_mix 500` | `niche-stress` | `PASS` | `6.359` |  |
| 183 | `drv` | `niche-final` | `PASS` | `0.0` |  |
| 184 | `dirty` | `niche-final` | `PASS` | `0.0` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
