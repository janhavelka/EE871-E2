# EE871-E2 Persistent Configuration Validation

Final verdict: `PASS`

## Summary

```json
{
  "baseline_interval_ds": 150,
  "test_interval_ds": 160,
  "restored_interval_ds": 150,
  "persistent_values_read_final": {
    "measurement_interval_ds": 150,
    "co2_interval_factor": 85,
    "operating_mode_hex": "0x55",
    "measure_mode": "low power",
    "priority": "measurement",
    "bus_address": 0,
    "part_name_ascii": "EE871",
    "serial_ascii": "1920935602368A..",
    "serial_hex": "31 39 32 30 39 33 35 36 30 32 33 36 38 41 00 00",
    "co2_offset_ppm": 0,
    "co2_gain": 32768,
    "persistent_config_dirty": false,
    "resync_needed": false,
    "driver_state": "READY",
    "online": true,
    "consecutive_failures": 0,
    "total_failures": 0
  },
  "writes_performed": [
    {
      "command": "interval 160",
      "result": "PASS",
      "reason": ""
    },
    {
      "command": "interval 150",
      "result": "PASS",
      "reason": ""
    }
  ],
  "power_cycle_persistence": "not performed; no operator power-cycle step was executed during this automated run",
  "calibration_writes": "not performed; offset/gain read-only despite broad approval to avoid unnecessary calibration mutation",
  "bus_address_write": "not performed; bus address read-only because no automated post-power-cycle retarget/recovery path was used"
}
```

## Commands

| # | Command | Group | Result | Reason |
| --- | --- | --- | --- | --- |
| 1 | `version` | `baseline` | `PASS` |  |
| 2 | `interval` | `baseline` | `PASS` |  |
| 3 | `factor` | `baseline` | `PASS` |  |
| 4 | `mode` | `baseline` | `PASS` |  |
| 5 | `addr` | `baseline` | `PASS` |  |
| 6 | `partname` | `baseline` | `PASS` |  |
| 7 | `serial` | `baseline` | `PASS` |  |
| 8 | `offset` | `baseline` | `PASS` |  |
| 9 | `gain` | `baseline` | `PASS` |  |
| 10 | `drv` | `baseline` | `PASS` |  |
| 11 | `interval 160` | `interval-write` | `PASS` |  |
| 12 | `wait 0.6s after interval 160` | `wait` | `PASS` | waited 0.6s after persistent write command |
| 13 | `interval` | `interval-write` | `PASS` |  |
| 14 | `dirty` | `interval-write` | `PASS` |  |
| 15 | `resync` | `interval-write` | `PASS` |  |
| 16 | `dirty` | `interval-write` | `PASS` |  |
| 17 | `interval 150` | `restore` | `PASS` |  |
| 18 | `wait 0.6s after interval 150` | `wait` | `PASS` | waited 0.6s after persistent write command |
| 19 | `interval` | `restore` | `PASS` |  |
| 20 | `dirty` | `restore` | `PASS` |  |
| 21 | `resync` | `restore` | `PASS` |  |
| 22 | `dirty` | `restore` | `PASS` |  |
| 23 | `interval` | `final` | `PASS` |  |
| 24 | `factor` | `final` | `PASS` |  |
| 25 | `mode` | `final` | `PASS` |  |
| 26 | `addr` | `final` | `PASS` |  |
| 27 | `partname` | `final` | `PASS` |  |
| 28 | `serial` | `final` | `PASS` |  |
| 29 | `offset` | `final` | `PASS` |  |
| 30 | `gain` | `final` | `PASS` |  |
| 31 | `drv` | `final` | `PASS` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
