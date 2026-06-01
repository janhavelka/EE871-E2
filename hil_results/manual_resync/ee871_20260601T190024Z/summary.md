# EE871-E2 HIL Summary

Final verdict: `PASS`

PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.

## Run Metadata

- timestamp_utc: `2026-06-01T19:00:24Z`
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

- PASS: `3`
- FAIL: `0`
- SKIP: `0`
- OPERATOR_REVIEW_REQUIRED: `0`

## Parsed State

```json
{
  "persistent_config_dirty": false,
  "resync_needed": false
}
```

## Commands

| # | Command | Group | Result | Elapsed s | Reason |
| --- | --- | --- | --- | --- | --- |
| 1 | `dirty` | `manual-safe` | `PASS` | `0.001` |  |
| 2 | `resync` | `manual-safe` | `PASS` | `0.2` |  |
| 3 | `dirty` | `manual-safe` | `PASS` | `0.001` |  |

## Artifacts

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
