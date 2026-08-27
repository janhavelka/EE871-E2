#!/usr/bin/env python3
"""Run a checkpointed, read-only EE871 serial soak and capture live evidence."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import ee871_hil_runner as hil


SCRIPT_VERSION = "1.3"
DEFAULT_DURATION_HOURS = 8.0
DEFAULT_SAMPLE_INTERVAL_SECONDS = 60.0
DEFAULT_STRESS_PERIOD_MINUTES = 30.0
DEFAULT_STRESS_COUNT = 500
DEFAULT_SCHEDULED_NACK_RETRY_MS = 1500
RESULT_SCHEDULED_CONTROL_NACK_RECOVERED = "SCHEDULED_CONTROL_NACK_RECOVERED"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def utc_text(value: datetime | None = None) -> str:
    current = value or utc_now()
    return current.replace(microsecond=0).isoformat().replace("+00:00", "Z")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a non-persistent EE871 soak with periodic raw measurements, "
            "status/health checks, and mixed-read stress blocks."
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=hil.DEFAULT_BAUD)
    parser.add_argument("--duration-hours", type=float, default=DEFAULT_DURATION_HOURS)
    parser.add_argument(
        "--sample-interval-seconds",
        type=float,
        default=DEFAULT_SAMPLE_INTERVAL_SECONDS,
    )
    parser.add_argument(
        "--stress-period-minutes",
        type=float,
        default=DEFAULT_STRESS_PERIOD_MINUTES,
    )
    parser.add_argument("--stress-count", type=int, default=DEFAULT_STRESS_COUNT)
    parser.add_argument(
        "--scheduled-nack-retry-ms",
        type=int,
        default=DEFAULT_SCHEDULED_NACK_RETRY_MS,
        help=(
            "One bounded application-level retry delay for a fully framed MV3/MV4 "
            "control-byte NACK; zero disables the retry."
        ),
    )
    parser.add_argument("--command-timeout", type=float, default=hil.DEFAULT_COMMAND_TIMEOUT_S)
    parser.add_argument("--idle", type=float, default=hil.DEFAULT_IDLE_S)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("hil_results") / "soak",
        help="Parent directory used when --run-dir is omitted.",
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="Exact artifact directory. It must not already contain soak evidence.",
    )
    parser.add_argument("--board", default="unknown")
    parser.add_argument("--target-name", default="unknown")
    parser.add_argument("--operator", default="unknown")
    args = parser.parse_args(argv)

    if args.duration_hours <= 0:
        parser.error("--duration-hours must be greater than zero")
    if args.sample_interval_seconds < 1:
        parser.error("--sample-interval-seconds must be at least 1")
    if args.stress_period_minutes <= 0:
        parser.error("--stress-period-minutes must be greater than zero")
    if args.stress_count <= 0:
        parser.error("--stress-count must be greater than zero")
    if args.scheduled_nack_retry_ms < 0:
        parser.error("--scheduled-nack-retry-ms must not be negative")
    if args.command_timeout <= 0:
        parser.error("--command-timeout must be greater than zero")
    if args.idle < 0:
        parser.error("--idle must not be negative")
    return args


def make_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        path = args.run_dir
        path.mkdir(parents=True, exist_ok=True)
        if any(path.iterdir()):
            raise RuntimeError(f"Run directory is not empty: {path}")
        return path

    stamp = utc_now().strftime("%Y%m%dT%H%M%SZ")
    candidate = args.output_dir / f"ee871_soak_{stamp}"
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = args.output_dir / f"ee871_soak_{stamp}_{suffix}"
    candidate.mkdir(parents=True)
    return candidate


def command_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        port=args.port,
        baud=args.baud,
        command_timeout=args.command_timeout,
        idle=args.idle,
    )


def preflight_specs(stress_count: int) -> list[hil.CommandSpec]:
    return [
        hil.CommandSpec(
            "version",
            "Capture exact firmware and library metadata.",
            group="soak-preflight",
            expected_any=("EE871 library version:",),
            validators=("version",),
        ),
        hil.CommandSpec(
            "id",
            "Verify EE871 group, subgroup, and CO2 availability.",
            group="soak-preflight",
            expected_any=("Group=0x0367, Subgroup=0x09, Available=0x08",),
            validators=("status_ok",),
        ),
        hil.CommandSpec(
            "interval",
            "Record measurement interval without writing it.",
            group="soak-preflight",
            expected_any=("Interval:",),
            validators=("status_ok", "interval_read"),
        ),
        hil.CommandSpec(
            "selftest",
            "Run the safe firmware self-test.",
            group="soak-preflight",
            expected_any=("Selftest result:",),
            validators=("selftest",),
            timeout_s=60.0,
        ),
        stress_spec(stress_count, "soak-preflight"),
        health_spec("soak-preflight"),
        dirty_spec("soak-preflight"),
    ]


def sample_specs() -> list[hil.CommandSpec]:
    return [
        hil.CommandSpec(
            "co2fast",
            "Read raw MV3 fast response.",
            group="soak-sample",
            expected_any=("CO2 fast:",),
            validators=("status_ok",),
        ),
        hil.CommandSpec(
            "co2avg",
            "Read raw MV4 averaged response.",
            group="soak-sample",
            expected_any=("CO2 avg:",),
            validators=("status_ok", "co2_avg"),
        ),
        hil.CommandSpec(
            "status",
            "Read sensor status after MV3/MV4.",
            group="soak-sample",
            expected_any=("hasCo2Error():",),
            validators=("status_ok",),
        ),
        health_spec("soak-sample"),
        dirty_spec("soak-sample"),
    ]


def final_specs(stress_count: int) -> list[hil.CommandSpec]:
    return [
        stress_spec(stress_count, "soak-final"),
        hil.CommandSpec(
            "selftest",
            "Run final safe firmware self-test.",
            group="soak-final",
            expected_any=("Selftest result:",),
            validators=("selftest",),
            timeout_s=60.0,
        ),
        health_spec("soak-final"),
        dirty_spec("soak-final"),
        hil.CommandSpec(
            "interval",
            "Verify the measurement interval is still readable and unchanged.",
            group="soak-final",
            expected_any=("Interval:",),
            validators=("status_ok", "interval_read", "interval_expected"),
        ),
    ]


def stress_spec(count: int, group: str) -> hil.CommandSpec:
    command = f"stress_mix {count}"
    return hil.CommandSpec(
        command,
        "Run bounded mixed safe reads.",
        group=group,
        expected_any=("=== stress_mix summary ===",),
        validators=("stress",),
        # Match the HIL runner's margin for the same command (420 s for 500 ops);
        # 0.05 s/op left under 2x headroom at the observed ~13-15 ops/s.
        timeout_s=max(120.0, count * 0.84),
    )


def health_spec(group: str) -> hil.CommandSpec:
    return hil.CommandSpec(
        "drv",
        "Require READY health with zero consecutive failures.",
        group=group,
        expected_any=("=== Driver Health ===",),
        validators=("health_ready",),
    )


def dirty_spec(group: str) -> hil.CommandSpec:
    return hil.CommandSpec(
        "dirty",
        "Require clean persistent configuration state.",
        group=group,
        expected_any=("=== Persistent Config Dirty State ===",),
        validators=("dirty_clean",),
    )


def compact_row(row: dict[str, Any]) -> dict[str, Any]:
    compact = {
        key: row[key]
        for key in (
            "command",
            "group",
            "result",
            "elapsed_s",
            "wait_reason",
            "attempt",
            "attempt_started_utc",
            "attempt_id",
            "retry_of",
        )
        if key in row
    }
    if row.get("planned_command") != row.get("command"):
        compact["planned_command"] = row.get("planned_command")
    for key in ("reason", "requires_opt_in", "notes"):
        if row.get(key):
            compact[key] = row[key]
    for key in ("destructive", "operator_required"):
        if row.get(key):
            compact[key] = True
    if row.get("result") != hil.RESULT_PASS:
        if row.get("parsed"):
            compact["parsed"] = row["parsed"]
        if row.get("clean_excerpt"):
            compact["failure_excerpt"] = row["clean_excerpt"]
    return compact


def exception_row(command: str, group: str, message: str) -> dict[str, Any]:
    spec = hil.CommandSpec(command, "Serial connection/command failure.", group=group)
    return hil.result_row(
        spec,
        command,
        hil.RESULT_FAIL,
        message,
        0.0,
        "",
        "exception",
        {},
    )


def skipped_row(spec: hil.CommandSpec, reason: str) -> dict[str, Any]:
    return hil.result_row(
        spec,
        spec.command,
        hil.RESULT_SKIP,
        reason,
        0.0,
        "",
        "not-sent",
        {},
    )


def is_scheduled_control_nack(spec: hil.CommandSpec, row: dict[str, Any]) -> bool:
    """Recognize the narrow, empirically observed scheduled-sample NACK."""
    if spec.group != "soak-sample" or spec.command not in ("co2fast", "co2avg"):
        return False
    status = (row.get("parsed") or {}).get("status") or {}
    clean = hil.strip_ansi(str(row.get("raw", "")))
    return (
        row.get("result") == hil.RESULT_FAIL
        and status.get("name") == "NACK"
        and status.get("code") == 8
        and "Control byte NACK" in clean
    )


def adjust_domain_result(row: dict[str, Any]) -> None:
    clean = hil.strip_ansi(str(row.get("raw", "")))
    if row.get("command") == "status" and "hasCo2Error(): YES" in clean:
        row["result"] = hil.RESULT_FAIL
        row["reason"] = "EE871 CO2 status error bit is set"


def update_observations(
    observations: dict[str, Any],
    row: dict[str, Any],
    previous_total_success: int | None,
) -> int | None:
    parsed = row.get("parsed") or {}
    for key in ("co2_fast_ppm", "co2_avg_ppm"):
        value = parsed.get(key)
        if not isinstance(value, int):
            continue
        current = observations.setdefault(
            key,
            {"minimum": value, "maximum": value, "first": value, "last": value},
        )
        current["minimum"] = min(current["minimum"], value)
        current["maximum"] = max(current["maximum"], value)
        current["last"] = value

    total_success = parsed.get("total_success")
    if isinstance(total_success, int):
        if previous_total_success is not None and total_success < previous_total_success:
            observations["transport_counter_regressions"] += 1
        return total_success
    return previous_total_success


def append_transcript(
    path: Path,
    row_number: int,
    cycle: int,
    row: dict[str, Any],
) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as handle:
        handle.write(
            f"=== {utc_text()} row={row_number} cycle={cycle} "
            f"command={row.get('command')} ===\n"
        )
        handle.write(
            f"result={row.get('result')} reason={row.get('reason')} "
            f"elapsed_s={row.get('elapsed_s')} wait={row.get('wait_reason')} "
            f"attempt={row.get('attempt', 1)} "
            f"attempt_started_utc={row.get('attempt_started_utc', 'unknown')} "
            f"retry_of={row.get('retry_of', 'none')}\n"
        )
        raw = str(row.get("raw", ""))
        handle.write(raw)
        if raw and not raw.endswith("\n"):
            handle.write("\n")
        handle.write("\n")
        handle.flush()


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def aggregate_counts(records: list[dict[str, Any]]) -> dict[str, int]:
    counts = {
        hil.RESULT_PASS: 0,
        hil.RESULT_FAIL: 0,
        hil.RESULT_SKIP: 0,
        hil.RESULT_OPERATOR: 0,
    }
    for row in records:
        result = str(row.get("result", ""))
        counts[result] = counts.get(result, 0) + 1
    return counts


def checkpoint_payload(
    metadata: dict[str, Any],
    records: list[dict[str, Any]],
    state: dict[str, Any],
    observations: dict[str, Any],
    cycle_count: int,
    reconnects: int,
    status: str,
    started_monotonic: float,
    final_verdict: str | None = None,
) -> dict[str, Any]:
    elapsed = max(0.0, time.monotonic() - started_monotonic)
    payload: dict[str, Any] = {
        "metadata": metadata,
        "status": status,
        "last_checkpoint_utc": utc_text(),
        "elapsed_seconds": round(elapsed, 3),
        "progress_percent": round(
            min(100.0, (elapsed / metadata["requested_duration_seconds"]) * 100.0),
            3,
        ),
        "cycle_count": cycle_count,
        "serial_reconnects": reconnects,
        "counts": aggregate_counts(records),
        "parsed_state": state,
        "observations": observations,
        "commands": records,
    }
    if final_verdict is not None:
        payload["final_verdict"] = final_verdict
    return payload


def write_summary_md(path: Path, payload: dict[str, Any]) -> None:
    counts = payload["counts"]
    observations = payload["observations"]
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# EE871-E2 Soak Summary\n\n")
        handle.write(f"- Final verdict: `{payload.get('final_verdict')}`\n")
        handle.write(f"- Status: `{payload.get('status')}`\n")
        handle.write(f"- Start UTC: `{payload['metadata']['start_utc']}`\n")
        handle.write(f"- End UTC: `{payload.get('last_checkpoint_utc')}`\n")
        handle.write(f"- Requested duration: `{payload['metadata']['duration_hours']} h`\n")
        handle.write(f"- Elapsed: `{payload['elapsed_seconds']} s`\n")
        handle.write(f"- Sample cycles: `{payload['cycle_count']}`\n")
        handle.write(f"- Serial reconnects: `{payload['serial_reconnects']}`\n")
        handle.write(
            f"- Commands: PASS `{counts.get(hil.RESULT_PASS, 0)}`, "
            f"SCHEDULED_CONTROL_NACK_RECOVERED "
            f"`{counts.get(RESULT_SCHEDULED_CONTROL_NACK_RECOVERED, 0)}`, "
            f"FAIL `{counts.get(hil.RESULT_FAIL, 0)}`, "
            f"REVIEW `{counts.get(hil.RESULT_OPERATOR, 0)}`, "
            f"SKIP `{counts.get(hil.RESULT_SKIP, 0)}`\n"
        )
        handle.write(
            f"- Transport counter regressions: "
            f"`{observations['transport_counter_regressions']}`\n"
        )
        handle.write(
            f"- Scheduled control-byte NACKs recovered by one bounded retry: "
            f"`{observations['scheduled_control_nack_recoveries']}`\n"
        )
        for key, label in (
            ("co2_fast_ppm", "Raw MV3"),
            ("co2_avg_ppm", "Raw MV4"),
        ):
            if key in observations:
                item = observations[key]
                handle.write(
                    f"- {label} observed range: `{item['minimum']}..{item['maximum']} ppm` "
                    f"(first `{item['first']}`, last `{item['last']}`)\n"
                )
        handle.write("\n")
        handle.write(
            "This soak validates bounded transport, status/health behavior, "
            "persistent-state cleanliness, and counter monotonicity for the "
            "recorded bench interval. It does not validate CO2 accuracy or calibration.\n"
        )


def open_serial(args: argparse.Namespace) -> object:
    serial_args = command_args(args)
    ser = hil.open_serial(serial_args)
    try:
        _, _, timed_out = hil.synchronize_cli(
            ser,
            args.command_timeout,
        )
        if timed_out:
            raise RuntimeError("serial CLI synchronization timed out")
        return ser
    except Exception:
        close_serial(ser)
        raise


def close_serial(ser: object | None) -> None:
    if ser is None:
        return
    close = getattr(ser, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    run_dir = make_run_dir(args)
    transcript_path = run_dir / "serial_transcript.txt"
    checkpoint_path = run_dir / "checkpoint.json"
    summary_json_path = run_dir / "summary.json"
    summary_md_path = run_dir / "summary.md"

    start_utc = utc_now()
    requested_seconds = args.duration_hours * 3600.0
    target_end_utc = start_utc + timedelta(seconds=requested_seconds)
    started_monotonic = time.monotonic()
    deadline = started_monotonic + requested_seconds
    next_sample = started_monotonic
    stress_period_seconds = args.stress_period_minutes * 60.0
    next_stress = started_monotonic + stress_period_seconds

    metadata = {
        "tool": "ee871_soak_runner.py",
        "tool_version": SCRIPT_VERSION,
        "start_utc": utc_text(start_utc),
        "target_end_utc": utc_text(target_end_utc),
        "duration_hours": args.duration_hours,
        "requested_duration_seconds": requested_seconds,
        "sample_interval_seconds": args.sample_interval_seconds,
        "stress_period_minutes": args.stress_period_minutes,
        "stress_count": args.stress_count,
        "scheduled_nack_retry_ms": args.scheduled_nack_retry_ms,
        "port": args.port,
        "baud": args.baud,
        "board": args.board,
        "target_name": args.target_name,
        "operator": args.operator,
        "git_branch": hil.git_value("branch", "--show-current"),
        "git_commit": hil.git_value("rev-parse", "--short=12", "HEAD"),
        "git_worktree": hil.worktree_state(),
        "claim_boundary": (
            "Transport/health/persistent-state soak only; no CO2 accuracy or "
            "calibration claim. A fully framed control-byte NACK on a scheduled "
            "MV3/MV4 sample is retried once by this application-level harness "
            "only; the NACK's sensor-internal cause is not inferred."
        ),
    }

    transcript_path.write_text(
        "EE871-E2 soak transcript\n"
        + "\n".join(f"{key}={value}" for key, value in metadata.items())
        + "\n\n",
        encoding="utf-8",
    )

    records: list[dict[str, Any]] = []
    state: dict[str, Any] = {}
    observations: dict[str, Any] = {
        "transport_counter_regressions": 0,
        "scheduled_control_nack_recoveries": 0,
    }
    cycle_count = 0
    reconnects = 0
    connected_once = False
    previous_total_success: int | None = None
    baseline_interval: int | None = None
    completed_duration = False
    status = "RUNNING"
    ser: object | None = None

    def persist_checkpoint() -> None:
        payload = checkpoint_payload(
            metadata,
            records,
            state,
            observations,
            cycle_count,
            reconnects,
            status,
            started_monotonic,
        )
        write_json_atomic(checkpoint_path, payload)

    def record(row: dict[str, Any], cycle: int) -> None:
        nonlocal previous_total_success, baseline_interval
        adjust_domain_result(row)
        hil.update_state(state, row)
        parsed = row.get("parsed") or {}
        interval = parsed.get("measurement_interval_ds")
        if isinstance(interval, int) and baseline_interval is None:
            baseline_interval = interval
            state["expected_measurement_interval_ds"] = interval
        previous_total_success = update_observations(
            observations,
            row,
            previous_total_success,
        )
        append_transcript(transcript_path, len(records) + 1, cycle, row)
        records.append(compact_row(row))
        persist_checkpoint()

    def ensure_connection(group: str, cycle: int) -> bool:
        nonlocal ser, reconnects, connected_once
        if ser is not None:
            return True
        try:
            ser = open_serial(args)
            if connected_once:
                reconnects += 1
            connected_once = True
            return True
        except Exception as exc:
            record(
                exception_row(
                    "serial-connect",
                    group,
                    f"{type(exc).__name__}: {exc}",
                ),
                cycle,
            )
            ser = None
            return False

    def run_specs(specs: list[hil.CommandSpec], cycle: int) -> None:
        nonlocal ser
        group = specs[0].group if specs else "soak"
        if not ensure_connection(group, cycle):
            for unsent in specs:
                record(
                    skipped_row(unsent, "not sent because serial connection failed"),
                    cycle,
                )
            return
        for index, spec in enumerate(specs):
            attempt_id = f"cycle-{cycle}-spec-{index + 1}-{spec.command}"
            attempt_started_utc = utc_text()
            try:
                row = hil.run_serial_command(
                    ser,
                    spec,
                    spec.command,
                    command_args(args),
                    state,
                )
                row["attempt"] = 1
                row["attempt_started_utc"] = attempt_started_utc
                row["attempt_id"] = attempt_id
            except Exception as exc:
                row = exception_row(
                    spec.command,
                    spec.group,
                    f"{type(exc).__name__}: {exc}",
                )
                row["attempt"] = 1
                row["attempt_started_utc"] = attempt_started_utc
                row["attempt_id"] = attempt_id
                record(row, cycle)
                close_serial(ser)
                ser = None
                for unsent in specs[index + 1 :]:
                    record(
                        skipped_row(unsent, "not sent after serial exception"),
                        cycle,
                    )
                return

            if (
                args.scheduled_nack_retry_ms > 0
                and is_scheduled_control_nack(spec, row)
            ):
                time.sleep(args.scheduled_nack_retry_ms / 1000.0)
                retry_started_utc = utc_text()
                retry_exception = False
                try:
                    retry_row = hil.run_serial_command(
                        ser,
                        spec,
                        spec.command,
                        command_args(args),
                        state,
                    )
                except Exception as exc:
                    retry_exception = True
                    retry_row = exception_row(
                        spec.command,
                        spec.group,
                        f"scheduled-NACK retry {type(exc).__name__}: {exc}",
                    )
                retry_row["attempt"] = 2
                retry_row["attempt_started_utc"] = retry_started_utc
                retry_row["attempt_id"] = f"{attempt_id}-retry"
                retry_row["retry_of"] = attempt_id

                if retry_row.get("result") == hil.RESULT_PASS:
                    row["result"] = RESULT_SCHEDULED_CONTROL_NACK_RECOVERED
                    row["reason"] = (
                        "fully framed scheduled MV3/MV4 control-byte NACK; "
                        f"retry after {args.scheduled_nack_retry_ms} ms passed"
                    )
                    observations["scheduled_control_nack_recoveries"] += 1

                record(row, cycle)
                record(retry_row, cycle)
                if retry_exception or retry_row.get("wait_reason") == "timeout":
                    close_serial(ser)
                    ser = None
                    for unsent in specs[index + 1 :]:
                        record(
                            skipped_row(
                                unsent,
                                (
                                    "not sent after scheduled-NACK retry "
                                    + (
                                        "serial exception"
                                        if retry_exception
                                        else "framing timeout"
                                    )
                                ),
                            ),
                            cycle,
                        )
                    return
                continue

            record(row, cycle)
            if row.get("wait_reason") == "timeout":
                close_serial(ser)
                ser = None
                for unsent in specs[index + 1 :]:
                    record(
                        skipped_row(unsent, "not sent after framing timeout"),
                        cycle,
                    )
                return

    print(f"Run directory: {run_dir}", flush=True)
    print(f"Start UTC: {metadata['start_utc']}", flush=True)
    print(f"Target end UTC: {metadata['target_end_utc']}", flush=True)

    try:
        run_specs(preflight_specs(min(args.stress_count, 100)), 0)
        if any(row.get("result") != hil.RESULT_PASS for row in records):
            status = "PREFLIGHT_FAILED"
        else:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_sample:
                    cycle_count += 1
                    run_specs(sample_specs(), cycle_count)
                    next_sample += args.sample_interval_seconds
                    if next_sample <= now:
                        next_sample = now + args.sample_interval_seconds

                now = time.monotonic()
                if now >= next_stress:
                    run_specs(
                        [
                            stress_spec(args.stress_count, "soak-periodic-stress"),
                            health_spec("soak-periodic-stress"),
                            dirty_spec("soak-periodic-stress"),
                        ],
                        cycle_count,
                    )
                    next_stress += stress_period_seconds
                    if next_stress <= now:
                        next_stress = now + stress_period_seconds

                remaining = min(next_sample, next_stress, deadline) - time.monotonic()
                if remaining > 0:
                    time.sleep(min(1.0, remaining))

            completed_duration = time.monotonic() >= deadline
            run_specs(final_specs(args.stress_count), cycle_count + 1)
            status = "COMPLETE" if completed_duration else "INCOMPLETE"
    except KeyboardInterrupt:
        status = "INTERRUPTED"
    except Exception as exc:
        status = "RUNNER_EXCEPTION"
        record(
            exception_row(
                "runner",
                "soak-runner",
                f"{type(exc).__name__}: {exc}",
            ),
            cycle_count,
        )
    finally:
        close_serial(ser)

    counts = aggregate_counts(records)
    hard_failure = (
        counts.get(hil.RESULT_FAIL, 0) > 0
        or counts.get(hil.RESULT_OPERATOR, 0) > 0
        or counts.get(hil.RESULT_SKIP, 0) > 0
        or reconnects > 0
        or observations["transport_counter_regressions"] > 0
        or state.get("driver_state") != "READY"
        or state.get("consecutive_failures") != 0
        or state.get("persistent_config_dirty") is not False
    )
    if hard_failure:
        final_verdict = hil.VERDICT_FAIL
    elif not completed_duration or status != "COMPLETE":
        final_verdict = hil.VERDICT_INCOMPLETE
    else:
        final_verdict = hil.VERDICT_PASS

    final_payload = checkpoint_payload(
        metadata,
        records,
        state,
        observations,
        cycle_count,
        reconnects,
        status,
        started_monotonic,
        final_verdict,
    )
    write_json_atomic(checkpoint_path, final_payload)
    write_json_atomic(summary_json_path, final_payload)
    write_summary_md(summary_md_path, final_payload)
    try:
        checkpoint_path.unlink()
    except OSError:
        # The final summary is authoritative; a locked duplicate is harmless.
        pass

    print(f"Status: {status}", flush=True)
    print(f"Final verdict: {final_verdict}", flush=True)
    print(f"Cycles: {cycle_count}", flush=True)
    print(f"Counts: {final_payload['counts']}", flush=True)
    return hil.exit_code_for_verdict(final_verdict)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
