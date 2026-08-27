#!/usr/bin/env python3
"""Exercise serial framing without touching the EE871 E2 transport."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import ee871_hil_runner as hil


SCRIPT_VERSION = "1.2"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=hil.DEFAULT_BAUD)
    parser.add_argument("--count", type=int, default=10_000)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--board", default="unknown")
    parser.add_argument("--target-name", default="unknown")
    parser.add_argument("--operator", default="unknown")
    parser.add_argument("--expected-arduino-version")
    parser.add_argument("--expected-idf-version")
    parser.add_argument("--expected-library-version")
    args = parser.parse_args(argv)
    if args.count <= 0:
        parser.error("--count must be greater than zero")
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    return args


def make_run_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    if any(path.iterdir()):
        raise RuntimeError(f"Run directory is not empty: {path}")
    return path


def expected_version_failures(
    state: dict[str, Any],
    args: argparse.Namespace,
) -> list[str]:
    checks = (
        ("arduino_esp32_version", args.expected_arduino_version, "Arduino-ESP32"),
        ("esp_idf_version", args.expected_idf_version, "ESP-IDF"),
        ("library_version", args.expected_library_version, "EE871"),
    )
    failures: list[str] = []
    for key, expected, label in checks:
        if expected is not None and state.get(key) != expected:
            failures.append(f"{label} {state.get(key)!r} != expected {expected!r}")
    return failures


def write_reports(run_dir: Path, payload: dict[str, Any]) -> None:
    (run_dir / "summary.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    counts = payload["counts"]
    with (run_dir / "summary.md").open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# EE871-E2 Serial-Only Framing Discriminator\n\n")
        handle.write(f"- Final verdict: `{payload['final_verdict']}`\n")
        handle.write(f"- Start UTC: `{payload['metadata']['start_utc']}`\n")
        handle.write(f"- End UTC: `{payload['metadata']['end_utc']}`\n")
        handle.write(f"- Port: `{payload['metadata']['port']}` at `{payload['metadata']['baud']}` baud\n")
        handle.write(f"- Requested round trips: `{payload['metadata']['requested_count']}`\n")
        handle.write(f"- Complete PASS replies: `{counts['pass']}`\n")
        handle.write(f"- Failures: `{counts['fail']}`\n")
        handle.write(f"- Elapsed: `{payload['elapsed_seconds']} s`\n")
        handle.write(f"- Reply-length histogram: `{payload['reply_length_histogram']}`\n")
        handle.write(f"- Distinct complete reply hashes: `{payload['distinct_reply_hashes']}`\n")
        handle.write(
            f"- Runtime stack: Arduino-ESP32 "
            f"`{payload['runtime_stack'].get('arduino_esp32_version')}`, ESP-IDF "
            f"`{payload['runtime_stack'].get('esp_idf_version')}`, EE871 "
            f"`{payload['runtime_stack'].get('library_version')}`\n"
        )
        if payload["failure"]:
            handle.write(f"- First failure: `{payload['failure']}`\n")
        handle.write("\n")
        handle.write(
            "`dirty` reads only driver-maintained state and performs no E2 bus "
            "operation. This result isolates CLI/native-USB response framing; "
            "it does not validate sensor transport, CO2 values, or long-term "
            "stability.\n"
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    run_dir = make_run_dir(args.run_dir)
    started_utc = hil.iso_timestamp()
    started = time.monotonic()
    pass_count = 0
    failure = ""
    lengths: Counter[int] = Counter()
    hashes: Counter[str] = Counter()
    state: dict[str, Any] = {}
    ser: object | None = None

    command_args = SimpleNamespace(
        command_timeout=args.timeout,
        idle=0.0,
    )
    serial_args = SimpleNamespace(port=args.port, baud=args.baud)
    try:
        ser = hil.open_serial(serial_args)
        _, _, handshake_timeout = hil.synchronize_cli(
            ser,
            args.timeout,
            0.0,
        )
        if handshake_timeout:
            raise RuntimeError("serial CLI synchronization timed out")

        version_spec = hil.CommandSpec(
            "version",
            "Capture exact firmware stack.",
            expected_any=("EE871 library version:",),
            validators=("version",),
        )
        version_row = hil.run_serial_command(
            ser,
            version_spec,
            "version",
            command_args,
            state,
        )
        hil.update_state(state, version_row)
        if version_row["result"] != hil.RESULT_PASS:
            raise RuntimeError(
                f"version command {version_row['result']}: {version_row['reason']}"
            )
        version_failures = expected_version_failures(state, args)
        if version_failures:
            raise RuntimeError("; ".join(version_failures))

        dirty_spec = hil.CommandSpec(
            "dirty",
            "Read state only; no E2 bus operation.",
            expected_any=("Persistent Config Dirty State",),
            validators=("dirty_clean",),
        )
        for index in range(args.count):
            row = hil.run_serial_command(
                ser,
                dirty_spec,
                "dirty",
                command_args,
                state,
            )
            raw = str(row.get("raw", ""))
            if row["result"] != hil.RESULT_PASS:
                failure = (
                    f"round trip {index + 1}: {row['result']} "
                    f"{row.get('reason', '')}".strip()
                )
                break
            encoded = raw.encode("utf-8")
            lengths[len(encoded)] += 1
            hashes[hashlib.sha256(encoded).hexdigest()] += 1
            pass_count += 1
            if pass_count % 1000 == 0:
                print(f"{pass_count}/{args.count}", flush=True)
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        if ser is not None:
            close = getattr(ser, "close", None)
            if callable(close):
                close()

    complete = pass_count == args.count and not failure
    payload = {
        "metadata": {
            "tool": "ee871_serial_discriminator.py",
            "tool_version": SCRIPT_VERSION,
            "start_utc": started_utc,
            "end_utc": hil.iso_timestamp(),
            "port": args.port,
            "baud": args.baud,
            "requested_count": args.count,
            "board": args.board,
            "target_name": args.target_name,
            "operator": args.operator,
            "git_branch": hil.git_value("branch", "--show-current"),
            "git_commit": hil.git_value("rev-parse", "--short=12", "HEAD"),
            "git_worktree": hil.worktree_state(),
        },
        "final_verdict": hil.VERDICT_PASS if complete else hil.VERDICT_FAIL,
        "counts": {"pass": pass_count, "fail": 0 if complete else 1},
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "reply_length_histogram": {
            str(length): count for length, count in sorted(lengths.items())
        },
        "distinct_reply_hashes": len(hashes),
        "reply_sha256_counts": dict(hashes),
        "runtime_stack": {
            key: state.get(key)
            for key in (
                "arduino_esp32_version",
                "esp_idf_version",
                "library_version",
                "library_full",
            )
        },
        "failure": failure,
        "claim_boundary": (
            "The dirty command performs no E2 bus operation. This is serial "
            "CLI/native-USB framing evidence only."
        ),
    }
    write_reports(run_dir, payload)
    print(f"Final verdict: {payload['final_verdict']}", flush=True)
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
