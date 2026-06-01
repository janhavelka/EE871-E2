#!/usr/bin/env python3
"""Run EE871-E2 serial HIL tests and capture repeatable evidence.

The runner drives the repository diagnostic CLI over a serial port. It does
not flash firmware and it does not claim hardware success from dry-runs.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_VERSION = "1.0"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 8.0
DEFAULT_COMMAND_TIMEOUT_S = 20.0
DEFAULT_IDLE_S = 0.35
DEFAULT_OUTPUT_DIR = Path("hil_logs")
PERSISTENT_CONFIRM_TEXT = "I UNDERSTAND EE871 PERSISTENT WRITES"
PERSISTENT_RUNTIME_CONFIRM_TEXT = "RUN EE871 PERSISTENT WRITES"

RESULT_PASS = "PASS"
RESULT_FAIL = "FAIL"
RESULT_SKIP = "SKIP"
RESULT_OPERATOR = "OPERATOR_REVIEW_REQUIRED"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"
VERDICT_OPERATOR = "OPERATOR_REVIEW_REQUIRED"
VERDICT_INCOMPLETE = "INCOMPLETE"

ANSI_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
PROMPT_RE = re.compile(r"(^|\r?\n)>\s*$")
BOOL_TRUE = {"yes", "true", "1", "on"}
BOOL_FALSE = {"no", "false", "0", "off"}

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class CommandSpec:
    command: str
    description: str
    group: str = "safe"
    expected_any: tuple[str, ...] = ()
    validators: tuple[str, ...] = ()
    timeout_s: float | None = None
    send: bool = True
    operator_required: bool = False
    destructive: bool = False
    requires_opt_in: str | None = None
    notes: str = ""
    dynamic: str | None = None


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def decode(data: bytes) -> str:
    return data.decode("utf-8", errors="replace")


def parse_boolish(value: str) -> bool | None:
    lowered = value.strip().lower()
    if lowered in BOOL_TRUE:
        return True
    if lowered in BOOL_FALSE:
        return False
    return None


def last_match(pattern: str, text: str, flags: int = 0) -> re.Match[str] | None:
    matches = list(re.finditer(pattern, text, flags))
    return matches[-1] if matches else None


def header_sections(text: str, header: str) -> list[tuple[int, str]]:
    sections: list[tuple[int, str]] = []
    for match in re.finditer(re.escape(header), text, re.IGNORECASE):
        tail = text[match.end():]
        next_header = re.search(r"\r?\n=== [^\r\n]+ ===", tail)
        end = match.end() + next_header.start() if next_header else len(text)
        sections.append((match.start(), text[match.start():end]))
    return sections


def last_header_section(text: str, header: str) -> str:
    sections = header_sections(text, header)
    return sections[-1][1] if sections else text


def git_value(*args: str, empty_value: str = "unknown") -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return result.stdout.strip() or empty_value


def parse_status(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    statuses: list[dict[str, Any]] = []
    for match in re.finditer(
        r"\bStatus:\s*([A-Z_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)",
        clean,
        re.IGNORECASE,
    ):
        statuses.append(
            {
                "name": match.group(1).upper(),
                "code": int(match.group(2)),
                "detail": int(match.group(3)),
            }
        )
    if not statuses:
        return {}
    return {"statuses": statuses, "status": statuses[-1]}


def parse_version(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    patterns = {
        "firmware_build": r"Example firmware build:\s*([^\r\n]+)",
        "library_version": r"EE871 library version:\s*([^\r\n]+)",
        "library_full": r"EE871 library full:\s*([^\r\n]+)",
        "library_build": r"EE871 library build:\s*([^\r\n]+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, clean, re.IGNORECASE)
        if match:
            parsed[key] = match.group(1).strip()
    match = re.search(r"EE871 library commit:\s*([0-9a-fA-F]+|unknown)(?:\s*\(([^)]+)\))?", clean)
    if match:
        parsed["library_commit"] = match.group(1)
        if match.group(2):
            parsed["library_git_status"] = match.group(2)
    return parsed


def parse_selftest(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    match = last_match(
        r"Selftest result:\s*pass=(\d+)\s+fail=(\d+)\s+skip=(\d+)",
        clean,
        re.IGNORECASE,
    )
    if not match:
        return {}
    return {
        "selftest": {
            "pass": int(match.group(1)),
            "fail": int(match.group(2)),
            "skip": int(match.group(3)),
        }
    }


def parse_stress(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    candidates: list[tuple[int, str, dict[str, int | str]]] = []
    for pos, section in header_sections(clean, "=== Stress Summary ==="):
        total = last_match(r"\bTotal:\s*(\d+)", section)
        success = last_match(r"\bSuccess:\s*(\d+)", section)
        errors = last_match(r"\bErrors:\s*(\d+)", section)
        if total and success and errors:
            candidates.append((pos, section, {
                "kind": "stress",
                "total": int(total.group(1)),
                "success": int(success.group(1)),
                "errors": int(errors.group(1)),
            }))
    for pos, section in header_sections(clean, "=== stress_mix summary ==="):
        total = last_match(r"\bTotal:\s*ok=(\d+)\s+fail=(\d+)", section, re.IGNORECASE)
        if total:
            ok = int(total.group(1))
            fail = int(total.group(2))
            candidates.append((pos, section, {
                "kind": "stress_mix",
                "total": ok + fail,
                "success": ok,
                "errors": fail,
            }))
    if candidates:
        _, section, stress = max(candidates, key=lambda item: item[0])
        parsed["stress"] = stress
        match = last_match(r"Health delta:\s*success\s*\+(\d+),\s*failures\s*\+(\d+)", section, re.IGNORECASE)
        if match:
            parsed["health_delta_success"] = int(match.group(1))
            parsed["health_delta_failures"] = int(match.group(2))
    return parsed


def parse_health(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    section = last_header_section(clean, "=== Driver Health ===")
    parsed: dict[str, Any] = {}
    match = last_match(r"\bState:\s*(UNINIT|READY|DEGRADED|OFFLINE)", section, re.IGNORECASE)
    if match:
        parsed["driver_state"] = match.group(1).upper()
    match = last_match(r"\bOnline:\s*(yes|no|true|false|0|1)", section, re.IGNORECASE)
    if match:
        parsed["online"] = parse_boolish(match.group(1))
    match = last_match(r"\bConsecutive failures:\s*(\d+)", section, re.IGNORECASE)
    if match:
        parsed["consecutive_failures"] = int(match.group(1))
    match = last_match(r"\bTotal success:\s*(\d+)", section, re.IGNORECASE)
    if match:
        parsed["total_success"] = int(match.group(1))
    match = last_match(r"\bTotal failures:\s*(\d+)", section, re.IGNORECASE)
    if match:
        parsed["total_failures"] = int(match.group(1))
    match = last_match(r"\bSuccess rate:\s*([0-9]+(?:\.[0-9]+)?)%", section, re.IGNORECASE)
    if match:
        parsed["success_rate_pct"] = float(match.group(1))
    return parsed


def parse_dirty(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    matches = list(re.finditer(r"\bpersistentConfigDirty:\s*(yes|no|true|false|0|1)", clean, re.IGNORECASE))
    if matches:
        parsed["persistent_config_dirty"] = parse_boolish(matches[-1].group(1))
    matches = list(re.finditer(
        r"\bpersistentConfigDirtyError:\s*([A-Z_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["persistent_config_dirty_error"] = {
            "name": match.group(1).upper(),
            "code": int(match.group(2)),
            "detail": int(match.group(3)),
        }
    matches = list(re.finditer(r"\bpersistentConfigDirtyError message:\s*([^\r\n]+)", clean, re.IGNORECASE))
    if matches:
        parsed["persistent_config_dirty_error_message"] = matches[-1].group(1).strip()
    matches = list(re.finditer(r"\bresyncNeeded:\s*(yes|no|true|false|0|1)", clean, re.IGNORECASE))
    if matches:
        parsed["resync_needed"] = parse_boolish(matches[-1].group(1))
    return parsed


def parse_measurements(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    match = last_match(r"\bCO2 avg:\s*(\d+)\s*ppm", clean, re.IGNORECASE)
    if match:
        parsed["co2_avg_ppm"] = int(match.group(1))
    match = last_match(r"\bCO2 fast:\s*(\d+)\s*ppm", clean, re.IGNORECASE)
    if match:
        parsed["co2_fast_ppm"] = int(match.group(1))
    match = last_match(r"\bInterval:\s*(\d+)\s*deciseconds", clean, re.IGNORECASE)
    if match:
        parsed["measurement_interval_ds"] = int(match.group(1))
    match = last_match(r"\bCO2 offset:\s*(-?\d+)\s*ppm", clean, re.IGNORECASE)
    if match:
        parsed["co2_offset_ppm"] = int(match.group(1))
    match = last_match(r"\bCO2 gain:\s*(\d+)", clean, re.IGNORECASE)
    if match:
        parsed["co2_gain"] = int(match.group(1))
    match = last_match(r"\bBus address:\s*(\d+)", clean, re.IGNORECASE)
    if match:
        parsed["device_address"] = int(match.group(1))
    return parsed


def parse_response(command: str, text: str) -> dict[str, Any]:
    parsed: dict[str, Any] = {}
    parsed.update(parse_status(text))
    parsed.update(parse_version(text))
    parsed.update(parse_selftest(text))
    parsed.update(parse_stress(text))
    parsed.update(parse_health(text))
    parsed.update(parse_dirty(text))
    parsed.update(parse_measurements(text))
    parsed["command"] = command
    return parsed


def command_count(command: str) -> int | None:
    match = re.search(r"\b(?:stress|stress_mix)\s+(\d+)\b", command)
    return int(match.group(1)) if match else None


def expected_token_present(clean: str, spec: CommandSpec) -> bool:
    if not spec.expected_any:
        return True
    lowered = clean.lower()
    return any(token.lower() in lowered for token in spec.expected_any)


def validate_parsed(
    spec: CommandSpec,
    parsed: dict[str, Any],
    state: dict[str, Any] | None = None,
) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    reviews: list[str] = []
    context = state or {}

    for validator in spec.validators:
        if validator == "version":
            if not parsed.get("library_version"):
                reviews.append("library version not parsed")
        elif validator == "status_ok":
            status = parsed.get("status")
            if not status:
                reviews.append("status line not parsed")
            elif status.get("name") != "OK" or status.get("code") != 0:
                failures.append(f"status is {status.get('name')}")
        elif validator == "co2_avg":
            if "co2_avg_ppm" not in parsed:
                reviews.append("CO2 averaged value not parsed")
        elif validator == "selftest":
            data = parsed.get("selftest")
            if not isinstance(data, dict):
                reviews.append("selftest result line not parsed")
            else:
                if data.get("fail", 0) > 0:
                    failures.append(f"selftest fail={data.get('fail')}")
                if data.get("pass", 0) <= 0:
                    reviews.append("selftest pass count is zero or missing")
        elif validator == "stress":
            data = parsed.get("stress")
            expected = command_count(spec.command)
            if not isinstance(data, dict):
                reviews.append("stress summary not parsed")
            else:
                if data.get("errors", 0) > 0:
                    failures.append(f"stress errors={data.get('errors')}")
                if expected is not None and data.get("total") != expected:
                    failures.append(f"stress total {data.get('total')} != requested {expected}")
                if expected is not None and data.get("success") != expected:
                    failures.append(f"stress success {data.get('success')} != requested {expected}")
                if parsed.get("health_delta_failures", 0) > 0:
                    failures.append(f"stress health failures +{parsed.get('health_delta_failures')}")
        elif validator == "health_ready":
            driver_state = parsed.get("driver_state")
            if not driver_state:
                reviews.append("driver state not parsed")
            elif driver_state != "READY":
                failures.append(f"driver state is {driver_state}")
            if "online" not in parsed:
                reviews.append("online flag not parsed")
            elif parsed.get("online") is not True:
                failures.append("driver is not online")
            if "consecutive_failures" not in parsed:
                reviews.append("consecutive failures not parsed")
            elif parsed.get("consecutive_failures") != 0:
                failures.append(f"consecutive failures is {parsed.get('consecutive_failures')}")
        elif validator == "dirty_clean":
            if "persistent_config_dirty" not in parsed:
                reviews.append("persistent dirty flag not parsed")
            elif parsed.get("persistent_config_dirty") is not False:
                failures.append("persistent config is dirty")
            if "resync_needed" not in parsed:
                reviews.append("resync-needed flag not parsed")
            elif parsed.get("resync_needed") is not False:
                failures.append("resync is needed")
        elif validator == "dirty_state":
            if "persistent_config_dirty" not in parsed:
                reviews.append("persistent dirty flag not parsed")
            if "persistent_config_dirty_error" not in parsed:
                reviews.append("persistent dirty error not parsed")
        elif validator == "interval_read":
            if "measurement_interval_ds" not in parsed:
                reviews.append("measurement interval not parsed")
        elif validator == "offset_read":
            if "co2_offset_ppm" not in parsed:
                reviews.append("CO2 offset not parsed")
        elif validator == "gain_read":
            if "co2_gain" not in parsed:
                reviews.append("CO2 gain not parsed")
        elif validator == "interval_expected":
            expected = context.get("expected_measurement_interval_ds")
            actual = parsed.get("measurement_interval_ds")
            if not isinstance(expected, int):
                reviews.append("expected measurement interval not recorded")
            elif actual != expected:
                failures.append(f"measurement interval readback {actual} != expected {expected}")
        elif validator == "offset_expected":
            expected = context.get("expected_co2_offset_ppm")
            actual = parsed.get("co2_offset_ppm")
            if not isinstance(expected, int):
                reviews.append("expected CO2 offset not recorded")
            elif actual != expected:
                failures.append(f"CO2 offset readback {actual} != expected {expected}")
        elif validator == "gain_expected":
            expected = context.get("expected_co2_gain")
            actual = parsed.get("co2_gain")
            if not isinstance(expected, int):
                reviews.append("expected CO2 gain not recorded")
            elif actual != expected:
                failures.append(f"CO2 gain readback {actual} != expected {expected}")
        elif validator == "health_faulted":
            driver_state = parsed.get("driver_state")
            consecutive = parsed.get("consecutive_failures")
            online = parsed.get("online")
            if not driver_state:
                reviews.append("driver state not parsed")
            if not isinstance(consecutive, int):
                reviews.append("consecutive failures not parsed")
            if "online" not in parsed:
                reviews.append("online flag not parsed")
            if driver_state == "READY" and consecutive == 0 and online is True:
                failures.append("driver health did not reflect induced fault")
        elif validator == "expected_failure":
            status = parsed.get("status")
            if not status:
                reviews.append("bounded failure status not parsed")
            elif status.get("name") == "OK" and status.get("code") == 0:
                failures.append("command reported OK during operator fault step")
        else:
            reviews.append(f"unknown validator {validator}")

    return failures, reviews


def classify_response(
    spec: CommandSpec,
    text: str,
    timed_out: bool,
    parsed: dict[str, Any],
    state: dict[str, Any] | None = None,
) -> tuple[str, str]:
    if spec.operator_required:
        return RESULT_OPERATOR, "operator evidence required"
    if timed_out:
        return RESULT_FAIL, "timeout"

    clean = strip_ansi(text)
    if not clean.strip():
        return RESULT_OPERATOR, "no serial response captured"

    if not expected_token_present(clean, spec):
        return RESULT_OPERATOR, "expected output token missing"

    failures, reviews = validate_parsed(spec, parsed, state)
    if failures:
        return RESULT_FAIL, "; ".join(failures)
    if reviews:
        return RESULT_OPERATOR, "; ".join(reviews)
    return RESULT_PASS, ""


def response_has_completion(command: str, text: str) -> bool:
    clean = strip_ansi(text)
    if not clean.strip():
        return False
    if command.startswith("stress_mix"):
        return "=== stress_mix summary ===" in clean and re.search(r"\bTotal:\s*ok=\d+\s+fail=\d+", clean) is not None
    if command.startswith("stress"):
        return "=== Stress Summary ===" in clean and re.search(r"\bErrors:\s*\d+", clean) is not None
    if command == "selftest":
        return re.search(r"Selftest result:\s*pass=\d+\s+fail=\d+\s+skip=\d+", clean) is not None
    if command == "drv":
        return "=== Driver Health ===" in clean and "persistentConfigDirty" in clean
    if command == "dirty":
        return "persistentConfigDirty" in clean and "resyncNeeded" in clean
    if command == "resync":
        return "=== Persistent Config Resync ===" in clean and "After:" in clean and "resyncNeeded" in clean
    if command == "version":
        return "EE871 library version:" in clean
    if command == "help" or command == "?":
        return "EE871-E2 CLI Help" in clean and "selftest" in clean
    if command in {"read", "probe", "recover", "interval", "offset", "gain", "addr"}:
        return "Status:" in clean
    return "Status:" in clean or PROMPT_RE.search(clean) is not None


def read_until_ready(ser: object, timeout_s: float, idle_s: float, command: str | None = None) -> tuple[str, str, bool]:
    deadline = time.monotonic() + timeout_s
    last_data_at = time.monotonic()
    data_seen = False
    completion_seen = command is None
    chunks: list[str] = []

    while time.monotonic() < deadline:
        pending = int(getattr(ser, "in_waiting", 0) or 0)
        data = ser.read(pending or 1)
        if data:
            chunks.append(decode(data))
            data_seen = True
            last_data_at = time.monotonic()
            joined = "".join(chunks)
            if command is not None and response_has_completion(command, joined):
                completion_seen = True
            if PROMPT_RE.search(strip_ansi(joined)):
                return joined, "prompt", False
            continue
        if data_seen and completion_seen and (time.monotonic() - last_data_at) >= idle_s:
            return "".join(chunks), "completion-idle" if command else "serial-idle", False
        if data_seen and command is None and (time.monotonic() - last_data_at) >= idle_s:
            return "".join(chunks), "serial-idle", False

    return "".join(chunks), "timeout", True


def safe_specs() -> list[CommandSpec]:
    return [
        CommandSpec("version", "Print firmware and library version.", expected_any=("EE871 library version:",), validators=("version",)),
        CommandSpec("help", "Capture CLI command surface.", expected_any=("EE871-E2 CLI Help",)),
        CommandSpec("probe", "Probe device without health side effects.", expected_any=("Status:",), validators=("status_ok",)),
        CommandSpec("read", "Read CO2 averaged value.", expected_any=("CO2 avg:",), validators=("status_ok", "co2_avg")),
        CommandSpec("selftest", "Run safe command self-test.", expected_any=("Selftest result:",), validators=("selftest",), timeout_s=60.0),
        CommandSpec("drv", "Capture driver health.", expected_any=("Driver Health",), validators=("health_ready",)),
        CommandSpec("dirty", "Capture persistent dirty-state diagnostics.", expected_any=("persistentConfigDirty",), validators=("dirty_clean",)),
        CommandSpec("stress 50", "Run bounded CO2 read stress.", expected_any=("Stress Summary",), validators=("stress",), timeout_s=90.0),
        CommandSpec("drv", "Capture driver health after stress.", expected_any=("Driver Health",), validators=("health_ready",)),
        CommandSpec("dirty", "Confirm stress did not dirty persistent config.", expected_any=("persistentConfigDirty",), validators=("dirty_clean",)),
    ]


def extended_specs(read_count: int, cycle_count: int) -> list[CommandSpec]:
    specs = [
        CommandSpec("stress 500", "Extended bounded CO2 read stress.", group="extended", expected_any=("Stress Summary",), validators=("stress",), timeout_s=420.0),
    ]
    for _ in range(max(1, read_count)):
        specs.append(CommandSpec("read", "Extended repeated CO2 average read.", group="extended-read-loop", expected_any=("CO2 avg:",), validators=("status_ok", "co2_avg")))
    for _ in range(max(1, cycle_count)):
        specs.extend(
            [
                CommandSpec("probe", "Extended probe/read/selftest loop probe.", group="extended-cycle", expected_any=("Status:",), validators=("status_ok",)),
                CommandSpec("read", "Extended probe/read/selftest loop read.", group="extended-cycle", expected_any=("CO2 avg:",), validators=("status_ok", "co2_avg")),
                CommandSpec("selftest", "Extended probe/read/selftest loop selftest.", group="extended-cycle", expected_any=("Selftest result:",), validators=("selftest",), timeout_s=60.0),
            ]
        )
    specs.extend(
        [
            CommandSpec("recover", "Recover after extended safe sequence.", group="extended", expected_any=("Status:",), validators=("status_ok",), timeout_s=45.0),
            CommandSpec("drv", "Capture driver health after extended recovery.", group="extended", expected_any=("Driver Health",), validators=("health_ready",)),
            CommandSpec("dirty", "Capture dirty state after extended sequence.", group="extended", expected_any=("persistentConfigDirty",), validators=("dirty_clean",)),
        ]
    )
    return specs


def maintenance_specs(args: argparse.Namespace) -> list[CommandSpec]:
    specs = [
        CommandSpec("dirty", "Confirm clean dirty state before persistent writes.", group="maintenance", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-persistent-writes"),
        CommandSpec("interval", "Read measurement interval before write.", group="maintenance", expected_any=("Interval:",), validators=("status_ok", "interval_read"), requires_opt_in="--include-persistent-writes"),
    ]
    if args.maintenance_interval is None:
        specs.append(
            CommandSpec(
                "interval <current>",
                "Rewrite the currently reported measurement interval.",
                group="maintenance",
                expected_any=("Status:",),
                validators=("status_ok",),
                destructive=True,
                requires_opt_in="--include-persistent-writes --confirm-persistent-writes",
                dynamic="interval_current",
                notes="Uses the interval value parsed from the previous interval read.",
            )
        )
    else:
        specs.append(
            CommandSpec(
                f"interval {args.maintenance_interval}",
                "Write requested measurement interval.",
                group="maintenance",
                expected_any=("Status:",),
                validators=("status_ok",),
                destructive=True,
                requires_opt_in="--include-persistent-writes --confirm-persistent-writes",
            )
        )
    specs.extend(
        [
            CommandSpec("interval", "Read measurement interval after write.", group="maintenance", expected_any=("Interval:",), validators=("status_ok", "interval_read", "interval_expected"), requires_opt_in="--include-persistent-writes"),
            CommandSpec("dirty", "Capture dirty state after interval write.", group="maintenance", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-persistent-writes"),
            CommandSpec("resync", "Run explicit persistent config resync.", group="maintenance", expected_any=("Persistent Config Resync",), validators=("status_ok", "dirty_clean"), timeout_s=45.0, requires_opt_in="--include-persistent-writes"),
            CommandSpec("dirty", "Capture dirty state after resync.", group="maintenance", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-persistent-writes"),
        ]
    )
    if args.write_co2_offset is not None:
        specs.extend(
            [
                CommandSpec("offset", "Read CO2 offset before write.", group="maintenance-calibration", expected_any=("CO2 offset:",), validators=("status_ok", "offset_read"), requires_opt_in="--include-persistent-writes"),
                CommandSpec(f"offset {args.write_co2_offset}", "Write persistent CO2 offset.", group="maintenance-calibration", expected_any=("Status:",), validators=("status_ok",), destructive=True, requires_opt_in="--include-persistent-writes --confirm-persistent-writes"),
                CommandSpec("offset", "Read CO2 offset after write.", group="maintenance-calibration", expected_any=("CO2 offset:",), validators=("status_ok", "offset_read", "offset_expected"), requires_opt_in="--include-persistent-writes"),
                CommandSpec("dirty", "Capture dirty state after CO2 offset write.", group="maintenance-calibration", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-persistent-writes"),
            ]
        )
    if args.write_co2_gain is not None:
        specs.extend(
            [
                CommandSpec("gain", "Read CO2 gain before write.", group="maintenance-calibration", expected_any=("CO2 gain:",), validators=("status_ok", "gain_read"), requires_opt_in="--include-persistent-writes"),
                CommandSpec(f"gain {args.write_co2_gain}", "Write persistent CO2 gain.", group="maintenance-calibration", expected_any=("Status:",), validators=("status_ok",), destructive=True, requires_opt_in="--include-persistent-writes --confirm-persistent-writes"),
                CommandSpec("gain", "Read CO2 gain after write.", group="maintenance-calibration", expected_any=("CO2 gain:",), validators=("status_ok", "gain_read", "gain_expected"), requires_opt_in="--include-persistent-writes"),
                CommandSpec("dirty", "Capture dirty state after CO2 gain write.", group="maintenance-calibration", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-persistent-writes"),
            ]
        )
    return specs


def operator_fault_specs(args: argparse.Namespace) -> list[CommandSpec]:
    specs: list[CommandSpec] = []
    if args.include_unplug_replug:
        specs.extend(
            [
                CommandSpec("operator: unplug EE871", "Operator disconnects EE871 power or E2 lines.", group="fault-unplug", send=False, operator_required=True, requires_opt_in="--include-unplug-replug"),
                CommandSpec("read", "Expect bounded read failure while unplugged.", group="fault-unplug", expected_any=("Status:",), validators=("expected_failure",), timeout_s=30.0, requires_opt_in="--include-unplug-replug"),
                CommandSpec("drv", "Capture health after unplug failure.", group="fault-unplug", expected_any=("Driver Health",), validators=("health_faulted", "dirty_state"), requires_opt_in="--include-unplug-replug"),
                CommandSpec("operator: replug EE871", "Operator reconnects EE871 and waits for lines to idle.", group="fault-unplug", send=False, operator_required=True, requires_opt_in="--include-unplug-replug"),
                CommandSpec("recover", "Recover after replug.", group="fault-unplug", expected_any=("Status:",), validators=("status_ok",), timeout_s=45.0, requires_opt_in="--include-unplug-replug"),
                CommandSpec("drv", "Capture health after replug recovery.", group="fault-unplug", expected_any=("Driver Health",), validators=("health_ready",), requires_opt_in="--include-unplug-replug"),
            ]
        )
    if args.include_stuck_line:
        specs.extend(
            [
                CommandSpec("operator: apply stuck E2 line", "Operator applies a safe SCL/SDA stuck-line fault jig.", group="fault-stuck-line", send=False, operator_required=True, requires_opt_in="--include-stuck-line"),
                CommandSpec("buscheck", "Capture bounded bus-idle failure while line is stuck.", group="fault-stuck-line", expected_any=("Status:",), validators=("expected_failure",), timeout_s=30.0, requires_opt_in="--include-stuck-line"),
                CommandSpec("operator: release stuck E2 line", "Operator removes the stuck-line fault and confirms pull-ups recover.", group="fault-stuck-line", send=False, operator_required=True, requires_opt_in="--include-stuck-line"),
                CommandSpec("recover", "Recover after stuck-line fault.", group="fault-stuck-line", expected_any=("Status:",), validators=("status_ok",), timeout_s=45.0, requires_opt_in="--include-stuck-line"),
                CommandSpec("drv", "Capture health after stuck-line recovery.", group="fault-stuck-line", expected_any=("Driver Health",), validators=("health_ready",), requires_opt_in="--include-stuck-line"),
            ]
        )
    if args.include_power_cycle:
        specs.extend(
            [
                CommandSpec("operator: power-cycle target", "Operator power-cycles the EE871 and controller if required.", group="fault-power-cycle", send=False, operator_required=True, requires_opt_in="--include-power-cycle"),
                CommandSpec("version", "Capture firmware identity after power cycle.", group="fault-power-cycle", expected_any=("EE871 library version:",), validators=("version",), timeout_s=30.0, requires_opt_in="--include-power-cycle"),
                CommandSpec("drv", "Capture driver health after power cycle.", group="fault-power-cycle", expected_any=("Driver Health",), validators=("health_ready",), requires_opt_in="--include-power-cycle"),
                CommandSpec("dirty", "Capture dirty state after power cycle.", group="fault-power-cycle", expected_any=("persistentConfigDirty",), validators=("dirty_clean",), requires_opt_in="--include-power-cycle"),
            ]
        )
    return specs


def build_plan(args: argparse.Namespace) -> list[CommandSpec]:
    specs = safe_specs()
    if args.include_extended:
        specs.extend(extended_specs(args.read_loop_count, args.cycle_loop_count))
    if args.include_persistent_writes:
        specs.extend(maintenance_specs(args))
    specs.extend(operator_fault_specs(args))
    return specs


def confirm_persistent_runtime(args: argparse.Namespace) -> None:
    if not args.include_persistent_writes or args.dry_run:
        return
    print()
    print("Persistent EE871 writes may change sensor configuration across power cycles.")
    print("Run them only on a bench sensor where the original values have been recorded.")
    answer = input(f"Type '{PERSISTENT_RUNTIME_CONFIRM_TEXT}' to continue: ").strip()
    if answer != PERSISTENT_RUNTIME_CONFIRM_TEXT:
        print("Persistent-write confirmation was not provided.", file=sys.stderr)
        raise SystemExit(2)


def resolve_dynamic_command(spec: CommandSpec, state: dict[str, Any]) -> tuple[str | None, str | None]:
    if spec.dynamic == "interval_current":
        interval = state.get("measurement_interval_ds")
        if isinstance(interval, int):
            return f"interval {interval}", None
        return None, "cannot rewrite current interval because previous interval read was not parsed"
    return spec.command, None


def maintenance_write_block_reason(spec: CommandSpec, state: dict[str, Any]) -> str | None:
    if not spec.destructive or not spec.group.startswith("maintenance"):
        return None
    if state.get("persistent_config_dirty") is not False:
        return "not sent: persistent dirty state was not parsed as clean before maintenance write"
    if state.get("resync_needed") is not False:
        return "not sent: resync-needed state was not parsed as clean before maintenance write"
    return None


def result_row(
    spec: CommandSpec,
    command: str,
    result: str,
    reason: str,
    elapsed_s: float,
    raw: str,
    wait_reason: str,
    parsed: dict[str, Any],
) -> dict[str, Any]:
    return {
        "command": command,
        "planned_command": spec.command,
        "description": spec.description,
        "group": spec.group,
        "result": result,
        "reason": reason,
        "elapsed_s": round(elapsed_s, 3),
        "wait_reason": wait_reason,
        "destructive": spec.destructive,
        "operator_required": spec.operator_required,
        "requires_opt_in": spec.requires_opt_in,
        "notes": spec.notes,
        "parsed": parsed,
        "raw": raw,
        "clean_excerpt": strip_ansi(raw)[-1600:],
    }


def dry_run_row(spec: CommandSpec, state: dict[str, Any]) -> dict[str, Any]:
    command, reason = resolve_dynamic_command(spec, state)
    if reason:
        return result_row(spec, spec.command, RESULT_SKIP, f"dry-run unresolved dynamic command: {reason}", 0.0, "", "dry-run", {})
    if spec.operator_required:
        return result_row(spec, command or spec.command, RESULT_OPERATOR, "operator evidence required; dry-run did not execute", 0.0, "", "dry-run", {})
    return result_row(spec, command or spec.command, RESULT_SKIP, "dry-run did not execute serial command", 0.0, "", "dry-run", {})


def run_operator_step(spec: CommandSpec) -> dict[str, Any]:
    print()
    print(f"Operator step: {spec.description}")
    print("Type 'done' after performing the step, 'skip' to skip it, or 'abort' to stop.")
    try:
        answer = input("operator> ").strip().lower()
    except EOFError:
        answer = ""
    if answer == "abort":
        raise KeyboardInterrupt("operator aborted HIL run")
    if answer == "skip":
        return result_row(spec, spec.command, RESULT_SKIP, "operator skipped step", 0.0, "", "operator", {})
    if answer != "done":
        return result_row(spec, spec.command, RESULT_OPERATOR, "operator did not confirm step", 0.0, "", "operator", {})
    return result_row(spec, spec.command, RESULT_OPERATOR, "operator reported step done; external evidence required", 0.0, "", "operator", {})


def run_serial_command(
    ser: object,
    spec: CommandSpec,
    command: str,
    args: argparse.Namespace,
    state: dict[str, Any],
) -> dict[str, Any]:
    timeout_s = spec.timeout_s if spec.timeout_s is not None else args.command_timeout
    start = time.monotonic()
    ser.write((command + "\n").encode("utf-8"))
    flush = getattr(ser, "flush", None)
    if callable(flush):
        flush()
    response, wait_reason, timed_out = read_until_ready(ser, timeout_s, args.idle, command)
    elapsed = time.monotonic() - start
    parsed = parse_response(command, response)
    result, reason = classify_response(spec, response, timed_out, parsed, state)
    return result_row(spec, command, result, reason, elapsed, response, wait_reason, parsed)


def update_state(state: dict[str, Any], row: dict[str, Any]) -> None:
    parsed = row.get("parsed") or {}
    for key in (
        "firmware_build",
        "library_version",
        "library_full",
        "library_build",
        "library_commit",
        "library_git_status",
        "driver_state",
        "online",
        "consecutive_failures",
        "total_success",
        "total_failures",
        "persistent_config_dirty",
        "resync_needed",
        "measurement_interval_ds",
        "co2_offset_ppm",
        "co2_gain",
        "device_address",
    ):
        if key in parsed:
            state[key] = parsed[key]
    if "selftest" in parsed:
        state["last_selftest"] = parsed["selftest"]
    if "stress" in parsed:
        state["last_stress"] = parsed["stress"]


def record_persistent_write_expectation(row: dict[str, Any], state: dict[str, Any]) -> None:
    if row.get("result") != RESULT_PASS or not row.get("destructive"):
        return
    command = str(row.get("command", ""))
    match = re.fullmatch(r"interval\s+(\d+)", command)
    if match:
        state["expected_measurement_interval_ds"] = int(match.group(1))
        return
    match = re.fullmatch(r"offset\s+(-?\d+)", command)
    if match:
        state["expected_co2_offset_ppm"] = int(match.group(1))
        return
    match = re.fullmatch(r"gain\s+(\d+)", command)
    if match:
        state["expected_co2_gain"] = int(match.group(1))


def verdict(results: list[dict[str, Any]], dry_run: bool) -> str:
    if dry_run:
        return VERDICT_INCOMPLETE
    if not results:
        return VERDICT_INCOMPLETE
    values = {row.get("result") for row in results}
    if RESULT_FAIL in values:
        return VERDICT_FAIL
    if RESULT_OPERATOR in values:
        return VERDICT_OPERATOR
    if RESULT_SKIP in values:
        return VERDICT_INCOMPLETE
    return VERDICT_PASS if values == {RESULT_PASS} else VERDICT_INCOMPLETE


def exit_code_for_verdict(final: str) -> int:
    if final == VERDICT_PASS:
        return 0
    if final == VERDICT_FAIL:
        return 1
    if final == VERDICT_OPERATOR:
        return 2
    return 3


def counts(results: list[dict[str, Any]]) -> dict[str, int]:
    out = {RESULT_PASS: 0, RESULT_FAIL: 0, RESULT_SKIP: 0, RESULT_OPERATOR: 0}
    for row in results:
        result = str(row.get("result", ""))
        out[result] = out.get(result, 0) + 1
    return out


def make_log_dir(output_dir: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate = output_dir / f"ee871_{stamp}"
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = output_dir / f"ee871_{stamp}_{suffix}"
    candidate.mkdir(parents=True)
    return candidate


def metadata(
    args: argparse.Namespace,
    log_dir: Path,
    git_status: str,
    git_branch: str,
    git_commit: str,
) -> dict[str, Any]:
    return {
        "tool": "ee871_hil_runner.py",
        "tool_version": SCRIPT_VERSION,
        "timestamp_utc": iso_timestamp(),
        "port": args.port or "<dry-run>",
        "baud": args.baud,
        "timeout_s": args.timeout,
        "command_timeout_s": args.command_timeout,
        "idle_s": args.idle,
        "output_dir": str(log_dir),
        "dry_run": bool(args.dry_run),
        "board": args.board,
        "target_name": args.target_name,
        "operator": args.operator,
        "expected_device_address": args.device_address,
        "git_branch": git_branch,
        "git_commit": git_commit,
        "git_worktree": "unknown" if git_status == "unknown" else ("clean" if not git_status else "dirty"),
    }


def write_transcript(path: Path, meta: dict[str, Any], initial_output: str, results: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write("EE871-E2 serial HIL transcript\n")
        for key in ("timestamp_utc", "port", "baud", "dry_run", "git_branch", "git_commit", "git_worktree"):
            fh.write(f"{key}={meta.get(key)}\n")
        fh.write("\n")
        if initial_output:
            fh.write("=== initial serial output ===\n")
            fh.write(initial_output)
            if not initial_output.endswith("\n"):
                fh.write("\n")
            fh.write("\n")
        for idx, row in enumerate(results, 1):
            fh.write(f"=== command {idx}: {row['command']} ===\n")
            fh.write(f"result={row['result']} reason={row['reason']} elapsed_s={row['elapsed_s']} wait={row['wait_reason']}\n")
            raw = row.get("raw") or ""
            fh.write(raw)
            if raw and not raw.endswith("\n"):
                fh.write("\n")
            fh.write("\n")


def write_summary_json(
    path: Path,
    meta: dict[str, Any],
    results: list[dict[str, Any]],
    final: str,
    state: dict[str, Any],
    initial_output: str,
    aggregate_counts: dict[str, int],
) -> None:
    payload = {
        "metadata": meta,
        "final_verdict": final,
        "counts": aggregate_counts,
        "parsed_state": state,
        "initial_serial_output_present": bool(initial_output.strip()),
        "claim_boundary": (
            "PASS is limited to the selected automated serial EE871 CLI command groups. "
            "It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, "
            "fault tolerance, long-soak stability, calibration validity, or production readiness."
        ),
        "commands": [
            {key: value for key, value in row.items() if key != "raw"}
            for row in results
        ],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_summary_md(
    path: Path,
    meta: dict[str, Any],
    results: list[dict[str, Any]],
    final: str,
    state: dict[str, Any],
    aggregate_counts: dict[str, int],
) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write("# EE871-E2 HIL Summary\n\n")
        fh.write(f"Final verdict: `{final}`\n\n")
        fh.write("PASS is limited to the selected automated serial EE871 CLI command groups. It does not prove CO2 accuracy, warm-up suitability, persistent-write safety, fault tolerance, long-soak stability, calibration validity, or production readiness.\n\n")
        fh.write("## Run Metadata\n\n")
        for key in ("timestamp_utc", "port", "baud", "dry_run", "board", "target_name", "operator", "expected_device_address", "git_branch", "git_commit", "git_worktree"):
            fh.write(f"- {key}: `{meta.get(key)}`\n")
        fh.write("\n## Counts\n\n")
        for key in (RESULT_PASS, RESULT_FAIL, RESULT_SKIP, RESULT_OPERATOR):
            fh.write(f"- {key}: `{aggregate_counts.get(key, 0)}`\n")
        fh.write("\n## Parsed State\n\n")
        fh.write("```json\n")
        fh.write(json.dumps(state, indent=2))
        fh.write("\n```\n\n")
        fh.write("## Commands\n\n")
        fh.write("| # | Command | Group | Result | Elapsed s | Reason |\n")
        fh.write("| --- | --- | --- | --- | --- | --- |\n")
        for idx, row in enumerate(results, 1):
            reason = str(row.get("reason", "")).replace("|", "\\|")
            fh.write(
                f"| {idx} | `{row.get('command')}` | `{row.get('group')}` | `{row.get('result')}` | `{row.get('elapsed_s')}` | {reason} |\n"
            )
        fh.write("\n## Artifacts\n\n")
        fh.write("- `serial_transcript.txt`\n")
        fh.write("- `summary.json`\n")
        fh.write("- `summary.md`\n")


def open_serial(args: argparse.Namespace) -> object:
    try:
        import serial  # type: ignore
    except ImportError:
        print("pyserial is required for real serial HIL runs. Install it with: python -m pip install pyserial", file=sys.stderr)
        raise SystemExit(2)
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.05, write_timeout=2.0)
    try:
        ser.dtr = False
        ser.rts = False
    except (AttributeError, OSError):
        pass
    return ser


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example COM5 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="Startup/initial serial drain timeout in seconds.")
    parser.add_argument("--command-timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT_S)
    parser.add_argument("--idle", type=float, default=DEFAULT_IDLE_S, help="Idle gap after completion token before a command is considered complete.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--address", "--device-address", dest="device_address", default="0", help="Expected E2 device address metadata. This does not retarget firmware.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-extended", "--extended-safe", dest="include_extended", action="store_true")
    parser.add_argument("--read-loop-count", type=int, default=10)
    parser.add_argument("--cycle-loop-count", type=int, default=3)
    parser.add_argument("--include-persistent-writes", action="store_true")
    parser.add_argument(
        "--confirm-persistent-writes",
        nargs="?",
        const=PERSISTENT_CONFIRM_TEXT,
        default="",
        help=(
            "Required with --include-persistent-writes. May be passed as a flag; "
            f"an explicit value must match {PERSISTENT_CONFIRM_TEXT!r}."
        ),
    )
    parser.add_argument("--maintenance-interval", type=int, help="Measurement interval deciseconds to write; defaults to rewriting the parsed current value.")
    parser.add_argument("--write-co2-offset", type=int, help="Optional persistent CO2 offset write value. Requires persistent-write flags.")
    parser.add_argument("--write-co2-gain", type=int, help="Optional persistent CO2 gain write value. Requires persistent-write flags.")
    parser.add_argument("--include-unplug-replug", action="store_true")
    parser.add_argument("--include-stuck-line", action="store_true")
    parser.add_argument("--include-power-cycle", action="store_true")
    parser.add_argument("--board", default="unspecified")
    parser.add_argument("--target-name", default="unspecified")
    parser.add_argument("--operator", default="unspecified")
    args = parser.parse_args(argv)

    if not args.dry_run and not args.port:
        parser.error("--port is required unless --dry-run is used")
    if args.include_persistent_writes and args.confirm_persistent_writes != PERSISTENT_CONFIRM_TEXT:
        parser.error(
            "--include-persistent-writes requires "
            f"--confirm-persistent-writes {PERSISTENT_CONFIRM_TEXT!r}"
        )
    if (args.maintenance_interval is not None or args.write_co2_offset is not None or args.write_co2_gain is not None) and not (
        args.include_persistent_writes and args.confirm_persistent_writes
    ):
        parser.error("persistent write values require --include-persistent-writes --confirm-persistent-writes")
    if args.maintenance_interval is not None and not (150 <= args.maintenance_interval <= 36000):
        parser.error("--maintenance-interval must be 150..36000 deciseconds")
    if args.write_co2_offset is not None and not (-32768 <= args.write_co2_offset <= 32767):
        parser.error("--write-co2-offset must be -32768..32767")
    if args.write_co2_gain is not None and not (0 <= args.write_co2_gain <= 65535):
        parser.error("--write-co2-gain must be 0..65535")
    try:
        args.device_address = int(str(args.device_address), 0)
    except ValueError:
        parser.error("--address/--device-address must be an integer")
    if not (0 <= args.device_address <= 7):
        parser.error("--address/--device-address must be 0..7")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    confirm_persistent_runtime(args)
    plan = build_plan(args)
    git_status = git_value("status", "--short", empty_value="")
    git_branch = git_value("branch", "--show-current")
    git_commit = git_value("rev-parse", "--short=12", "HEAD")
    log_dir = make_log_dir(args.output_dir)
    meta = metadata(args, log_dir, git_status, git_branch, git_commit)
    results: list[dict[str, Any]] = []
    state: dict[str, Any] = {}
    initial_output = ""

    try:
        if args.dry_run:
            for spec in plan:
                row = dry_run_row(spec, state)
                results.append(row)
                update_state(state, row)
        else:
            ser = open_serial(args)
            try:
                initial_output, _, _ = read_until_ready(ser, args.timeout, args.idle, None)
                for spec in plan:
                    if spec.operator_required or not spec.send:
                        row = run_operator_step(spec)
                    else:
                        reason = maintenance_write_block_reason(spec, state)
                        command: str | None
                        if reason:
                            command = spec.command
                        else:
                            command, reason = resolve_dynamic_command(spec, state)
                        if reason:
                            result = RESULT_SKIP if reason.startswith("not sent:") else RESULT_OPERATOR
                            row = result_row(spec, spec.command, result, reason, 0.0, "", "not-sent", {})
                        else:
                            row = run_serial_command(ser, spec, command or spec.command, args, state)
                    results.append(row)
                    update_state(state, row)
                    record_persistent_write_expectation(row, state)
            finally:
                close = getattr(ser, "close", None)
                if callable(close):
                    close()
    except KeyboardInterrupt as exc:
        results.append(
            result_row(
                CommandSpec("operator abort", "Operator aborted run.", group="runner"),
                "operator abort",
                RESULT_SKIP,
                str(exc) or "operator aborted",
                0.0,
                "",
                "aborted",
                {},
            )
        )

    final = verdict(results, args.dry_run)
    aggregate_counts = counts(results)
    write_transcript(log_dir / "serial_transcript.txt", meta, initial_output, results)
    write_summary_json(log_dir / "summary.json", meta, results, final, state, initial_output, aggregate_counts)
    write_summary_md(log_dir / "summary.md", meta, results, final, state, aggregate_counts)

    print(f"Output directory: {log_dir}")
    print(f"Final verdict: {final}")
    return exit_code_for_verdict(final)


if __name__ == "__main__":
    raise SystemExit(main())
