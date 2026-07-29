#!/usr/bin/env python3
"""Run EE871-E2 serial HIL tests and capture repeatable evidence.

The runner drives the repository diagnostic CLI over a serial port. It does
not flash firmware and it does not claim hardware success from dry-runs.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_VERSION = "2.3"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 8.0
DEFAULT_COMMAND_TIMEOUT_S = 20.0
DEFAULT_IDLE_S = 0.35
DEFAULT_OUTPUT_DIR = Path("hil_logs")
PERSISTENT_CONFIRM_TEXT = "I UNDERSTAND EE871 PERSISTENT WRITES"
PERSISTENT_RUNTIME_CONFIRM_TEXT = "RUN EE871 PERSISTENT WRITES"
CALIBRATION_CONFIRM_TEXT = "I UNDERSTAND EE871 CALIBRATION WRITES"
CALIBRATION_RUNTIME_CONFIRM_TEXT = "RUN EE871 CALIBRATION WRITES"
ADDRESS_CONFIRM_TEXT = "I UNDERSTAND EE871 ADDRESS CHANGE"
ADDRESS_RESTORE_CONFIRM_TEXT = "RESTORE THE RECORDED EE871 ADDRESS"
ADDRESS_RUNTIME_CONFIRM_TEXT = "RUN EE871 ADDRESS CHANGE"
ADDRESS_ACTIVATE_CONFIRM_TEXT = "ACTIVATED THE AUTHORIZED EE871 ADDRESS"
ADDRESS_RESTORE_ACTIVATE_CONFIRM_TEXT = "ACTIVATED THE RECORDED EE871 ADDRESS"
AUTO_ADJUST_CONFIRM_TEXT = "I UNDERSTAND EE871 AUTO ADJUST CANNOT BE UNDONE"
AUTO_ADJUST_RUNTIME_CONFIRM_TEXT = "RUN EE871 AUTO ADJUST ONCE"
AUTO_ADJUST_CONDITIONS_CONFIRM_TEXT = "EE871 AUTO ADJUST CONDITIONS ARE CONTROLLED"
STUCK_LINE_CONFIRM_TEXT = "I UNDERSTAND EE871 STUCK LINE FAULTS"
STUCK_LINE_RUNTIME_CONFIRM_TEXT = "RUN EE871 STUCK LINE FAULTS"
POWER_CYCLE_CONFIRM_TEXT = "I UNDERSTAND EE871 SENSOR POWER CYCLE"
POWER_CYCLE_RUNTIME_CONFIRM_TEXT = "RUN EE871 SENSOR POWER CYCLE"

CUSTOM_MEMORY_SIZE = 256
CUSTOM_MEMORY_VOLATILE_ADDRESSES = frozenset({0xC1, 0xFE, 0xFF})

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
STATUS_CODE_BY_NAME = {
    "OK": 0,
    "NOT_INITIALIZED": 1,
    "INVALID_CONFIG": 2,
    "E2_ERROR": 3,
    "TIMEOUT": 4,
    "INVALID_PARAM": 5,
    "DEVICE_NOT_FOUND": 6,
    "PEC_MISMATCH": 7,
    "NACK": 8,
    "BUSY": 9,
    "IN_PROGRESS": 10,
    "BUS_STUCK": 11,
    "ALREADY_INITIALIZED": 12,
    "OUT_OF_RANGE": 13,
    "NOT_SUPPORTED": 14,
    "VERIFY_MISMATCH": 15,
    "OFFLINE": 16,
    "CO2_SENSOR_ERROR": 17,
    "PERSISTENT_STATE_UNCERTAIN": 18,
}

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
    capture: tuple[str, ...] = ()
    operator_confirm_text: str | None = None


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
        r"\bStatus:\s*([A-Z0-9_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)",
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
    match = last_match(r"\bFirmware:\s*([0-9]+\.[0-9]+)", clean, re.IGNORECASE)
    if match:
        parsed["device_firmware_version"] = match.group(1)
    match = last_match(r"\bE2 spec version:\s*(\d+)", clean, re.IGNORECASE)
    if match:
        parsed["e2_spec_version"] = int(match.group(1))
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
        r"\bpersistentConfigDirtyError:\s*([A-Z0-9_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)",
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
    matches = list(re.finditer(r"\bmutation\.unresolved:\s*(yes|no|true|false|0|1)", clean, re.IGNORECASE))
    if matches:
        parsed["mutation_unresolved"] = parse_boolish(matches[-1].group(1))
    matches = list(re.finditer(
        r"\bmutation\.target:\s*([A-Z0-9_]+)\s*\(value=(\d+)\)",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["mutation_target"] = match.group(1).upper()
        parsed["mutation_target_value"] = int(match.group(2))
    matches = list(re.finditer(
        r"\bmutation\.effect:\s*([A-Z0-9_]+)\s*\(value=(\d+)\)",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["mutation_effect"] = match.group(1).upper()
        parsed["mutation_effect_value"] = int(match.group(2))
    matches = list(re.finditer(
        r"\bmutation\.addresses:\s*first=0x([0-9A-Fa-f]{2})\s+last=0x([0-9A-Fa-f]{2})",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["mutation_first_address"] = int(match.group(1), 16)
        parsed["mutation_last_address"] = int(match.group(2), 16)
    matches = list(re.finditer(
        r"\bmutation\.elements:\s*requested=(\d+)\s+acknowledged=(\d+)\s+observed=(\d+)\s+matched=(\d+)",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["mutation_elements_requested"] = int(match.group(1))
        parsed["mutation_elements_acknowledged"] = int(match.group(2))
        parsed["mutation_elements_observed"] = int(match.group(3))
        parsed["mutation_elements_matched"] = int(match.group(4))
    matches = list(re.finditer(r"\bmutation\.attemptedValue:\s*0x([0-9A-Fa-f]{2})", clean, re.IGNORECASE))
    if matches:
        parsed["mutation_attempted_value"] = int(matches[-1].group(1), 16)
    for prefix, key in (
        ("preObservedValue", "mutation_pre_observed"),
        ("observedValue", "mutation_observed"),
    ):
        matches = list(re.finditer(
            rf"\bmutation\.{prefix}:\s*valid=(yes|no|true|false|0|1)\s+value=0x([0-9A-Fa-f]{{2}})",
            clean,
            re.IGNORECASE,
        ))
        if matches:
            match = matches[-1]
            parsed[f"{key}_valid"] = parse_boolish(match.group(1))
            parsed[f"{key}_value"] = int(match.group(2), 16)
    matches = list(re.finditer(
        r"\bmutation\.cause:\s*([A-Z0-9_]+)\s*\(code=(\d+),\s*detail=(-?\d+)\)",
        clean,
        re.IGNORECASE,
    ))
    if matches:
        match = matches[-1]
        parsed["mutation_cause"] = {
            "name": match.group(1).upper(),
            "code": int(match.group(2)),
            "detail": int(match.group(3)),
        }
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
    match = last_match(r"\bCO2 interval factor:\s*(-?\d+)", clean, re.IGNORECASE)
    if match:
        parsed["co2_interval_factor"] = int(match.group(1))
    match = last_match(r"\bCO2 filter:\s*(\d+)", clean, re.IGNORECASE)
    if match:
        parsed["co2_filter"] = int(match.group(1))
    match = last_match(r"\bOperating mode:\s*0x([0-9A-Fa-f]{2})", clean, re.IGNORECASE)
    if match:
        parsed["operating_mode"] = int(match.group(1), 16)
    match = last_match(r"\bPart name hex:\s*([0-9A-Fa-f]{32})", clean, re.IGNORECASE)
    if match:
        parsed["part_name_hex"] = match.group(1).upper()
    match = last_match(r"\bCal points:\s*lower=(\d+)\s*ppm,\s*upper=(\d+)\s*ppm", clean, re.IGNORECASE)
    if match:
        parsed["co2_cal_points"] = {
            "lower_ppm": int(match.group(1)),
            "upper_ppm": int(match.group(2)),
        }
    match = last_match(r"\bAuto adjustment:\s*(RUNNING|idle)", clean, re.IGNORECASE)
    if match:
        parsed["auto_adjust_running"] = match.group(1).upper() == "RUNNING"
    match = last_match(r"\bStatus:\s*0x([0-9A-Fa-f]{2})", clean, re.IGNORECASE)
    if match:
        parsed["sensor_status_byte"] = int(match.group(1), 16)
    return parsed


def parse_checked_sample(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    match = last_match(r"\bSample kind:\s*(FAST|AVERAGE)\s*\(MV([34])\)", clean, re.IGNORECASE)
    if match:
        parsed["checked_sample_kind"] = match.group(1).upper()
        parsed["checked_sample_mv"] = int(match.group(2))
    match = last_match(
        r"\bValue step:\s*attempted=(yes|no|true|false|0|1)\s+status=([A-Z0-9_]+)\s+detail=(-?\d+)",
        clean,
        re.IGNORECASE,
    )
    if match:
        parsed["checked_value_attempted"] = parse_boolish(match.group(1))
        parsed["checked_value_status"] = match.group(2).upper()
        parsed["checked_value_detail"] = int(match.group(3))
    match = last_match(r"\bCO2 value:\s*(\d+)\s*ppm,\s*valid=(yes|no|true|false|0|1)", clean, re.IGNORECASE)
    if match:
        parsed["checked_ppm"] = int(match.group(1))
        parsed["checked_ppm_valid"] = parse_boolish(match.group(2))
    match = last_match(
        r"\bStatus step:\s*attempted=(yes|no|true|false|0|1)\s+status=([A-Z0-9_]+)\s+detail=(-?\d+)\s+valid=(yes|no|true|false|0|1)"
        r"(?:,\s*byte=0x([0-9A-Fa-f]{2}),\s*co2Error=(yes|no|true|false|0|1))?",
        clean,
        re.IGNORECASE,
    )
    if match:
        parsed["checked_status_attempted"] = parse_boolish(match.group(1))
        parsed["checked_status_status"] = match.group(2).upper()
        parsed["checked_status_detail"] = int(match.group(3))
        parsed["checked_status_valid"] = parse_boolish(match.group(4))
        if match.group(5):
            parsed["checked_status_byte"] = int(match.group(5), 16)
            parsed["checked_co2_error"] = parse_boolish(match.group(6))
    match = last_match(
        r"\bError-code step:\s*attempted=(yes|no|true|false|0|1)\s+status=([A-Z0-9_]+)\s+detail=(-?\d+)\s+valid=(yes|no|true|false|0|1)"
        r"(?:,\s*code=(\d+)\s*\([^)]+\))?",
        clean,
        re.IGNORECASE,
    )
    if match:
        parsed["checked_error_attempted"] = parse_boolish(match.group(1))
        parsed["checked_error_status"] = match.group(2).upper()
        parsed["checked_error_detail"] = int(match.group(3))
        parsed["checked_error_valid"] = parse_boolish(match.group(4))
        if match.group(5):
            parsed["checked_error_code"] = int(match.group(5))
    match = last_match(r"\bSensor error:\s*([A-Z0-9_]+)\s*\(enum=(\d+)\)", clean, re.IGNORECASE)
    if match:
        parsed["checked_sensor_error"] = match.group(1).upper()
        parsed["checked_sensor_error_value"] = int(match.group(2))
    return parsed


def parse_features(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    for key, pattern in (
        ("operating_functions", r"Operating functions \(0x07\):\s*0x([0-9A-Fa-f]{2})"),
        ("operating_mode_support", r"Mode support \(0x08\):\s*0x([0-9A-Fa-f]{2})"),
        ("special_features", r"Special features \(0x09\):\s*0x([0-9A-Fa-f]{2})"),
    ):
        match = last_match(pattern, clean, re.IGNORECASE)
        if match:
            parsed[key] = int(match.group(1), 16)
    caps: dict[str, bool] = {}
    for match in re.finditer(r"\b(has[A-Za-z0-9]+):\s*(true|false)", clean, re.IGNORECASE):
        value = parse_boolish(match.group(2))
        if value is not None:
            caps[match.group(1)] = value
    if caps:
        parsed["capabilities"] = caps
    return parsed


def parse_custom_dump(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    rows: list[tuple[int, list[int]]] = []
    for match in re.finditer(
        r"(?m)^\s*0x([0-9A-Fa-f]{2}):((?:\s+[0-9A-Fa-f]{2})+)\s*$",
        clean,
    ):
        start = int(match.group(1), 16)
        values = [int(token, 16) for token in match.group(2).split()]
        rows.append((start, values))
    if not rows:
        return {}
    memory: list[int | None] = [None] * CUSTOM_MEMORY_SIZE
    errors: list[str] = []
    for start, values in rows:
        for offset, value in enumerate(values):
            address = start + offset
            if address >= CUSTOM_MEMORY_SIZE:
                errors.append(f"row at 0x{start:02X} exceeds custom memory")
                continue
            if memory[address] is not None:
                errors.append(f"duplicate custom-memory address 0x{address:02X}")
                continue
            memory[address] = value
    missing = [address for address, value in enumerate(memory) if value is None]
    return {
        "custom_memory": [value if value is not None else -1 for value in memory],
        "custom_memory_complete": not errors and not missing,
        "custom_memory_errors": errors,
        "custom_memory_missing": missing,
        "custom_memory_rows": len(rows),
    }


def parse_bus_levels(text: str) -> dict[str, Any]:
    clean = strip_ansi(text)
    parsed: dict[str, Any] = {}
    for line, key in (("SCL", "scl_high"), ("SDA", "sda_high")):
        match = last_match(rf"\b{line}:\s*(HIGH|LOW)", clean, re.IGNORECASE)
        if match:
            parsed[key] = match.group(1).upper() == "HIGH"
    if "Bus is idle (both lines high)" in clean:
        parsed["bus_idle"] = True
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
    parsed.update(parse_checked_sample(text))
    parsed.update(parse_features(text))
    parsed.update(parse_custom_dump(text))
    parsed.update(parse_bus_levels(text))
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


def validate_checked_sample(
    parsed: dict[str, Any],
    expected_kind: str,
    state: dict[str, Any] | None = None,
) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    reviews: list[str] = []
    required = (
        "checked_sample_kind",
        "checked_sample_mv",
        "checked_value_attempted",
        "checked_value_status",
        "checked_value_detail",
        "checked_ppm",
        "checked_ppm_valid",
        "checked_status_attempted",
        "checked_status_status",
        "checked_status_detail",
        "checked_status_valid",
        "checked_error_attempted",
        "checked_error_status",
        "checked_error_detail",
        "checked_error_valid",
        "checked_sensor_error",
        "checked_sensor_error_value",
    )
    missing = [key for key in required if key not in parsed]
    if missing:
        reviews.append(f"checked sample fields not parsed: {', '.join(missing)}")
        return failures, reviews
    if parsed.get("checked_sample_kind") != expected_kind:
        failures.append(
            f"checked sample kind {parsed.get('checked_sample_kind')} != {expected_kind}"
        )
    expected_mv = 3 if expected_kind == "FAST" else 4
    if parsed.get("checked_sample_mv") != expected_mv:
        failures.append(
            f"checked sample MV{parsed.get('checked_sample_mv')} != MV{expected_mv}"
        )
    if parsed.get("checked_value_attempted") is not True:
        failures.append("checked value read was not attempted")
    if parsed.get("checked_status_attempted") is not True:
        failures.append("checked status read was not attempted")
    if parsed.get("checked_value_status") != "OK":
        failures.append(f"checked value status is {parsed.get('checked_value_status')}")
    elif parsed.get("checked_value_detail") != 0:
        failures.append(f"checked value OK detail is {parsed.get('checked_value_detail')}")
    if parsed.get("checked_status_status") != "OK":
        failures.append(f"checked status status is {parsed.get('checked_status_status')}")
    elif parsed.get("checked_status_detail") != 0:
        failures.append(f"checked status OK detail is {parsed.get('checked_status_detail')}")
    if parsed.get("checked_status_valid") is not True:
        failures.append("checked status byte is not valid")

    top = parsed.get("status") or {}
    co2_error = parsed.get("checked_co2_error")
    if parsed.get("checked_status_valid") is True and not isinstance(co2_error, bool):
        reviews.append("checked CO2-error flag not parsed for valid status byte")
    status_byte = parsed.get("checked_status_byte")
    if parsed.get("checked_status_valid") is True and not isinstance(status_byte, int):
        reviews.append("checked status byte not parsed for valid status")
    elif isinstance(status_byte, int) and isinstance(co2_error, bool):
        if bool(status_byte & 0x08) != co2_error:
            failures.append(
                f"status byte 0x{status_byte:02X} bit3 disagrees with co2Error={co2_error}"
            )
    if co2_error is False:
        if parsed.get("checked_ppm_valid") is not True:
            failures.append("clean checked ppm is not valid")
        if parsed.get("checked_error_attempted") is not False:
            failures.append("error-code read was attempted without CO2 error")
        if parsed.get("checked_error_status") != "OK":
            failures.append(
                f"clean sample error-step status is {parsed.get('checked_error_status')}"
            )
        if parsed.get("checked_error_detail") != 0:
            failures.append(
                f"clean sample error-step detail is {parsed.get('checked_error_detail')}"
            )
        if parsed.get("checked_error_valid") is not False:
            failures.append("error code is valid without CO2 error")
        if parsed.get("checked_sensor_error") != "NONE":
            failures.append(
                f"sensor error is {parsed.get('checked_sensor_error')} without CO2 error"
            )
        if parsed.get("checked_sensor_error_value") != 0:
            failures.append(
                f"clean sensor-error enum is {parsed.get('checked_sensor_error_value')}"
            )
        if "checked_error_code" in parsed:
            failures.append("clean sample unexpectedly includes an error code")
        if top.get("name") != "OK" or top.get("code") != 0:
            failures.append(f"clean checked sample returned {top.get('name')}")
    elif co2_error is True:
        if parsed.get("checked_ppm_valid") is not False:
            failures.append("sensor-error checked ppm was incorrectly marked valid")
        operating_functions = (state or {}).get("operating_functions")
        if not isinstance(operating_functions, int):
            reviews.append("error-code capability was not recorded before checked sample")
            return failures, reviews
        has_error_code = bool(operating_functions & 0x80)
        if has_error_code:
            if parsed.get("checked_error_attempted") is not True:
                failures.append("CO2 error did not attempt advertised error-code read")
            if parsed.get("checked_error_status") != "OK":
                failures.append(
                    f"checked error-code status is {parsed.get('checked_error_status')}"
                )
            elif parsed.get("checked_error_detail") != 0:
                failures.append(
                    f"checked error-code OK detail is {parsed.get('checked_error_detail')}"
                )
            if parsed.get("checked_error_valid") is not True:
                failures.append("CO2 error code is not valid")
            error_code = parsed.get("checked_error_code")
            if not isinstance(error_code, int):
                failures.append("CO2 error code value was not parsed")
            expected_sensor = {
                1: ("SUPPLY_VOLTAGE_LOW", 1),
                200: ("SENSOR_COUNTS_LOW", 200),
                201: ("SENSOR_COUNTS_HIGH", 201),
                202: ("SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK", 202),
            }.get(error_code, ("UNKNOWN", 255))
            if (
                parsed.get("checked_sensor_error"),
                parsed.get("checked_sensor_error_value"),
            ) != expected_sensor:
                failures.append(
                    "sensor-error enum/name is inconsistent with "
                    f"error code {error_code}"
                )
            if top.get("name") != "CO2_SENSOR_ERROR" or top.get("code") != 17:
                failures.append(
                    "CO2 sensor error returned "
                    f"{top.get('name')} (code={top.get('code')})"
                )
            elif isinstance(error_code, int) and top.get("detail") != error_code:
                failures.append(
                    f"CO2 sensor status detail {top.get('detail')} != error code {error_code}"
                )
        else:
            if parsed.get("checked_error_attempted") is not False:
                failures.append("unadvertised error-code read was attempted")
            if parsed.get("checked_error_status") != "OK":
                failures.append(
                    "unattempted error-code step status is "
                    f"{parsed.get('checked_error_status')}"
                )
            elif parsed.get("checked_error_detail") != 0:
                failures.append(
                    "unattempted error-code step OK detail is "
                    f"{parsed.get('checked_error_detail')}"
                )
            if parsed.get("checked_error_valid") is not False:
                failures.append("unadvertised error code was marked valid")
            if "checked_error_code" in parsed:
                failures.append("unadvertised detailed error code was emitted")
            if (
                parsed.get("checked_sensor_error"),
                parsed.get("checked_sensor_error_value"),
            ) != ("UNKNOWN", 255):
                failures.append("unsupported detailed-error path did not report UNKNOWN (255)")
            if top.get("name") != "CO2_SENSOR_ERROR" or top.get("code") != 17:
                failures.append(
                    "CO2 sensor error returned "
                    f"{top.get('name')} (code={top.get('code')})"
                )
            elif isinstance(status_byte, int) and top.get("detail") != status_byte:
                failures.append(
                    f"CO2 sensor status detail {top.get('detail')} != status byte {status_byte}"
                )
        failures.append(
            f"sensor reported {parsed.get('checked_sensor_error')}"
            + (
                f" (code={parsed.get('checked_error_code')})"
                if "checked_error_code" in parsed
                else f" (status=0x{status_byte:02X})" if isinstance(status_byte, int) else ""
            )
        )
    return failures, reviews


def custom_memory_diff(
    baseline: list[int],
    current: list[int],
    allowed_nonvolatile: frozenset[int] = frozenset(),
    mutation_target: str | None = None,
) -> tuple[list[dict[str, int]], list[dict[str, int]]]:
    expected: list[dict[str, int]] = []
    unexpected: list[dict[str, int]] = []
    if len(baseline) != CUSTOM_MEMORY_SIZE or len(current) != CUSTOM_MEMORY_SIZE:
        return expected, [{"address": -1, "before": len(baseline), "after": len(current)}]
    for address, (before, after) in enumerate(zip(baseline, current)):
        if before == after:
            continue
        item = {"address": address, "before": before, "after": after}
        target_allows_address = (
            address in allowed_nonvolatile
            and (address != 0xD9 or mutation_target == "AUTO_ADJUST")
        )
        if address in CUSTOM_MEMORY_VOLATILE_ADDRESSES or target_allows_address:
            expected.append(item)
        else:
            unexpected.append(item)
    return expected, unexpected


def is_complete_custom_memory_image(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == CUSTOM_MEMORY_SIZE
        and all(type(item) is int and 0 <= item <= 0xFF for item in value)
    )


def validate_parsed(
    spec: CommandSpec,
    parsed: dict[str, Any],
    state: dict[str, Any] | None = None,
) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    reviews: list[str] = []
    context = state or {}
    status = parsed.get("status") or {}
    optional_not_supported = (
        "status_optional" in spec.validators
        and status.get("name") == "NOT_SUPPORTED"
    )
    statuses = parsed.get("statuses")
    if not isinstance(statuses, list):
        statuses = [status] if status else []
    for reported_status in statuses:
        if not isinstance(reported_status, dict):
            failures.append("parsed status evidence is not structured")
            continue
        name = reported_status.get("name")
        code = reported_status.get("code")
        expected_code = STATUS_CODE_BY_NAME.get(name)
        if expected_code is None:
            failures.append(f"unrecognized status name {name}")
        elif code != expected_code:
            failures.append(
                f"status {name} reported code {code}, expected {expected_code}"
            )

    for validator in spec.validators:
        if validator == "version":
            if not parsed.get("library_version"):
                reviews.append("library version not parsed")
        elif validator == "device_fw_read":
            if not parsed.get("device_firmware_version"):
                reviews.append("device firmware version not parsed")
        elif validator == "e2spec_read":
            if not parsed.get("e2_spec_version"):
                reviews.append("E2 specification version not parsed")
        elif validator == "status_ok":
            status = parsed.get("status")
            if not status:
                reviews.append("status line not parsed")
            elif status.get("name") != "OK" or status.get("code") != 0:
                failures.append(f"status is {status.get('name')}")
        elif validator == "status_optional":
            status = parsed.get("status")
            if not status:
                reviews.append("status line not parsed")
            elif status.get("name") not in {"OK", "NOT_SUPPORTED"}:
                failures.append(f"optional command status is {status.get('name')}")
        elif validator == "status_address_uncertain":
            status = parsed.get("status")
            if not status:
                reviews.append("address-change status line not parsed")
            elif status.get("name") != "PERSISTENT_STATE_UNCERTAIN":
                failures.append(f"address change status is {status.get('name')}")
        elif validator == "status_auto_adjust_start":
            status = parsed.get("status")
            if not status:
                reviews.append("auto-adjust status line not parsed")
            elif status.get("name") not in {"OK", "PERSISTENT_STATE_UNCERTAIN"}:
                failures.append(f"auto-adjust start status is {status.get('name')}")
        elif validator == "co2_avg":
            if "co2_avg_ppm" not in parsed:
                reviews.append("CO2 averaged value not parsed")
        elif validator == "co2_fast":
            if "co2_fast_ppm" not in parsed:
                reviews.append("CO2 fast value not parsed")
        elif validator == "checked_fast":
            checked_failures, checked_reviews = validate_checked_sample(parsed, "FAST", context)
            failures.extend(checked_failures)
            reviews.extend(checked_reviews)
        elif validator == "checked_average":
            checked_failures, checked_reviews = validate_checked_sample(parsed, "AVERAGE", context)
            failures.extend(checked_failures)
            reviews.extend(checked_reviews)
        elif validator == "bus_idle":
            if parsed.get("bus_idle") is not True:
                failures.append("bus idle confirmation was not parsed")
        elif validator == "levels_idle":
            if "scl_high" not in parsed or "sda_high" not in parsed:
                reviews.append("SCL/SDA levels not parsed")
            elif parsed.get("scl_high") is not True or parsed.get("sda_high") is not True:
                failures.append("E2 lines are not both high")
        elif validator == "levels_sda_low":
            if "scl_high" not in parsed or "sda_high" not in parsed:
                reviews.append("SCL/SDA levels not parsed")
            elif parsed.get("scl_high") is not True or parsed.get("sda_high") is not False:
                failures.append("fault jig did not show SCL high and SDA low")
        elif validator == "levels_scl_low":
            if "scl_high" not in parsed or "sda_high" not in parsed:
                reviews.append("SCL/SDA levels not parsed")
            elif parsed.get("scl_high") is not False or parsed.get("sda_high") is not True:
                failures.append("fault jig did not show SCL low and SDA high")
        elif validator == "features":
            for key in ("operating_functions", "operating_mode_support", "special_features"):
                if key not in parsed:
                    reviews.append(f"{key} not parsed")
        elif validator == "caps_consistent":
            caps = parsed.get("capabilities")
            if not isinstance(caps, dict):
                reviews.append("capability booleans not parsed")
            else:
                mapping = {
                    "hasSerialNumber": ("operating_functions", 0x01),
                    "hasPartName": ("operating_functions", 0x02),
                    "hasAddressConfig": ("operating_functions", 0x04),
                    "hasGlobalInterval": ("operating_functions", 0x10),
                    "hasSpecificInterval": ("operating_functions", 0x20),
                    "hasFilterConfig": ("operating_functions", 0x40),
                    "hasErrorCode": ("operating_functions", 0x80),
                    "hasLowPowerMode": ("operating_mode_support", 0x01),
                    "hasE2Priority": ("operating_mode_support", 0x02),
                    "hasAutoAdjust": ("special_features", 0x01),
                }
                for cap, (field, mask) in mapping.items():
                    bits = context.get(field)
                    if not isinstance(bits, int):
                        reviews.append(f"feature byte {field} not recorded before caps")
                        continue
                    if caps.get(cap) != bool(bits & mask):
                        failures.append(f"{cap} disagrees with {field}")
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
        elif validator == "health_failures_unchanged":
            before = context.get("total_failures")
            after = parsed.get("total_failures")
            if not isinstance(before, int):
                reviews.append("pre-sample total-failure counter was not recorded")
            elif not isinstance(after, int):
                reviews.append("post-sample total-failure counter was not parsed")
            elif after != before:
                failures.append(
                    f"transport total failures changed across checked sample: {before} -> {after}"
                )
        elif validator == "dirty_clean":
            if "persistent_config_dirty" not in parsed:
                reviews.append("persistent dirty flag not parsed")
            elif parsed.get("persistent_config_dirty") is not False:
                failures.append("persistent config is dirty")
            if "resync_needed" not in parsed:
                reviews.append("resync-needed flag not parsed")
            elif parsed.get("resync_needed") is not False:
                failures.append("resync is needed")
            if "mutation_unresolved" not in parsed:
                reviews.append("mutation unresolved flag not parsed")
            elif parsed.get("mutation_unresolved") is not False:
                failures.append("mutation is unresolved")
            dirty_error = parsed.get("persistent_config_dirty_error")
            if not isinstance(dirty_error, dict):
                reviews.append("persistent dirty error not parsed")
            elif (
                parsed.get("mutation_unresolved") is False
                and (
                    dirty_error.get("name") != "OK"
                    or dirty_error.get("code") != 0
                    or dirty_error.get("detail") != 0
                )
            ):
                failures.append(
                    "resolved mutation state has non-OK persistent dirty error"
                )
        elif validator == "dirty_state":
            required_dirty = (
                "persistent_config_dirty",
                "persistent_config_dirty_error",
                "resync_needed",
                "mutation_unresolved",
                "mutation_target",
                "mutation_target_value",
                "mutation_effect",
                "mutation_effect_value",
                "mutation_first_address",
                "mutation_last_address",
                "mutation_elements_requested",
                "mutation_elements_acknowledged",
                "mutation_elements_observed",
                "mutation_elements_matched",
                "mutation_attempted_value",
                "mutation_pre_observed_valid",
                "mutation_pre_observed_value",
                "mutation_observed_valid",
                "mutation_observed_value",
                "mutation_cause",
            )
            missing = [key for key in required_dirty if key not in parsed]
            if missing:
                reviews.append(
                    "complete mutation diagnostic not parsed: "
                    + ", ".join(missing)
                )
        elif validator == "interval_read":
            if not optional_not_supported and "measurement_interval_ds" not in parsed:
                reviews.append("measurement interval not parsed")
        elif validator == "offset_read":
            if not optional_not_supported and "co2_offset_ppm" not in parsed:
                reviews.append("CO2 offset not parsed")
        elif validator == "gain_read":
            if not optional_not_supported and "co2_gain" not in parsed:
                reviews.append("CO2 gain not parsed")
        elif validator == "factor_read":
            if not optional_not_supported and "co2_interval_factor" not in parsed:
                reviews.append("CO2 interval factor not parsed")
        elif validator == "filter_read":
            if not optional_not_supported and "co2_filter" not in parsed:
                reviews.append("CO2 filter not parsed")
        elif validator == "mode_read":
            if not optional_not_supported and "operating_mode" not in parsed:
                reviews.append("operating mode not parsed")
        elif validator == "part_name_hex_read":
            if not optional_not_supported and "part_name_hex" not in parsed:
                reviews.append("exact part name not parsed")
        elif validator == "address_read":
            if "device_address" not in parsed:
                reviews.append("bus address not parsed")
        elif validator == "address_matches_configured":
            configured = context.get("configured_device_address")
            actual = parsed.get("device_address")
            if not isinstance(configured, int):
                reviews.append("configured device address not recorded")
            elif not isinstance(actual, int):
                reviews.append("persistent bus address not parsed")
            elif actual != configured:
                failures.append(
                    f"persistent bus address {actual} != configured address {configured}"
                )
        elif validator == "calpoints_read":
            if not optional_not_supported and "co2_cal_points" not in parsed:
                reviews.append("CO2 calibration points not parsed")
        elif validator == "typed_capability_baseline":
            capability_map: dict[str, tuple[bool | None, str]] = {
                "partnamehex": (
                    bool(context.get("operating_functions", 0) & 0x02)
                    if isinstance(context.get("operating_functions"), int)
                    else None,
                    "part_name_hex",
                ),
                "addr": (
                    bool(context.get("operating_functions", 0) & 0x04)
                    if isinstance(context.get("operating_functions"), int)
                    else None,
                    "device_address",
                ),
                "interval": (
                    bool(context.get("operating_functions", 0) & 0x10)
                    if isinstance(context.get("operating_functions"), int)
                    else None,
                    "measurement_interval_ds",
                ),
                "factor": (
                    bool(context.get("operating_functions", 0) & 0x20)
                    if isinstance(context.get("operating_functions"), int)
                    else None,
                    "co2_interval_factor",
                ),
                "filter": (
                    bool(context.get("operating_functions", 0) & 0x40)
                    if isinstance(context.get("operating_functions"), int)
                    else None,
                    "co2_filter",
                ),
                "mode": (
                    bool(context.get("operating_mode_support", 0) & 0x03)
                    if isinstance(context.get("operating_mode_support"), int)
                    else None,
                    "operating_mode",
                ),
            }
            advertised, value_key = capability_map.get(
                spec.command, (None, "")
            )
            if not isinstance(advertised, bool):
                reviews.append(
                    f"capability evidence for {spec.command} was not recorded"
                )
            elif advertised:
                if status.get("name") != "OK" or value_key not in parsed:
                    failures.append(
                        f"advertised {spec.command} baseline was not read successfully"
                    )
            elif status.get("name") != "NOT_SUPPORTED":
                failures.append(
                    f"unadvertised {spec.command} baseline did not return NOT_SUPPORTED"
                )
            elif value_key in parsed:
                failures.append(
                    f"unadvertised {spec.command} returned contradictory typed state"
                )
        elif validator == "auto_adjust_read":
            if not optional_not_supported and "auto_adjust_running" not in parsed:
                reviews.append("auto-adjust status not parsed")
        elif validator == "auto_adjust_preflight":
            special = context.get("special_features")
            if not isinstance(special, int):
                reviews.append("special-features byte not recorded before auto-adjust preflight")
            elif special & 0x01:
                if status.get("name") != "OK":
                    failures.append(
                        f"advertised auto-adjust status is {status.get('name')}"
                    )
                elif parsed.get("auto_adjust_running") is not False:
                    failures.append("advertised auto-adjust is not proven idle")
            else:
                if status.get("name") != "NOT_SUPPORTED":
                    failures.append(
                        "unadvertised auto-adjust did not return clean NOT_SUPPORTED"
                    )
                if "auto_adjust_running" in parsed:
                    failures.append(
                        "unadvertised auto-adjust returned contradictory typed state"
                    )
        elif validator == "auto_adjust_idle":
            if "auto_adjust_running" not in parsed:
                reviews.append("auto-adjust status not parsed")
            elif parsed.get("auto_adjust_running") is not False:
                failures.append("auto-adjust is already running")
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
        elif validator in {
            "factor_expected",
            "mode_expected",
            "part_name_hex_expected",
            "address_expected",
        }:
            field_map = {
                "factor_expected": ("expected_co2_interval_factor", "co2_interval_factor", "CO2 interval factor"),
                "mode_expected": ("expected_operating_mode", "operating_mode", "operating mode"),
                "part_name_hex_expected": ("expected_part_name_hex", "part_name_hex", "part name"),
                "address_expected": ("expected_device_address", "device_address", "bus address"),
            }
            expected_key, actual_key, label = field_map[validator]
            expected = context.get(expected_key)
            actual = parsed.get(actual_key)
            if expected is None:
                reviews.append(f"expected {label} not recorded")
            elif actual != expected:
                failures.append(f"{label} readback {actual} != expected {expected}")
        elif validator == "custom_memory_complete":
            if parsed.get("custom_memory_complete") is not True:
                errors = parsed.get("custom_memory_errors") or []
                missing = parsed.get("custom_memory_missing") or []
                failures.append(
                    f"custom-memory snapshot incomplete: errors={errors}, missing={len(missing)}"
                )
        elif validator == "custom_memory_restored":
            baseline = context.get("baseline_custom_memory")
            current = parsed.get("custom_memory")
            if not isinstance(baseline, list):
                reviews.append("baseline custom-memory image not recorded")
            elif not isinstance(current, list) or parsed.get("custom_memory_complete") is not True:
                failures.append("final custom-memory image is incomplete")
            else:
                expected_diff, unexpected_diff = custom_memory_diff(baseline, current)
                parsed["custom_memory_expected_diff"] = expected_diff
                parsed["custom_memory_unexpected_diff"] = unexpected_diff
                if unexpected_diff:
                    addresses = ", ".join(
                        f"0x{item['address']:02X}" for item in unexpected_diff[:16]
                    )
                    failures.append(f"unexpected custom-memory changes at {addresses}")
        elif validator == "custom_memory_target_only":
            baseline = context.get("baseline_custom_memory")
            current = parsed.get("custom_memory")
            first = context.get("expected_mutation_first_address")
            last = context.get("expected_mutation_last_address")
            if not isinstance(baseline, list):
                reviews.append("baseline custom-memory image not recorded")
            elif not isinstance(first, int) or not isinstance(last, int) or first > last:
                reviews.append("expected mutation address range not recorded")
            elif not isinstance(current, list) or parsed.get("custom_memory_complete") is not True:
                failures.append("post-test custom-memory image is incomplete")
            else:
                allowed = frozenset(range(first, last + 1))
                expected_diff, unexpected_diff = custom_memory_diff(
                    baseline,
                    current,
                    allowed,
                    context.get("expected_mutation_target"),
                )
                parsed["custom_memory_expected_diff"] = expected_diff
                parsed["custom_memory_unexpected_diff"] = unexpected_diff
                parsed["custom_memory_allowed_target_addresses"] = sorted(allowed)
                if not any(item["address"] in allowed for item in expected_diff):
                    failures.append(
                        "post-test image does not show a changed byte in the selected target"
                    )
                if unexpected_diff:
                    addresses = ", ".join(
                        f"0x{item['address']:02X}" for item in unexpected_diff[:16]
                    )
                    failures.append(
                        f"post-test write changed bytes outside the selected target at {addresses}"
                    )
        elif validator == "mutation_verified":
            expected_target = context.get("expected_mutation_target")
            expected_count = context.get("expected_mutation_count")
            expected_first = context.get("expected_mutation_first_address")
            expected_last = context.get("expected_mutation_last_address")
            if parsed.get("mutation_unresolved") is not False:
                failures.append("mutation remains unresolved")
            if parsed.get("mutation_target") != expected_target:
                failures.append(
                    f"mutation target {parsed.get('mutation_target')} != {expected_target}"
                )
            if parsed.get("mutation_effect") != "VERIFIED":
                failures.append(f"mutation effect is {parsed.get('mutation_effect')}")
            if parsed.get("mutation_first_address") != expected_first:
                failures.append(
                    f"mutation first address {parsed.get('mutation_first_address')} "
                    f"!= {expected_first}"
                )
            if parsed.get("mutation_last_address") != expected_last:
                failures.append(
                    f"mutation last address {parsed.get('mutation_last_address')} "
                    f"!= {expected_last}"
                )
            cause = parsed.get("mutation_cause") or {}
            if cause.get("name") != "OK" or cause.get("code") != 0:
                failures.append(f"mutation cause is {cause.get('name')}")
            if isinstance(expected_count, int):
                for key in (
                    "mutation_elements_requested",
                    "mutation_elements_acknowledged",
                    "mutation_elements_observed",
                    "mutation_elements_matched",
                ):
                    if parsed.get(key) != expected_count:
                        failures.append(f"{key}={parsed.get(key)} != {expected_count}")
            else:
                reviews.append("expected mutation element count not recorded")
        elif validator == "mutation_address_unresolved":
            if parsed.get("mutation_unresolved") is not True:
                failures.append("address mutation is not unresolved")
            if parsed.get("mutation_target") != "BUS_ADDRESS":
                failures.append(f"mutation target is {parsed.get('mutation_target')}")
            if parsed.get("mutation_effect") != "ACKNOWLEDGED":
                failures.append(f"address mutation effect is {parsed.get('mutation_effect')}")
            if parsed.get("mutation_elements_requested") != 1:
                failures.append("address mutation requested count is not 1")
            if parsed.get("mutation_elements_acknowledged") != 1:
                failures.append("address mutation acknowledged count is not 1")
            if parsed.get("mutation_first_address") != 0xC0:
                failures.append("address mutation first address is not 0xC0")
            if parsed.get("mutation_last_address") != 0xC0:
                failures.append("address mutation last address is not 0xC0")
        elif validator == "mutation_address_reconciled":
            if parsed.get("mutation_unresolved") is not False:
                failures.append("address mutation remains unresolved")
            if parsed.get("mutation_target") != "BUS_ADDRESS":
                failures.append(f"mutation target is {parsed.get('mutation_target')}")
            if parsed.get("mutation_effect") not in {"VERIFIED", "RESYNCHRONIZED"}:
                failures.append(f"address mutation effect is {parsed.get('mutation_effect')}")
            if parsed.get("mutation_elements_observed") != 1:
                failures.append("address mutation observed count is not 1")
            if parsed.get("mutation_elements_matched") != 1:
                failures.append("address mutation matched count is not 1")
            if parsed.get("mutation_first_address") != 0xC0:
                failures.append("address mutation first address is not 0xC0")
            if parsed.get("mutation_last_address") != 0xC0:
                failures.append("address mutation last address is not 0xC0")
        elif validator == "mutation_auto_adjust":
            if parsed.get("mutation_target") != "AUTO_ADJUST":
                failures.append(f"mutation target is {parsed.get('mutation_target')}")
            if parsed.get("mutation_first_address") != 0xD9:
                failures.append("auto-adjust mutation first address is not 0xD9")
            if parsed.get("mutation_last_address") != 0xD9:
                failures.append("auto-adjust mutation last address is not 0xD9")
            effect = parsed.get("mutation_effect")
            unresolved = parsed.get("mutation_unresolved")
            if (effect, unresolved) not in {
                ("VERIFIED", False),
                ("ACKNOWLEDGED", True),
                ("RESYNCHRONIZED", False),
            }:
                failures.append(
                    f"auto-adjust mutation evidence is effect={effect}, unresolved={unresolved}"
                )
        elif validator in {"health_faulted", "health_faulted_since_pre"}:
            driver_state = parsed.get("driver_state")
            consecutive = parsed.get("consecutive_failures")
            online = parsed.get("online")
            before_failures = context.get("fault_pre_total_failures")
            after_failures = parsed.get("total_failures")
            if not driver_state:
                reviews.append("driver state not parsed")
            if not isinstance(consecutive, int):
                reviews.append("consecutive failures not parsed")
            if "online" not in parsed:
                reviews.append("online flag not parsed")
            if validator == "health_faulted_since_pre":
                if context.get("fault_pre_group") != spec.group:
                    reviews.append(
                        "pre-fault health snapshot does not belong to this fault group"
                    )
                elif not isinstance(before_failures, int):
                    reviews.append("pre-fault total-failure counter not recorded")
                elif not isinstance(after_failures, int):
                    reviews.append("in-fault total-failure counter not parsed")
                elif after_failures <= before_failures:
                    failures.append(
                        f"transport failures did not increase: {before_failures} -> {after_failures}"
                    )
            if isinstance(consecutive, int) and consecutive < 1:
                failures.append("in-fault consecutive failures is less than one")
            if driver_state == "DEGRADED" and online is not True:
                failures.append("DEGRADED driver is not online")
            elif driver_state == "OFFLINE" and online is not False:
                failures.append("OFFLINE driver is still online")
            elif driver_state in {"READY", "UNINIT"}:
                failures.append(f"in-fault driver state is {driver_state}")
        elif validator == "expected_failure":
            status = parsed.get("status")
            if not status:
                reviews.append("bounded failure status not parsed")
            elif status.get("name") == "OK" and status.get("code") == 0:
                failures.append("command reported OK during operator fault step")
        elif validator == "fault_bus_line":
            status = parsed.get("status")
            if not status:
                reviews.append("fault status line not parsed")
            elif status.get("name") != "BUS_STUCK":
                failures.append(f"fault status is {status.get('name')}")
        else:
            reviews.append(f"unknown validator {validator}")

    return failures, reviews


def status_is_intentionally_accepted(
    spec: CommandSpec,
    parsed: dict[str, Any],
) -> bool:
    status = parsed.get("status")
    if not isinstance(status, dict):
        return False
    name = status.get("name")
    validators = set(spec.validators)
    if name == "NOT_SUPPORTED" and "status_optional" in validators:
        return True
    if (
        name == "PERSISTENT_STATE_UNCERTAIN"
        and "status_address_uncertain" in validators
    ):
        return True
    if (
        name in {"OK", "PERSISTENT_STATE_UNCERTAIN"}
        and "status_auto_adjust_start" in validators
    ):
        return True
    if "expected_failure" in validators and name != "OK":
        return True
    if "fault_bus_line" in validators and name == "BUS_STUCK":
        return True
    return False


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

    failures, reviews = validate_parsed(spec, parsed, state)
    if failures:
        return RESULT_FAIL, "; ".join(failures)
    status = parsed.get("status")
    if (
        isinstance(status, dict)
        and status.get("name") != "OK"
        and not status_is_intentionally_accepted(spec, parsed)
    ):
        return RESULT_FAIL, f"status is {status.get('name')}"
    if not expected_token_present(clean, spec):
        return RESULT_OPERATOR, "expected output token missing"
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
    if command in {"samplefast", "sampleavg"}:
        return "Sensor error:" in clean and "Error-code step:" in clean
    if command == "caps":
        return "=== Capabilities ===" in clean and "hasAutoAdjust:" in clean
    if command == "levels":
        return re.search(r"\bSCL:\s*(?:HIGH|LOW)", clean) is not None and re.search(
            r"\bSDA:\s*(?:HIGH|LOW)", clean
        ) is not None
    if command.startswith("reg dump"):
        return "=== Custom Register Dump ===" in clean and re.search(
            r"(?m)^\s*0xF0:(?:\s+[0-9A-Fa-f]{2}){16}\s*$", clean
        ) is not None
    if command.startswith("addr rebegin"):
        return "=== Address Candidate Rebegin/Resync ===" in clean and "After:" in clean
    if command in {
        "read", "probe", "recover", "status", "co2fast", "co2avg", "features",
        "fw", "e2spec", "interval", "factor", "filter", "mode", "offset",
        "gain", "addr", "partnamehex", "autoadj", "buscheck", "libreset",
    } or command.startswith(
        ("interval ", "factor ", "mode ", "offset ", "gain ",
         "addr ", "partnamehex ", "autoadj ")
    ):
        return "Status:" in clean
    return "Status:" in clean or PROMPT_RE.search(clean) is not None


def read_until_ready(
    ser: object,
    timeout_s: float,
    idle_s: float,
    command: str | None = None,
    require_prompt: bool = False,
) -> tuple[str, str, bool]:
    deadline = time.monotonic() + timeout_s
    last_data_at = time.monotonic()
    data_seen = False
    completion_seen = command is None and not require_prompt
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
        if data_seen and command is None and not require_prompt and (time.monotonic() - last_data_at) >= idle_s:
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
        CommandSpec("buscheck", "Confirm the E2 bus is idle.", group="complete-safe", expected_any=("Bus is idle",), validators=("status_ok", "bus_idle")),
        CommandSpec("levels", "Record released E2 line levels.", group="complete-safe", expected_any=("SCL:", "SDA:"), validators=("levels_idle",)),
        CommandSpec("status", "Read side-effecting EE871 status.", group="complete-safe", expected_any=("hasCo2Error():",), validators=("status_ok",)),
        CommandSpec("co2fast", "Read raw MV3 fast-response CO2.", group="complete-safe", expected_any=("CO2 fast:",), validators=("status_ok", "co2_fast")),
        CommandSpec("co2avg", "Read raw MV4 averaged CO2.", group="complete-safe", expected_any=("CO2 avg:",), validators=("status_ok", "co2_avg")),
        CommandSpec("features", "Read all feature bytes before checked-sample validation.", group="complete-safe", expected_any=("Operating functions (0x07):",), validators=("status_ok", "features"), capture=("operating_functions", "operating_mode_support", "special_features")),
        CommandSpec("caps", "Compare cached capabilities with feature bytes.", group="complete-safe", expected_any=("Capabilities",), validators=("caps_consistent",)),
        CommandSpec("drv", "Capture transport health immediately before checked samples.", group="complete-safe", expected_any=("Driver Health",), validators=("health_ready",)),
        CommandSpec("samplefast", "Run checked MV3 value/status/error procedure.", group="complete-safe", expected_any=("Sample kind:", "Sensor error:"), validators=("checked_fast",)),
        CommandSpec("drv", "Prove the checked MV3 procedure did not invent a transport failure.", group="complete-safe", expected_any=("Driver Health",), validators=("health_ready", "health_failures_unchanged")),
        CommandSpec("sampleavg", "Run checked MV4 value/status/error procedure.", group="complete-safe", expected_any=("Sample kind:", "Sensor error:"), validators=("checked_average",)),
        CommandSpec("drv", "Prove the checked MV4 procedure did not invent a transport failure.", group="complete-safe", expected_any=("Driver Health",), validators=("health_ready", "health_failures_unchanged")),
        CommandSpec("fw", "Read device firmware version.", group="complete-safe", expected_any=("Firmware:",), validators=("status_ok",)),
        CommandSpec("e2spec", "Read device E2 specification version.", group="complete-safe", expected_any=("E2 spec version:",), validators=("status_ok",)),
        CommandSpec("stress_mix 100", "Exercise the full mixed safe-read stress row.", group="complete-safe", expected_any=("stress_mix summary",), validators=("stress",), timeout_s=300.0),
        CommandSpec("dirty", "Confirm complete-safe reads did not mutate persistent state.", group="complete-safe", expected_any=("mutation.target",), validators=("dirty_clean", "dirty_state")),
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
            CommandSpec("resync", "Run safe full persistent coherence read.", group="extended", expected_any=("Persistent Config Resync",), validators=("status_ok", "dirty_clean"), timeout_s=90.0),
            CommandSpec("drv", "Capture driver health after extended recovery.", group="extended", expected_any=("Driver Health",), validators=("health_ready",)),
            CommandSpec("dirty", "Capture dirty state after extended sequence.", group="extended", expected_any=("persistentConfigDirty",), validators=("dirty_clean",)),
        ]
    )
    return specs


def baseline_specs(group: str, opt_in: str) -> list[CommandSpec]:
    return [
        CommandSpec("fw", "Record device firmware version before maintenance.", group=group, expected_any=("Firmware:",), validators=("status_ok", "device_fw_read"), requires_opt_in=opt_in, capture=("device_firmware_version",)),
        CommandSpec("e2spec", "Record diagnostic E2 specification version before maintenance.", group=group, expected_any=("E2 spec version:",), validators=("status_ok", "e2spec_read"), requires_opt_in=opt_in, capture=("e2_spec_version",)),
        CommandSpec("features", "Record feature bytes before maintenance.", group=group, expected_any=("Operating functions (0x07):",), validators=("status_ok", "features"), requires_opt_in=opt_in, capture=("operating_functions", "operating_mode_support", "special_features")),
        CommandSpec("caps", "Record cached capability booleans.", group=group, expected_any=("Capabilities",), validators=("caps_consistent",), requires_opt_in=opt_in),
        CommandSpec("dirty", "Require a fresh clean mutation diagnostic.", group=group, expected_any=("mutation.target",), validators=("dirty_clean", "dirty_state"), requires_opt_in=opt_in),
        CommandSpec("reg dump 0 256", "Capture the complete pre-write custom-memory image.", group=group, expected_any=("Custom Register Dump",), validators=("custom_memory_complete",), timeout_s=90.0, requires_opt_in=opt_in, capture=("custom_memory",)),
        CommandSpec("serial", "Record sensor serial bytes.", group=group, expected_any=("Status:",), validators=("status_optional",), requires_opt_in=opt_in),
        CommandSpec("partnamehex", "Record exact 16-byte part-name baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "part_name_hex_read", "typed_capability_baseline"), requires_opt_in=opt_in, capture=("part_name_hex",)),
        CommandSpec("addr", "Record and match the persistent bus-address baseline.", group=group, expected_any=("Status:",), validators=("status_ok", "address_read", "typed_capability_baseline", "address_matches_configured"), requires_opt_in=opt_in, capture=("device_address",)),
        CommandSpec("interval", "Record global-interval baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "interval_read", "typed_capability_baseline"), requires_opt_in=opt_in, capture=("measurement_interval_ds",)),
        CommandSpec("factor", "Record specific-interval factor baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "factor_read", "typed_capability_baseline"), requires_opt_in=opt_in, capture=("co2_interval_factor",)),
        CommandSpec("filter", "Record read-only CO2-filter evidence.", group=group, expected_any=("Status:",), validators=("status_optional", "filter_read", "typed_capability_baseline"), requires_opt_in=opt_in, capture=("co2_filter",)),
        CommandSpec("mode", "Record operating-mode baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "mode_read", "typed_capability_baseline"), requires_opt_in=opt_in, capture=("operating_mode",)),
        CommandSpec("offset", "Record CO2-offset baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "offset_read"), requires_opt_in=opt_in, capture=("co2_offset_ppm",)),
        CommandSpec("gain", "Record CO2-gain baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "gain_read"), requires_opt_in=opt_in, capture=("co2_gain",)),
        CommandSpec("calpoints", "Record calibration-point baseline.", group=group, expected_any=("Status:",), validators=("status_optional", "calpoints_read"), requires_opt_in=opt_in, capture=("co2_cal_points",)),
        CommandSpec("autoadj", "Prove capability-aware auto-adjust preflight.", group=group, expected_any=("Status:",), validators=("status_optional", "auto_adjust_read", "auto_adjust_preflight"), requires_opt_in=opt_in, capture=("auto_adjust_running",)),
    ]


def reversible_target_specs(
    *,
    group: str,
    target: str,
    read_command: str,
    write_command: str,
    dynamic_test: str | None,
    dynamic_restore: str,
    read_validators: tuple[str, ...],
    count: int,
    opt_in: str,
) -> list[CommandSpec]:
    test_spec = CommandSpec(
        write_command,
        f"Write the authorized {target} test value.",
        group=group,
        expected_any=("Status:",),
        validators=("status_ok",),
        destructive=True,
        requires_opt_in=opt_in,
        dynamic=dynamic_test,
        notes="A later restore is admitted only after a fresh verified-clean diagnostic.",
    )
    return [
        test_spec,
        CommandSpec(read_command, f"Verify the {target} test value.", group=group, expected_any=("Status:",), validators=("status_ok", *read_validators), requires_opt_in=opt_in),
        CommandSpec("dirty", f"Verify exact {target} mutation evidence.", group=group, expected_any=("mutation.elements",), validators=("dirty_clean", "dirty_state", "mutation_verified"), requires_opt_in=opt_in),
        CommandSpec(
            "reg dump 0 256",
            f"Capture the complete post-test {target} image before any restoration.",
            group=group,
            expected_any=("Custom Register Dump",),
            validators=("custom_memory_complete", "custom_memory_target_only"),
            timeout_s=90.0,
            requires_opt_in=opt_in,
            notes="Only the selected typed target and documented volatile bytes may differ from baseline.",
        ),
        CommandSpec(f"{read_command} <recorded-baseline>", f"Restore the recorded {target} baseline.", group=group, expected_any=("Status:",), validators=("status_ok",), destructive=True, requires_opt_in=opt_in, dynamic=dynamic_restore, notes="Never sent after any failed or uncertain maintenance step."),
        CommandSpec(read_command, f"Observe {target} after the restoration attempt (forensic read if restoration was blocked).", group=group, expected_any=("Status:",), validators=("status_ok", *read_validators), requires_opt_in=opt_in),
        CommandSpec("dirty", f"Verify exact {target} restoration evidence.", group=group, expected_any=("mutation.elements",), validators=("dirty_clean", "dirty_state", "mutation_verified"), requires_opt_in=opt_in, notes=f"Expected mutation element count: {count}."),
        CommandSpec(
            "reg dump 0 256",
            f"Verify the complete image after restoring {target}.",
            group=group,
            expected_any=("Custom Register Dump",),
            validators=("custom_memory_complete", "custom_memory_restored"),
            timeout_s=90.0,
            requires_opt_in=opt_in,
        ),
    ]


def maintenance_specs(args: argparse.Namespace) -> list[CommandSpec]:
    opt_in = "--include-persistent-writes --confirm-persistent-writes"
    specs = baseline_specs("maintenance-baseline", opt_in)
    if not args.include_calibration_writes:
        interval_command = (
            f"interval {args.maintenance_interval}"
            if args.maintenance_interval is not None
            else "interval <safe-alternate>"
        )
        specs.extend(reversible_target_specs(
            group="maintenance-interval",
            target="measurement interval",
            read_command="interval",
            write_command=interval_command,
            dynamic_test=None if args.maintenance_interval is not None else "interval_test",
            dynamic_restore="interval_baseline",
            read_validators=("interval_read", "interval_expected"),
            count=2,
            opt_in=opt_in,
        ))
        optional_targets = (
            (args.write_interval_factor, "factor", "CO2 interval factor", "factor_baseline", ("factor_read", "factor_expected"), 1),
            (args.write_operating_mode, "mode", "operating mode", "mode_baseline", ("mode_read", "mode_expected"), 1),
            (args.write_part_name_hex, "partnamehex", "exact part name", "part_name_hex_baseline", ("part_name_hex_read", "part_name_hex_expected"), 16),
        )
        for value, command, label, restore_dynamic, validators, count in optional_targets:
            if value is None:
                continue
            specs.extend(reversible_target_specs(
                group=f"maintenance-{command}",
                target=label,
                read_command=command,
                write_command=f"{command} {value}",
                dynamic_test=None,
                dynamic_restore=restore_dynamic,
                read_validators=validators,
                count=count,
                opt_in=opt_in,
            ))

    calibration_opt_in = (
        "--include-persistent-writes --include-calibration-writes "
        "--confirm-calibration-writes"
    )
    for value, command, label, restore_dynamic, validators in (
        (args.write_co2_offset, "offset", "CO2 offset calibration", "offset_baseline", ("offset_read", "offset_expected")),
        (args.write_co2_gain, "gain", "CO2 gain calibration", "gain_baseline", ("gain_read", "gain_expected")),
    ):
        if value is None:
            continue
        specs.extend(reversible_target_specs(
            group=f"maintenance-calibration-{command}",
            target=label,
            read_command=command,
            write_command=f"{command} {value}",
            dynamic_test=None,
            dynamic_restore=restore_dynamic,
            read_validators=validators,
            count=2,
            opt_in=calibration_opt_in,
        ))

    specs = [
        *specs,
        CommandSpec("reg dump 0 256", "Capture and compare the final custom-memory image.", group="maintenance-final", expected_any=("Custom Register Dump",), validators=("custom_memory_complete", "custom_memory_restored"), timeout_s=90.0, requires_opt_in=opt_in, capture=("final_custom_memory",)),
        CommandSpec("dirty", "Confirm final persistent state is resolved.", group="maintenance-final", expected_any=("mutation.target",), validators=("dirty_clean", "dirty_state"), requires_opt_in=opt_in),
    ]
    return specs


def address_change_specs(args: argparse.Namespace) -> list[CommandSpec]:
    opt_in = "--include-address-change --confirm-address-change"
    candidate = args.candidate_address
    specs = baseline_specs("address-baseline", opt_in)
    specs.extend(
        [
            CommandSpec(
                f"addr {candidate}",
                "Issue exactly one authorized address-candidate write.",
                group="address-candidate",
                expected_any=("Status:",),
                validators=("status_address_uncertain",),
                destructive=True,
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "dirty",
                "Prove the retained candidate is acknowledged and unresolved.",
                group="address-candidate",
                expected_any=("mutation.target",),
                validators=("dirty_state", "mutation_address_unresolved"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "operator: activate address candidate",
                "Keep the controller powered; perform the approved sensor-only address activation procedure. Do not scan.",
                group="address-candidate",
                send=False,
                operator_required=True,
                requires_opt_in=opt_in,
                dynamic="address_candidate_diagnostic_passed",
                operator_confirm_text=ADDRESS_ACTIVATE_CONFIRM_TEXT,
            ),
            CommandSpec(
                f"addr rebegin {candidate}",
                "End, rebegin at the explicit retained candidate, and resync without scanning.",
                group="address-candidate",
                expected_any=("Address Candidate Rebegin/Resync",),
                validators=("status_ok", "dirty_clean", "dirty_state", "mutation_address_reconciled"),
                timeout_s=90.0,
                requires_opt_in=opt_in,
                dynamic="address_candidate_activation_confirmed",
            ),
            CommandSpec(
                "addr",
                "Read back the reconciled candidate address.",
                group="address-candidate",
                expected_any=("Bus address:",),
                validators=("status_ok", "address_read", "address_expected"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "dirty",
                "Capture fresh resolved mutation evidence before address restoration.",
                group="address-candidate",
                expected_any=("mutation.target",),
                validators=("dirty_clean", "dirty_state", "mutation_address_reconciled"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "reg dump 0 256",
                "Capture the complete candidate-address image before restoration authorization.",
                group="address-candidate",
                expected_any=("Custom Register Dump",),
                validators=("custom_memory_complete", "custom_memory_target_only"),
                timeout_s=90.0,
                requires_opt_in=opt_in,
                notes="Only address 0xC0 and documented volatile bytes may differ from baseline.",
            ),
            CommandSpec(
                "operator: authorize address restoration",
                "Authorize a second complete address-change workflow back to the recorded baseline.",
                group="address-restore",
                send=False,
                operator_required=True,
                operator_confirm_text=ADDRESS_RESTORE_CONFIRM_TEXT,
                requires_opt_in="--confirm-address-restore",
                dynamic="address_candidate_verified",
            ),
            CommandSpec(
                "addr <recorded-baseline>",
                "Request restoration of the recorded original address.",
                group="address-restore",
                expected_any=("Status:",),
                validators=("status_address_uncertain",),
                destructive=True,
                requires_opt_in="--confirm-address-restore",
                dynamic="address_baseline_write",
            ),
            CommandSpec(
                "dirty",
                "Prove the retained original-address request is acknowledged and unresolved.",
                group="address-restore",
                expected_any=("mutation.target",),
                validators=("dirty_state", "mutation_address_unresolved"),
                requires_opt_in="--confirm-address-restore",
            ),
            CommandSpec(
                "operator: activate restored address",
                "Keep the controller powered; repeat the approved sensor-only activation procedure. Do not scan.",
                group="address-restore",
                send=False,
                operator_required=True,
                requires_opt_in="--confirm-address-restore",
                dynamic="address_restore_diagnostic_passed",
                operator_confirm_text=ADDRESS_RESTORE_ACTIVATE_CONFIRM_TEXT,
            ),
            CommandSpec(
                "addr rebegin <recorded-baseline>",
                "Rebegin at the explicit original address and resync without scanning.",
                group="address-restore",
                expected_any=("Address Candidate Rebegin/Resync",),
                validators=("status_ok", "dirty_clean", "dirty_state", "mutation_address_reconciled"),
                timeout_s=90.0,
                requires_opt_in="--confirm-address-restore",
                dynamic="address_restore_activation_confirmed",
            ),
            CommandSpec(
                "addr",
                "Verify the restored original address.",
                group="address-restore",
                expected_any=("Bus address:",),
                validators=("status_ok", "address_read", "address_expected"),
                requires_opt_in="--confirm-address-restore",
            ),
            CommandSpec(
                "reg dump 0 256",
                "Capture and compare the post-restore custom-memory image.",
                group="address-final",
                expected_any=("Custom Register Dump",),
                validators=("custom_memory_complete", "custom_memory_restored"),
                timeout_s=90.0,
                requires_opt_in=opt_in,
                capture=("final_custom_memory",),
            ),
            CommandSpec(
                "dirty",
                "Confirm final address state is resolved.",
                group="address-final",
                expected_any=("mutation.target",),
                validators=("dirty_clean", "dirty_state", "mutation_address_reconciled"),
                requires_opt_in=opt_in,
            ),
        ]
    )
    return specs


def auto_adjust_specs(args: argparse.Namespace) -> list[CommandSpec]:
    opt_in = "--include-auto-adjust --confirm-auto-adjust"
    specs = baseline_specs("auto-adjust-baseline", opt_in)
    specs.extend(
        [
            CommandSpec(
                "autoadj",
                "Require auto-adjust to be idle before the one-shot request.",
                group="auto-adjust",
                expected_any=("Auto adjustment:",),
                validators=("status_ok", "auto_adjust_read", "auto_adjust_idle"),
                requires_opt_in=opt_in,
                capture=("auto_adjust_running",),
            ),
            CommandSpec(
                "operator: confirm controlled auto-adjust conditions",
                "Confirm vendor-approved calibration conditions and authority. This action cannot be cancelled or restored.",
                group="auto-adjust",
                send=False,
                operator_required=True,
                requires_opt_in=opt_in,
                dynamic="auto_adjust_idle",
                operator_confirm_text=AUTO_ADJUST_CONDITIONS_CONFIRM_TEXT,
            ),
            CommandSpec(
                "autoadj start",
                "Start auto-adjust exactly once; the runner never retries it.",
                group="auto-adjust",
                expected_any=("Status:",),
                validators=("status_auto_adjust_start",),
                destructive=True,
                requires_opt_in=opt_in,
                dynamic="auto_adjust_authorized",
                notes="Non-replayable and non-restorable.",
            ),
            CommandSpec(
                "dirty",
                "Capture auto-adjust pre/post mutation evidence.",
                group="auto-adjust",
                expected_any=("mutation.target",),
                validators=("dirty_state", "mutation_auto_adjust"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "autoadj",
                "Observe auto-adjust status without replaying the request.",
                group="auto-adjust",
                expected_any=("Status:",),
                validators=("status_ok", "auto_adjust_read"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "dirty",
                "Capture final auto-adjust evidence without acknowledging or restoring it.",
                group="auto-adjust",
                expected_any=("mutation.target",),
                validators=("dirty_state", "mutation_auto_adjust"),
                requires_opt_in=opt_in,
            ),
            CommandSpec(
                "reg dump 0 256",
                "Capture the post-auto-adjust forensic image without treating changes as restorable.",
                group="auto-adjust-final",
                expected_any=("Custom Register Dump",),
                validators=("custom_memory_complete",),
                timeout_s=90.0,
                requires_opt_in=opt_in,
                capture=("final_custom_memory",),
            ),
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
        for line, level_validator in (("SDA", "levels_sda_low"), ("SCL", "levels_scl_low")):
            group = f"fault-{line.lower()}-low"
            specs.extend(
                [
                    CommandSpec("levels", f"Record idle levels before the {line}-low fault.", group=group, expected_any=("SCL:", "SDA:"), validators=("levels_idle",), requires_opt_in="--include-stuck-line"),
                    CommandSpec("drv", f"Capture health before the {line}-low fault.", group=group, expected_any=("Driver Health",), validators=("health_ready",), requires_opt_in="--include-stuck-line"),
                    CommandSpec(f"operator: apply {line}-low jig", f"Apply the reviewed open-drain/current-limited {line}-low fault jig; never force a line high.", group=group, send=False, operator_required=True, requires_opt_in="--include-stuck-line", dynamic="fault_preflight_passed"),
                    CommandSpec("levels", f"Record line levels during the {line}-low fault.", group=group, expected_any=("SCL:", "SDA:"), validators=(level_validator,), requires_opt_in="--include-stuck-line"),
                    CommandSpec("buscheck", f"Capture bounded raw bus-idle failure during {line}-low.", group=group, expected_any=("Status:",), validators=("fault_bus_line",), timeout_s=30.0, requires_opt_in="--include-stuck-line"),
                    CommandSpec("status", f"Capture bounded tracked status failure during {line}-low.", group=group, expected_any=("Status:",), validators=("fault_bus_line",), timeout_s=30.0, requires_opt_in="--include-stuck-line"),
                    CommandSpec("drv", f"Capture tracked health while the {line}-low jig remains applied.", group=group, expected_any=("Driver Health",), validators=("health_faulted_since_pre",), requires_opt_in="--include-stuck-line"),
                    CommandSpec("libreset", f"Capture bounded library reset behavior during {line}-low.", group=group, expected_any=("Status:",), validators=("fault_bus_line",), timeout_s=30.0, requires_opt_in="--include-stuck-line"),
                    CommandSpec(f"operator: release {line}-low jig", f"Release the {line}-low jig and confirm pull-ups/level shifter are not back-powering the sensor.", group=group, send=False, operator_required=True, requires_opt_in="--include-stuck-line"),
                    CommandSpec("levels", f"Confirm both lines recover after {line}-low.", group=group, expected_any=("SCL:", "SDA:"), validators=("levels_idle",), requires_opt_in="--include-stuck-line"),
                    CommandSpec("recover", f"Recover after the {line}-low fault.", group=group, expected_any=("Status:",), validators=("status_ok",), timeout_s=90.0, requires_opt_in="--include-stuck-line"),
                    CommandSpec("drv", f"Capture health after {line}-low recovery.", group=group, expected_any=("Driver Health",), validators=("health_ready",), requires_opt_in="--include-stuck-line"),
                ]
            )
    if args.include_power_cycle:
        specs.extend(
            [
                CommandSpec("dirty", "Require resolved state before sensor power-cycle.", group="fault-power-cycle", expected_any=("mutation.target",), validators=("dirty_clean", "dirty_state"), requires_opt_in="--include-power-cycle"),
                CommandSpec("levels", "Capture line levels before sensor power-cycle.", group="fault-power-cycle", expected_any=("SCL:", "SDA:"), validators=("levels_idle",), requires_opt_in="--include-power-cycle"),
                CommandSpec("operator: power-cycle sensor only", "Keep the controller and serial session powered; cycle only the named sensor rail using the approved procedure and prevent line back-powering.", group="fault-power-cycle", send=False, operator_required=True, requires_opt_in="--include-power-cycle"),
                CommandSpec("levels", "Capture line levels after sensor power-cycle.", group="fault-power-cycle", expected_any=("SCL:", "SDA:"), validators=("levels_idle",), requires_opt_in="--include-power-cycle"),
                CommandSpec("recover", "Explicitly recover identity/capabilities after sensor power-cycle.", group="fault-power-cycle", expected_any=("Status:",), validators=("status_ok",), timeout_s=90.0, requires_opt_in="--include-power-cycle"),
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
    if args.include_address_change:
        specs.extend(address_change_specs(args))
    if args.include_auto_adjust:
        specs.extend(auto_adjust_specs(args))
    specs.extend(operator_fault_specs(args))
    return specs


def exact_runtime_confirmation(enabled: bool, dry_run: bool, warning: str, text_value: str) -> None:
    if not enabled or dry_run:
        return
    print()
    print(warning)
    answer = input(f"Type '{text_value}' to continue: ").strip()
    if answer != text_value:
        print("Required runtime confirmation was not provided.", file=sys.stderr)
        raise SystemExit(2)


def confirm_hazardous_runtime(args: argparse.Namespace) -> None:
    exact_runtime_confirmation(
        args.include_persistent_writes,
        args.dry_run,
        "Persistent writes use a recorded 256-byte baseline and restore only typed settings after verified-clean evidence.",
        PERSISTENT_RUNTIME_CONFIRM_TEXT,
    )
    exact_runtime_confirmation(
        args.include_calibration_writes,
        args.dry_run,
        "Calibration writes require approved reference conditions even though the captured offset/gain baseline is restored.",
        CALIBRATION_RUNTIME_CONFIRM_TEXT,
    )
    exact_runtime_confirmation(
        args.include_address_change,
        args.dry_run,
        "Address change keeps the controller powered, never scans, and requires a second authorization before restoring the recorded address.",
        ADDRESS_RUNTIME_CONFIRM_TEXT,
    )
    exact_runtime_confirmation(
        args.include_auto_adjust,
        args.dry_run,
        "Auto-adjust is one-shot, non-cancellable, and non-restorable. The runner will never retry it.",
        AUTO_ADJUST_RUNTIME_CONFIRM_TEXT,
    )
    exact_runtime_confirmation(
        args.include_stuck_line,
        args.dry_run,
        "Stuck-line HIL requires a reviewed open-drain/current-limited jig and independent waveform evidence.",
        STUCK_LINE_RUNTIME_CONFIRM_TEXT,
    )
    exact_runtime_confirmation(
        args.include_power_cycle,
        args.dry_run,
        "Power-cycle HIL keeps the controller powered and cycles only the named sensor rail using the recorded procedure.",
        POWER_CYCLE_RUNTIME_CONFIRM_TEXT,
    )


def resolve_dynamic_command(spec: CommandSpec, state: dict[str, Any]) -> tuple[str | None, str | None]:
    if spec.dynamic == "interval_test":
        interval = state.get("baseline_measurement_interval_ds")
        if isinstance(interval, int):
            alternate = interval + 1 if interval < 36000 else interval - 1
            if not 150 <= alternate <= 36000:
                return None, "cannot derive a safe alternate interval from the recorded baseline"
            return f"interval {alternate}", None
        return None, "cannot derive interval test value because the baseline was not parsed"
    baseline_dynamic = {
        "interval_baseline": ("interval", "baseline_measurement_interval_ds"),
        "factor_baseline": ("factor", "baseline_co2_interval_factor"),
        "mode_baseline": ("mode", "baseline_operating_mode"),
        "part_name_hex_baseline": ("partnamehex", "baseline_part_name_hex"),
        "offset_baseline": ("offset", "baseline_co2_offset_ppm"),
        "gain_baseline": ("gain", "baseline_co2_gain"),
        "address_baseline_write": ("addr", "baseline_device_address"),
        "address_baseline_rebegin": ("addr rebegin", "baseline_device_address"),
    }
    if spec.dynamic in baseline_dynamic:
        command, key = baseline_dynamic[spec.dynamic]
        value = state.get(key)
        if value is None:
            return None, f"cannot resolve {command} because {key} was not captured"
        return f"{command} {value}", None
    if spec.dynamic == "address_candidate_diagnostic_passed":
        if state.get("address_candidate_diagnostic_passed") is not True:
            return None, "authorized address-candidate diagnostic did not pass"
        if state.get("mutation_unresolved") is not True:
            return None, "address candidate is not retained as unresolved"
        if state.get("mutation_target") != "BUS_ADDRESS":
            return None, "retained mutation is not BUS_ADDRESS"
        if state.get("mutation_attempted_value") != state.get("candidate_address"):
            return None, "retained address candidate does not match the authorized candidate"
        return spec.command, None
    if spec.dynamic == "address_candidate_activation_confirmed":
        if state.get("address_candidate_activation_confirmed") is not True:
            return None, "address-candidate activation was not explicitly confirmed"
        return spec.command, None
    if spec.dynamic == "address_candidate_verified":
        if state.get("address_candidate_verified") is not True:
            return None, "candidate address was not reconciled and read back successfully"
        return spec.command, None
    if spec.dynamic == "address_restore_diagnostic_passed":
        if state.get("address_restore_diagnostic_passed") is not True:
            return None, "recorded-address restoration diagnostic did not pass"
        if state.get("mutation_unresolved") is not True:
            return None, "recorded-address restoration is not retained as unresolved"
        if state.get("mutation_attempted_value") != state.get("baseline_device_address"):
            return None, "retained restoration candidate does not match the recorded baseline"
        return spec.command, None
    if spec.dynamic == "address_restore_activation_confirmed":
        if state.get("address_restore_activation_confirmed") is not True:
            return None, "recorded-address activation was not explicitly confirmed"
        value = state.get("baseline_device_address")
        if value is None:
            return None, "recorded address baseline was not captured"
        return f"addr rebegin {value}", None
    if spec.dynamic == "auto_adjust_idle":
        if state.get("auto_adjust_fresh_idle_passed") is not True:
            return None, "fresh pre-action auto-adjust idle check did not pass"
        return spec.command, None
    if spec.dynamic == "auto_adjust_authorized":
        if state.get("auto_adjust_authorized") is not True:
            return None, "controlled auto-adjust conditions were not explicitly confirmed"
        if state.get("auto_adjust_fresh_idle_passed") is not True:
            return None, "fresh pre-action auto-adjust idle check did not pass"
        return spec.command, None
    if spec.dynamic == "fault_preflight_passed":
        if state.get("fault_preflight_group") != spec.group:
            return None, "released levels and pre-fault health did not pass for this fault group"
        return spec.command, None
    return spec.command, None


def maintenance_write_block_reason(spec: CommandSpec, state: dict[str, Any]) -> str | None:
    if (
        state.get("address_failure_latched") is True
        and str(spec.group).startswith("address")
        and (
            spec.destructive
            or spec.operator_required
            or spec.command.startswith("addr rebegin ")
        )
    ):
        return "not sent: an earlier critical address-workflow step failed"
    if not spec.destructive:
        return None
    if state.get("destructive_failure_latched") is True:
        return "not sent: an earlier destructive command failed or was uncertain"
    if spec.group.startswith("maintenance") and state.get("maintenance_failure_latched") is True:
        return "not sent: a maintenance verification step failed; automatic restoration is unsafe"
    if state.get("baseline_complete") is not True:
        return "not sent: the complete required baseline preflight did not pass"
    if state.get("baseline_custom_memory_complete") is not True:
        return "not sent: complete custom-memory baseline was not checkpointed"
    if not is_complete_custom_memory_image(state.get("baseline_custom_memory")):
        return "not sent: immutable 256-byte custom-memory baseline is missing or invalid"
    captured_utc = state.get("baseline_custom_memory_captured_utc")
    if not isinstance(captured_utc, str) or not captured_utc.strip():
        return "not sent: immutable custom-memory capture time was not recorded"
    if state.get("baseline_device_address") != state.get("configured_device_address"):
        return "not sent: persistent bus-address baseline does not match configured address"
    if state.get("baseline_auto_adjust_preflight_complete") is not True:
        return "not sent: capability-aware auto-adjust idle preflight did not pass"
    if (
        spec.group.startswith("auto-adjust")
        and state.get("auto_adjust_fresh_idle_passed") is not True
    ):
        return "not sent: fresh pre-action auto-adjust idle check did not pass"
    target_baselines = {
        "maintenance-interval": "baseline_measurement_interval_ds",
        "maintenance-factor": "baseline_co2_interval_factor",
        "maintenance-mode": "baseline_operating_mode",
        "maintenance-partnamehex": "baseline_part_name_hex",
        "maintenance-calibration-offset": "baseline_co2_offset_ppm",
        "maintenance-calibration-gain": "baseline_co2_gain",
        "address-candidate": "baseline_device_address",
        "address-restore": "baseline_device_address",
        "auto-adjust": "baseline_auto_adjust_running",
    }
    baseline_key = next(
        (key for prefix, key in target_baselines.items() if spec.group.startswith(prefix)),
        None,
    )
    if baseline_key is not None and state.get(baseline_key) is None:
        return f"not sent: target baseline {baseline_key} was not captured"
    if spec.group.startswith("maintenance-mode"):
        baseline_mode = state.get("baseline_operating_mode")
        if not isinstance(baseline_mode, int) or not 0 <= baseline_mode <= 3:
            return "not sent: recorded operating-mode baseline is outside the typed restore range 0..3"
    if spec.group.startswith("maintenance-interval"):
        baseline_interval = state.get("baseline_measurement_interval_ds")
        if not isinstance(baseline_interval, int) or not 150 <= baseline_interval <= 36000:
            return "not sent: recorded interval baseline is outside the typed restore range 150..36000"
    if spec.group.startswith("maintenance-factor"):
        baseline_factor = state.get("baseline_co2_interval_factor")
        if (
            not isinstance(baseline_factor, int)
            or not -128 <= baseline_factor <= 127
            or baseline_factor == 0
        ):
            return "not sent: recorded factor baseline is outside the nonzero typed restore range"
    if spec.group.startswith("address"):
        baseline_address = state.get("baseline_device_address")
        if not isinstance(baseline_address, int) or not 0 <= baseline_address <= 7:
            return "not sent: recorded address baseline is outside the typed restore range 0..7"
    restore_dynamics = {
        "interval_baseline",
        "factor_baseline",
        "mode_baseline",
        "part_name_hex_baseline",
        "offset_baseline",
        "gain_baseline",
    }
    if spec.group.startswith("maintenance") and spec.dynamic not in restore_dynamics:
        command_name, _, raw_value = spec.command.partition(" ")
        no_op_fields = {
            "interval": "baseline_measurement_interval_ds",
            "factor": "baseline_co2_interval_factor",
            "mode": "baseline_operating_mode",
            "partnamehex": "baseline_part_name_hex",
            "offset": "baseline_co2_offset_ppm",
            "gain": "baseline_co2_gain",
        }
        baseline_field = no_op_fields.get(command_name)
        if baseline_field is not None and raw_value and "<" not in raw_value:
            baseline_value = state.get(baseline_field)
            requested_value: Any = raw_value.upper() if command_name == "partnamehex" else int(raw_value)
            if requested_value == baseline_value:
                return "not sent: selected test value equals the recorded baseline"
    if (
        spec.group.startswith("address-candidate")
        and state.get("baseline_device_address") == state.get("candidate_address")
    ):
        return "not sent: candidate address equals the recorded baseline"
    mutation_epoch = state.get("mutation_epoch", 0)
    if state.get("dirty_observation_epoch") != mutation_epoch:
        return "not sent: no fresh successful dirty diagnostic exists after the latest mutation"
    if state.get("persistent_config_dirty") is not False:
        return "not sent: persistent state was not parsed as clean before write"
    if state.get("resync_needed") is not False:
        return "not sent: resync-needed state was not parsed as clean before write"
    if state.get("mutation_unresolved") is not False:
        return "not sent: mutation state was not explicitly parsed as resolved"
    if spec.group.startswith("address-restore") and state.get("address_restore_authorized") is not True:
        return "not sent: recorded-address restoration was not independently authorized"
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
        "capture": list(spec.capture),
        "operator_confirm_text": spec.operator_confirm_text,
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
    confirmation = spec.operator_confirm_text or "done"
    print(
        f"Type '{confirmation}' after performing the step, "
        "'skip' to stop without performing it, or 'abort' to stop."
    )
    try:
        answer = input("operator> ").strip()
    except EOFError:
        answer = ""
    if answer.lower() == "abort":
        raise KeyboardInterrupt("operator aborted HIL run")
    if answer.lower() == "skip":
        raise KeyboardInterrupt(f"operator skipped required step: {spec.command}")
    if answer != confirmation:
        raise KeyboardInterrupt(f"operator did not exactly confirm required step: {spec.command}")
    return result_row(
        spec,
        spec.command,
        RESULT_OPERATOR,
        "operator confirmed step; external evidence remains review-required",
        0.0,
        "",
        "operator",
        {"operator_confirmed": True},
    )


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
    group = str(row.get("group", ""))
    planned_command = str(row.get("planned_command", ""))
    result = row.get("result")
    for key in (
        "firmware_build",
        "library_version",
        "library_full",
        "library_build",
        "library_commit",
        "library_git_status",
        "device_firmware_version",
        "e2_spec_version",
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
        "co2_interval_factor",
        "co2_filter",
        "operating_mode",
        "part_name_hex",
        "auto_adjust_running",
        "operating_functions",
        "operating_mode_support",
        "special_features",
        "capabilities",
        "mutation_unresolved",
        "mutation_target",
        "mutation_effect",
        "mutation_attempted_value",
    ):
        if key in parsed:
            state[key] = parsed[key]
    if "selftest" in parsed:
        state["last_selftest"] = parsed["selftest"]
    if "stress" in parsed:
        state["last_stress"] = parsed["stress"]
    if row.get("result") == RESULT_PASS:
        for capture in row.get("capture") or []:
            if capture == "final_custom_memory":
                if "custom_memory" in parsed:
                    state["final_custom_memory"] = parsed["custom_memory"]
                    state["final_custom_memory_complete"] = parsed.get("custom_memory_complete")
                continue
            if capture in parsed:
                captured_value = copy.deepcopy(parsed[capture])
                state.setdefault(f"baseline_{capture}", captured_value)
                if capture == "custom_memory":
                    state.setdefault(
                        "baseline_custom_memory_complete",
                        parsed.get("custom_memory_complete") is True,
                    )
                    state.setdefault(
                        "baseline_custom_memory_captured_utc",
                        iso_timestamp(),
                    )
        if (
            planned_command == "dirty"
            and "persistent_config_dirty" in parsed
            and "resync_needed" in parsed
            and "mutation_unresolved" in parsed
        ):
            state["dirty_observation_epoch"] = state.get("mutation_epoch", 0)
    elif row.get("planned_command") in {"dirty", "resync"}:
        state["dirty_observation_epoch"] = -1
    if (
        row.get("planned_command") == "operator: authorize address restoration"
        and parsed.get("operator_confirmed") is True
    ):
        state["address_restore_authorized"] = True
    if (
        row.get("planned_command") == "operator: activate address candidate"
        and parsed.get("operator_confirmed") is True
    ):
        state["address_candidate_activation_confirmed"] = True
    if (
        row.get("planned_command") == "operator: activate restored address"
        and parsed.get("operator_confirmed") is True
    ):
        state["address_restore_activation_confirmed"] = True
    if (
        row.get("planned_command") == "operator: confirm controlled auto-adjust conditions"
        and parsed.get("operator_confirmed") is True
    ):
        state["auto_adjust_authorized"] = True
    if (
        group == "auto-adjust"
        and planned_command == "autoadj"
        and "before the one-shot" in str(row.get("description", "")).lower()
    ):
        state["auto_adjust_fresh_idle_passed"] = (
            result == RESULT_PASS
            and parsed.get("auto_adjust_running") is False
        )
    if group.endswith("-baseline"):
        typed_commands = {
            "partnamehex",
            "addr",
            "interval",
            "factor",
            "filter",
            "mode",
            "offset",
            "gain",
            "calpoints",
            "autoadj",
        }
        if planned_command in typed_commands and result == RESULT_PASS:
            typed_results = state.setdefault("baseline_typed_results", {})
            if planned_command not in typed_results:
                typed_results[planned_command] = {
                    "status": copy.deepcopy(parsed.get("status") or {}),
                    "supported": (parsed.get("status") or {}).get("name") == "OK",
                    "value": copy.deepcopy({
                        key: value
                        for key, value in parsed.items()
                        if key not in {"command", "statuses", "status"}
                    }),
                }
        if result != RESULT_PASS:
            state["baseline_failure_latched"] = True
        if planned_command == "autoadj":
            state["baseline_auto_adjust_preflight_complete"] = result == RESULT_PASS
            state["baseline_complete"] = (
                result == RESULT_PASS
                and state.get("baseline_failure_latched") is not True
            )
    if (
        group in {"fault-sda-low", "fault-scl-low"}
        and planned_command == "levels"
        and "before the" in str(row.get("description", "")).lower()
    ):
        if result == RESULT_PASS:
            state["fault_released_levels_group"] = group
        else:
            state.pop("fault_released_levels_group", None)
            state.pop("fault_preflight_group", None)
    if (
        group in {"fault-sda-low", "fault-scl-low"}
        and planned_command == "drv"
        and "before the" in str(row.get("description", "")).lower()
    ):
        if (
            result == RESULT_PASS
            and state.get("fault_released_levels_group") == group
            and isinstance(parsed.get("total_failures"), int)
        ):
            state["fault_pre_total_failures"] = parsed["total_failures"]
            state["fault_pre_group"] = group
            state["fault_preflight_group"] = group
        else:
            state.pop("fault_pre_total_failures", None)
            state.pop("fault_pre_group", None)
            state.pop("fault_preflight_group", None)
    if group == "address-candidate" and planned_command == "dirty" and result == RESULT_PASS:
        state["address_candidate_diagnostic_passed"] = True
    if (
        group == "address-candidate"
        and planned_command.startswith("addr rebegin ")
        and result == RESULT_PASS
    ):
        state["address_candidate_rebegin_passed"] = True
    if (
        group == "address-candidate"
        and planned_command == "addr"
        and result == RESULT_PASS
        and state.get("address_candidate_rebegin_passed") is True
    ):
        state["address_candidate_verified"] = True
    if group == "address-restore" and planned_command == "dirty" and result == RESULT_PASS:
        state["address_restore_diagnostic_passed"] = True
    if (
        group == "address-restore"
        and planned_command.startswith("addr rebegin ")
        and result == RESULT_PASS
    ):
        state["address_restore_rebegin_passed"] = True
    if (
        state.get("address_change_started") is True
        and group.startswith("address")
        and row.get("operator_required") is not True
        and result != RESULT_PASS
    ):
        state["address_failure_latched"] = True
    if (
        str(row.get("group", "")).startswith("maintenance")
        and state.get("maintenance_destructive_started") is True
        and row.get("result") != RESULT_PASS
    ):
        state["maintenance_failure_latched"] = True


def record_persistent_write_expectation(row: dict[str, Any], state: dict[str, Any]) -> None:
    if not row.get("destructive") or row.get("wait_reason") == "not-sent":
        return
    state["mutation_epoch"] = int(state.get("mutation_epoch", 0)) + 1
    state["dirty_observation_epoch"] = -1
    state["persistent_config_dirty"] = None
    state["resync_needed"] = None
    state["mutation_unresolved"] = None
    if str(row.get("group", "")).startswith("maintenance"):
        state["maintenance_destructive_started"] = True
    if str(row.get("group", "")).startswith("address"):
        state["address_change_started"] = True
    if row.get("result") != RESULT_PASS:
        state["destructive_failure_latched"] = True
        return

    def expect(target: str, count: int, first: int, last: int) -> None:
        state["expected_mutation_target"] = target
        state["expected_mutation_count"] = count
        state["expected_mutation_first_address"] = first
        state["expected_mutation_last_address"] = last

    command = str(row.get("command", ""))
    match = re.fullmatch(r"interval\s+(\d+)", command)
    if match:
        state["expected_measurement_interval_ds"] = int(match.group(1))
        expect("GLOBAL_INTERVAL", 2, 0xC6, 0xC7)
    elif match := re.fullmatch(r"factor\s+(-?\d+)", command):
        state["expected_co2_interval_factor"] = int(match.group(1))
        expect("CO2_INTERVAL_FACTOR", 1, 0xCB, 0xCB)
    elif match := re.fullmatch(r"mode\s+(\d+)", command):
        state["expected_operating_mode"] = int(match.group(1))
        expect("OPERATING_MODE", 1, 0xD8, 0xD8)
    elif match := re.fullmatch(r"partnamehex\s+([0-9A-Fa-f]{32})", command):
        state["expected_part_name_hex"] = match.group(1).upper()
        expect("PART_NAME", 16, 0xB0, 0xBF)
    elif match := re.fullmatch(r"offset\s+(-?\d+)", command):
        state["expected_co2_offset_ppm"] = int(match.group(1))
        expect("CO2_OFFSET", 2, 0x58, 0x59)
    elif match := re.fullmatch(r"gain\s+(\d+)", command):
        state["expected_co2_gain"] = int(match.group(1))
        expect("CO2_GAIN", 2, 0x5A, 0x5B)
    elif match := re.fullmatch(r"addr\s+(\d+)", command):
        state["expected_device_address"] = int(match.group(1))
        expect("BUS_ADDRESS", 1, 0xC0, 0xC0)
    elif command == "autoadj start":
        expect("AUTO_ADJUST", 1, 0xD9, 0xD9)


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


def atomic_write_text(path: Path, text_value: str) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text_value, encoding="utf-8")
    temporary.replace(path)


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
        "sensor_id": args.sensor_id,
        "fixture_id": args.fixture_id,
        "power_procedure": args.power_procedure,
        "electrical_authority": args.electrical_authority,
        "configured_device_address": args.device_address,
        "git_branch": git_branch,
        "git_commit": git_commit,
        "git_worktree": "unknown" if git_status == "unknown" else ("clean" if not git_status else "dirty"),
    }


def write_transcript(path: Path, meta: dict[str, Any], initial_output: str, results: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write("EE871-E2 serial HIL transcript\n")
        for key in (
            "timestamp_utc", "port", "baud", "dry_run", "board", "target_name",
            "operator", "sensor_id", "fixture_id", "power_procedure",
            "electrical_authority", "configured_device_address", "git_branch",
            "git_commit", "git_worktree",
        ):
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
        for key in ("timestamp_utc", "port", "baud", "dry_run", "board", "target_name", "operator", "sensor_id", "fixture_id", "power_procedure", "electrical_authority", "configured_device_address", "git_branch", "git_commit", "git_worktree"):
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
        fh.write("- `checkpoint.json` (updated before destructive transmissions and after every completed step)\n")
        if isinstance(state.get("baseline_custom_memory"), list):
            fh.write("- `custom_memory_baseline.json`\n")
            fh.write("- `custom_memory_baseline.hex`\n")


def write_checkpoint(
    log_dir: Path,
    meta: dict[str, Any],
    initial_output: str,
    results: list[dict[str, Any]],
    state: dict[str, Any],
) -> None:
    checkpoint = {
        "metadata": meta,
        "updated_utc": iso_timestamp(),
        "baseline_custom_memory_captured_utc": state.get(
            "baseline_custom_memory_captured_utc"
        ),
        "in_flight_destructive": state.get("in_flight_destructive"),
        "parsed_state": state,
        "commands": results,
    }
    atomic_write_text(
        log_dir / "checkpoint.json",
        json.dumps(checkpoint, indent=2),
    )
    baseline = state.get("baseline_custom_memory")
    if not isinstance(baseline, list) or len(baseline) != CUSTOM_MEMORY_SIZE:
        return
    baseline_payload = {
        "metadata": meta,
        "captured_utc": state.get("baseline_custom_memory_captured_utc"),
        "updated_utc": checkpoint["updated_utc"],
        "size": CUSTOM_MEMORY_SIZE,
        "bytes": list(baseline),
        "typed_baselines": state.get("baseline_typed_results", {}),
        "addresses": {
            f"0x{address:02X}": value
            for address, value in enumerate(baseline)
        },
        "warning": "Forensic baseline only. Never replay this image; restore only typed allowlisted settings.",
    }
    atomic_write_text(
        log_dir / "custom_memory_baseline.json",
        json.dumps(baseline_payload, indent=2),
    )
    lines = []
    for start in range(0, CUSTOM_MEMORY_SIZE, 16):
        row = " ".join(f"{value:02X}" for value in baseline[start:start + 16])
        lines.append(f"0x{start:02X}: {row}")
    atomic_write_text(
        log_dir / "custom_memory_baseline.hex",
        "\n".join(lines) + "\n",
    )


def journal_destructive_start(
    state: dict[str, Any],
    spec: CommandSpec,
    resolved_command: str,
) -> None:
    if not spec.destructive:
        return
    if state.get("in_flight_destructive") is not None:
        raise RuntimeError("a destructive command is already journaled as in flight")
    state["in_flight_destructive"] = {
        "started_utc": iso_timestamp(),
        "command": resolved_command,
        "planned_command": spec.command,
        "group": spec.group,
        "description": spec.description,
        "warning": (
            "Outcome unknown until a completed result is checkpointed. "
            "Do not issue another write or assume restoration."
        ),
    }


def journal_destructive_completion(
    state: dict[str, Any],
    row: dict[str, Any],
) -> None:
    in_flight = state.get("in_flight_destructive")
    if not isinstance(in_flight, dict):
        return
    state["last_destructive_completion"] = {
        **in_flight,
        "completed_utc": iso_timestamp(),
        "result": row.get("result"),
        "reason": row.get("reason"),
    }
    state["in_flight_destructive"] = None


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


def meaningful_metadata(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and value.strip().lower() != "unspecified"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=f"%(prog)s {SCRIPT_VERSION}")
    parser.add_argument("--port", help="Serial port, for example COM5 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="Startup/initial serial drain timeout in seconds.")
    parser.add_argument("--command-timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT_S)
    parser.add_argument("--idle", type=float, default=DEFAULT_IDLE_S, help="Idle gap after completion token before a command is considered complete.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--address", "--device-address", dest="device_address", default="0", help="Expected E2 device address metadata. This does not retarget firmware.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-extended", "--extended-safe", "--complete-safe", dest="include_extended", action="store_true")
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
    parser.add_argument("--maintenance-interval", type=int, help="Measurement interval test value; defaults to a one-step safe alternate. Restore is sent only after verified-clean evidence.")
    parser.add_argument("--write-interval-factor", type=int, help="Optional nonzero factor test value (-128..-1 or 1..127); restore requires verified-clean evidence.")
    parser.add_argument("--write-operating-mode", type=int, help="Optional mode test value; restore requires verified-clean evidence.")
    parser.add_argument("--write-part-name-hex", help="Optional exact 32-hex-digit part-name test value; exact restore requires verified-clean evidence.")
    parser.add_argument("--include-calibration-writes", action="store_true")
    parser.add_argument(
        "--confirm-calibration-writes",
        nargs="?",
        const=CALIBRATION_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument("--write-co2-offset", type=int, help="Calibration offset test value; restore requires the calibration opt-in and verified-clean evidence.")
    parser.add_argument("--write-co2-gain", type=int, help="Calibration gain test value; restore requires the calibration opt-in and verified-clean evidence.")
    parser.add_argument("--include-address-change", action="store_true")
    parser.add_argument("--candidate-address", type=int)
    parser.add_argument(
        "--confirm-address-change",
        nargs="?",
        const=ADDRESS_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument(
        "--confirm-address-restore",
        nargs="?",
        const=ADDRESS_RESTORE_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument("--include-auto-adjust", action="store_true")
    parser.add_argument(
        "--confirm-auto-adjust",
        nargs="?",
        const=AUTO_ADJUST_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument("--include-unplug-replug", action="store_true")
    parser.add_argument("--include-stuck-line", action="store_true")
    parser.add_argument("--include-power-cycle", action="store_true")
    parser.add_argument(
        "--confirm-stuck-line",
        nargs="?",
        const=STUCK_LINE_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument(
        "--confirm-power-cycle",
        nargs="?",
        const=POWER_CYCLE_CONFIRM_TEXT,
        default="",
    )
    parser.add_argument("--board", default="unspecified")
    parser.add_argument("--target-name", default="unspecified")
    parser.add_argument("--operator", default="unspecified")
    parser.add_argument("--sensor-id", default="unspecified")
    parser.add_argument("--fixture-id", default="unspecified")
    parser.add_argument("--power-procedure", default="unspecified")
    parser.add_argument("--electrical-authority", default="unspecified")
    args = parser.parse_args(argv)

    if not args.dry_run and not args.port:
        parser.error("--port is required unless --dry-run is used")
    if args.include_persistent_writes and args.confirm_persistent_writes != PERSISTENT_CONFIRM_TEXT:
        parser.error(
            "--include-persistent-writes requires "
            f"--confirm-persistent-writes {PERSISTENT_CONFIRM_TEXT!r}"
        )
    persistent_values = (
        args.maintenance_interval,
        args.write_interval_factor,
        args.write_operating_mode,
        args.write_part_name_hex,
    )
    if any(value is not None for value in persistent_values) and not (
        args.include_persistent_writes and args.confirm_persistent_writes
    ):
        parser.error("persistent write values require --include-persistent-writes --confirm-persistent-writes")
    if args.maintenance_interval is not None and not (150 <= args.maintenance_interval <= 36000):
        parser.error("--maintenance-interval must be 150..36000 deciseconds")
    if args.write_co2_offset is not None and not (-32768 <= args.write_co2_offset <= 32767):
        parser.error("--write-co2-offset must be -32768..32767")
    if args.write_co2_gain is not None and not (0 <= args.write_co2_gain <= 65535):
        parser.error("--write-co2-gain must be 0..65535")
    if (
        args.write_interval_factor is not None
        and (
            not -128 <= args.write_interval_factor <= 127
            or args.write_interval_factor == 0
        )
    ):
        parser.error("--write-interval-factor must be -128..-1 or 1..127")
    if args.write_operating_mode is not None and not (0 <= args.write_operating_mode <= 3):
        parser.error("--write-operating-mode must be 0..3")
    if args.write_part_name_hex is not None:
        if re.fullmatch(r"[0-9A-Fa-f]{32}", args.write_part_name_hex) is None:
            parser.error("--write-part-name-hex must contain exactly 32 hex digits")
        args.write_part_name_hex = args.write_part_name_hex.upper()
    if args.include_calibration_writes:
        if not args.include_persistent_writes:
            parser.error("--include-calibration-writes requires --include-persistent-writes")
        if args.confirm_calibration_writes != CALIBRATION_CONFIRM_TEXT:
            parser.error(
                "--include-calibration-writes requires "
                f"--confirm-calibration-writes {CALIBRATION_CONFIRM_TEXT!r}"
            )
        if args.write_co2_offset is None and args.write_co2_gain is None:
            parser.error("--include-calibration-writes requires an explicit offset and/or gain test value")
        if any(value is not None for value in persistent_values):
            parser.error(
                "run calibration writes separately from interval, factor, "
                "mode, and part-name configuration tests"
            )
    elif args.write_co2_offset is not None or args.write_co2_gain is not None:
        parser.error(
            "calibration values require --include-calibration-writes "
            "--confirm-calibration-writes in addition to persistent-write confirmation"
        )
    if args.include_address_change:
        if args.candidate_address is None or not (0 <= args.candidate_address <= 7):
            parser.error("--include-address-change requires --candidate-address 0..7")
        if args.confirm_address_change != ADDRESS_CONFIRM_TEXT:
            parser.error(
                "--include-address-change requires "
                f"--confirm-address-change {ADDRESS_CONFIRM_TEXT!r}"
            )
        if args.confirm_address_restore != ADDRESS_RESTORE_CONFIRM_TEXT:
            parser.error(
                "--include-address-change requires "
                f"--confirm-address-restore {ADDRESS_RESTORE_CONFIRM_TEXT!r}"
            )
    elif args.candidate_address is not None:
        parser.error("--candidate-address requires --include-address-change")
    if args.include_auto_adjust and args.confirm_auto_adjust != AUTO_ADJUST_CONFIRM_TEXT:
        parser.error(
            "--include-auto-adjust requires "
            f"--confirm-auto-adjust {AUTO_ADJUST_CONFIRM_TEXT!r}"
        )
    if args.include_stuck_line and args.confirm_stuck_line != STUCK_LINE_CONFIRM_TEXT:
        parser.error(
            "--include-stuck-line requires "
            f"--confirm-stuck-line {STUCK_LINE_CONFIRM_TEXT!r}"
        )
    if args.include_power_cycle and args.confirm_power_cycle != POWER_CYCLE_CONFIRM_TEXT:
        parser.error(
            "--include-power-cycle requires "
            f"--confirm-power-cycle {POWER_CYCLE_CONFIRM_TEXT!r}"
        )
    try:
        args.device_address = int(str(args.device_address), 0)
    except ValueError:
        parser.error("--address/--device-address must be an integer")
    if not (0 <= args.device_address <= 7):
        parser.error("--address/--device-address must be 0..7")
    hazard_flags = (
        args.include_persistent_writes,
        args.include_address_change,
        args.include_auto_adjust,
        args.include_unplug_replug,
        args.include_stuck_line,
        args.include_power_cycle,
    )
    if sum(bool(value) for value in hazard_flags) > 1:
        parser.error("run persistent, address, auto-adjust, unplug, stuck-line, and power-cycle plans separately")
    if not args.dry_run and any(hazard_flags):
        required_metadata = {
            "--board": args.board,
            "--target-name": args.target_name,
            "--operator": args.operator,
            "--sensor-id": args.sensor_id,
            "--fixture-id": args.fixture_id,
            "--electrical-authority": args.electrical_authority,
        }
        if args.include_address_change or args.include_power_cycle:
            required_metadata["--power-procedure"] = args.power_procedure
        for attribute in (
            "board",
            "target_name",
            "operator",
            "sensor_id",
            "fixture_id",
            "electrical_authority",
            "power_procedure",
        ):
            value = getattr(args, attribute)
            if isinstance(value, str):
                setattr(args, attribute, value.strip())
        missing = [
            flag
            for flag, value in required_metadata.items()
            if not meaningful_metadata(value)
        ]
        if missing:
            parser.error(f"hazardous live plans require structured metadata: {', '.join(missing)}")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    confirm_hazardous_runtime(args)
    plan = build_plan(args)
    git_status = git_value("status", "--short", empty_value="")
    git_branch = git_value("branch", "--show-current")
    git_commit = git_value("rev-parse", "--short=12", "HEAD")
    log_dir = make_log_dir(args.output_dir)
    meta = metadata(args, log_dir, git_status, git_branch, git_commit)
    results: list[dict[str, Any]] = []
    state: dict[str, Any] = {
        "mutation_epoch": 0,
        "dirty_observation_epoch": -1,
        "candidate_address": args.candidate_address,
        "configured_device_address": args.device_address,
    }
    initial_output = ""

    try:
        if args.dry_run:
            for spec in plan:
                row = dry_run_row(spec, state)
                results.append(row)
                update_state(state, row)
                write_checkpoint(log_dir, meta, initial_output, results, state)
        else:
            ser = open_serial(args)
            try:
                initial_output, _, _ = read_until_ready(ser, args.timeout, args.idle, None, require_prompt=True)
                for spec in plan:
                    reason = maintenance_write_block_reason(spec, state)
                    command: str | None = spec.command
                    if not reason:
                        command, reason = resolve_dynamic_command(spec, state)
                    if reason:
                        row = result_row(spec, spec.command, RESULT_SKIP, reason, 0.0, "", "not-sent", {})
                    elif spec.operator_required or not spec.send:
                        row = run_operator_step(spec)
                    else:
                        if spec.destructive:
                            journal_destructive_start(state, spec, command or spec.command)
                            write_checkpoint(log_dir, meta, initial_output, results, state)
                        row = run_serial_command(ser, spec, command or spec.command, args, state)
                    results.append(row)
                    update_state(state, row)
                    record_persistent_write_expectation(row, state)
                    if spec.destructive and state.get("in_flight_destructive") is not None:
                        journal_destructive_completion(state, row)
                    write_checkpoint(log_dir, meta, initial_output, results, state)
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
        write_checkpoint(log_dir, meta, initial_output, results, state)
    except Exception as exc:
        error_name = type(exc).__name__
        error_message = str(exc) or "runner exception"
        state["runner_exception"] = {
            "type": error_name,
            "message": error_message,
            "recorded_utc": iso_timestamp(),
            "in_flight_destructive_preserved": (
                state.get("in_flight_destructive") is not None
            ),
        }
        results.append(
            result_row(
                CommandSpec(
                    "runner error",
                    "Runner stopped after an unexpected serial or processing error.",
                    group="runner",
                ),
                "runner error",
                RESULT_FAIL,
                f"{error_name}: {error_message}",
                0.0,
                "",
                "exception",
                {},
            )
        )
        write_checkpoint(log_dir, meta, initial_output, results, state)

    final = verdict(results, args.dry_run)
    aggregate_counts = counts(results)
    write_transcript(log_dir / "serial_transcript.txt", meta, initial_output, results)
    write_summary_json(log_dir / "summary.json", meta, results, final, state, initial_output, aggregate_counts)
    write_summary_md(log_dir / "summary.md", meta, results, final, state, aggregate_counts)
    write_checkpoint(log_dir, meta, initial_output, results, state)

    print(f"Output directory: {log_dir}")
    print(f"Final verdict: {final}")
    return exit_code_for_verdict(final)


if __name__ == "__main__":
    raise SystemExit(main())
