#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

REQUIRED_COMMON = [
    "BoardConfig.h",
    "BuildConfig.h",
    "Log.h",
    "TransportAdapter.h",
    "BusDiag.h",
    "CliShell.h",
    "CliStyle.h",
    "HealthView.h",
]

MANDATORY_COMMANDS = [
    "help",
    "scan",
    "probe",
    "recover",
    "drv",
    "dirty",
    "resync",
    "read",
    "verbose",
    "stress",
    "status",
    "co2fast",
    "co2avg",
    "samplefast",
    "sampleavg",
    "partnamehex",
]

REQUIRED_FRAGMENTS = [
    "persistentConfigDirty",
    "persistentConfigDirtyError",
    "persistentConfigDirtyError message",
    "resyncNeeded",
    "resyncPersistentConfig",
    "Write persistent custom register (bench only)",
    "Write persistent interval",
    "Write persistent CO2 offset",
    "Write persistent CO2 gain",
    "mutation.unresolved",
    "mutation.target",
    "mutation.effect",
    "mutation.addresses",
    "mutation.elements",
    "mutation.attemptedValue",
    "mutation.preObservedValue",
    "mutation.observedValue",
    "mutation.cause",
    "PERSISTENT_STATE_UNCERTAIN",
    "CO2_SENSOR_ERROR",
    "Value step: attempted=",
    "Value step message:",
    "Status step: attempted=",
    "Status step message:",
    "Error-code step: attempted=",
    "Error-code step message:",
    "Sensor error:",
    "may trigger next measurement",
    "addr rebegin <0-7>",
    "No retained unresolved BUS_ADDRESS candidate",
    "partnamehex <32-hex>",
    "parsePartNameHex",
]

REQUIRED_PATTERNS = {
    "dirty help entry": r'printHelpItem\(\s*"dirty"\s*,',
    "resync help entry": r'printHelpItem\(\s*"resync"\s*,',
    "dirty command dispatch": r'trimmed\s*==\s*"dirty"',
    "resync command dispatch": r'trimmed\s*==\s*"resync"',
    "dirty accessor": r"persistentConfigDirty",
    "dirty error accessor": r"persistentConfigDirtyError",
    "resync API": r"resyncPersistentConfig",
    "driver health dirty output": r"void\s+printDriverHealth\s*\([^)]*\)\s*\{[\s\S]*?printPersistentDirtyFields\s*\(\s*settings\s*\)",
    "status dirty summary": r"hasCo2Error\(\):[\s\S]*?printPersistentDirtySummaryIfDirty\s*\(",
    "resync before after output": r'trimmed\s*==\s*"resync"[\s\S]*?Before:[\s\S]*?resyncPersistentConfig\s*\(\s*\)[\s\S]*?After:',
    "dirty error code detail output": r"persistentConfigDirtyError:[\s\S]*?code=%u,\s*detail=%ld",
    "samplefast help entry": r'printHelpItem\(\s*"samplefast"\s*,',
    "sampleavg help entry": r'printHelpItem\(\s*"sampleavg"\s*,',
    "checked result formatter": r"void\s+printCheckedSample\s*\([^)]*Co2ReadResult",
    "mutation target formatter": r"mutationTargetToStr\s*\(",
    "mutation effect formatter": r"mutationEffectToStr\s*\(",
    "CO2 sensor error formatter": r"co2SensorErrorToStr\s*\(",
    "Arduino delayMs callback": r"deviceCfg\.delayMs\s*=\s*transport::delayMs\s*;",
    "Arduino yield callback": r"deviceCfg\.yield\s*=\s*transport::yieldTask\s*;",
    "address candidate rebegin": r'trimmed\.startsWith\(\s*"addr rebegin "\s*\)[\s\S]*?device\.end\(\)[\s\S]*?device\.begin\(deviceCfg\)[\s\S]*?resyncPersistentConfig\(\)',
    "strict signed parser": r"bool\s+parseIntToken\s*\(",
    "unsigned parser rejects negative tokens": r"bool\s+parseU8Token[\s\S]*?token\[0\]\s*==\s*'-'[\s\S]*?bool\s+parseU16Token[\s\S]*?token\[0\]\s*==\s*'-'",
    "selftest address capability gate": r"hasAddressConfig\(\)[\s\S]*?readBusAddress",
    "selftest interval capability gate": r"hasGlobalInterval\(\)[\s\S]*?readMeasurementInterval",
}


def fail(msg: str) -> None:
    print(f"CLI contract FAILED: {msg}")
    raise SystemExit(1)


def ensure_exists(path: pathlib.Path, label: str) -> None:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")


def ensure_missing(path: pathlib.Path, label: str) -> None:
    if path.exists():
        fail(f"forbidden {label} still present: {path.as_posix()}")


def command_branch(text: str, condition: str) -> str:
    start_match = re.search(
        rf'(?:if|else\s+if)\s*\(\s*trimmed\s*==\s*"{re.escape(condition)}"\s*\)\s*\{{',
        text,
    )
    if start_match is None:
        fail(f"command dispatch branch {condition!r} missing")
    start = start_match.end()
    end_match = re.search(r"\}\s*else\s+if\s*\(", text[start:])
    return text[start:] if end_match is None else text[start : start + end_match.start()]


def require_command_call(
    text: str,
    command: str,
    required_call: str,
    forbidden_calls: tuple[str, ...],
) -> None:
    branch = command_branch(text, command)
    if required_call not in branch:
        fail(f"{command!r} must call {required_call}")
    for forbidden in forbidden_calls:
        if forbidden in branch:
            fail(f"{command!r} must not call {forbidden}")


def main() -> int:
    common_dir = ROOT / "examples" / "common"
    bringup_main = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"

    ensure_exists(common_dir, "common example directory")
    ensure_exists(bringup_main, "bringup CLI example")

    ensure_missing(ROOT / "examples" / "00_smoke_boot", "deprecated example 00_smoke_boot")
    ensure_missing(
        ROOT / "examples" / "03_feature_walkthrough",
        "deprecated example 03_feature_walkthrough",
    )

    for name in REQUIRED_COMMON:
        ensure_exists(common_dir / name, f"common helper {name}")

    text = bringup_main.read_text(encoding="utf-8", errors="replace")
    transport_text = (common_dir / "E2Transport.h").read_text(
        encoding="utf-8", errors="replace"
    )

    for cmd in MANDATORY_COMMANDS:
        if re.search(rf"\b{re.escape(cmd)}\b", text) is None:
            fail(f"mandatory command '{cmd}' missing in {bringup_main.as_posix()}")

    for fragment in REQUIRED_FRAGMENTS:
        if fragment not in text:
            fail(f"mandatory CLI dirty diagnostic fragment '{fragment}' missing")

    for label, pattern in REQUIRED_PATTERNS.items():
        if re.search(pattern, text) is None:
            fail(f"missing {label} in {bringup_main.as_posix()}")

    require_command_call(
        text,
        "co2fast",
        "readCo2Fast(ppm)",
        ("readCo2FastSample", "readStatus(", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        text,
        "co2avg",
        "readCo2Average(ppm)",
        ("readCo2AverageSample", "readStatus(", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        text,
        "samplefast",
        "readCo2FastSample(result)",
        ("readCo2AverageSample", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        text,
        "sampleavg",
        "readCo2AverageSample(result)",
        ("readCo2FastSample", "customWrite(", "startAutoAdjust("),
    )

    for fragment in (
        "inline void delayMs(uint32_t ms, void* user)",
        "delay(ms);",
        "inline void yieldTask(void* user)",
        "::yield();",
    ):
        if fragment not in transport_text:
            fail(f"Arduino transport missing long-wait mapping: {fragment!r}")

    persistent_dispatch = text[text.find('trimmed == "partname"'):text.find('trimmed == "calpoints"')]
    if ".toInt()" in persistent_dispatch:
        fail("persistent CLI commands must not use String::toInt fallback parsing")

    if re.search(r"\bcfg\b", text) is None and re.search(r"\bsettings\b", text) is None:
        fail("either 'cfg' or 'settings' command must be present")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
