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
    "sampleavg",
    "samplefast",
    "verbose",
    "stress",
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
    "CO2_SENSOR_ERROR",
    "readCo2AverageSample",
    "readCo2FastSample",
    "Read raw MV3",
    "Read raw MV4",
    "Read checked MV3 then status",
    "Read checked MV4 then status",
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
    "sampleavg help entry": r'printHelpItem\(\s*"sampleavg"\s*,',
    "samplefast help entry": r'printHelpItem\(\s*"samplefast"\s*,',
    "sampleavg command dispatch": r'trimmed\s*==\s*"sampleavg"',
    "samplefast command dispatch": r'trimmed\s*==\s*"samplefast"',
    "sampleavg checked API": r"readCo2AverageSample\s*\(\s*sample\s*\)",
    "samplefast checked API": r"readCo2FastSample\s*\(\s*sample\s*\)",
    "co2avg raw API": r"readCo2Average\s*\(\s*ppm\s*\)",
    "co2fast raw API": r"readCo2Fast\s*\(\s*ppm\s*\)",
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

    for cmd in MANDATORY_COMMANDS:
        if re.search(rf"\b{re.escape(cmd)}\b", text) is None:
            fail(f"mandatory command '{cmd}' missing in {bringup_main.as_posix()}")

    for fragment in REQUIRED_FRAGMENTS:
        if fragment not in text:
            fail(f"mandatory CLI dirty diagnostic fragment '{fragment}' missing")

    for label, pattern in REQUIRED_PATTERNS.items():
        if re.search(pattern, text) is None:
            fail(f"missing {label} in {bringup_main.as_posix()}")

    if re.search(r"\bcfg\b", text) is None and re.search(r"\bsettings\b", text) is None:
        fail("either 'cfg' or 'settings' command must be present")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
