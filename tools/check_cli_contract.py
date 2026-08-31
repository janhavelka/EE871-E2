#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

REQUIRED_COMMON = [
    "BoardConfig.h",
    "BuildConfig.h",
    "E2Diagnostics.h",
    "E2Transport.h",
    "Log.h",
    "CliShell.h",
    "CliStyle.h",
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
]

REQUIRED_PATTERNS = {
    "bounded CLI line reader": r"cli_shell::readLine\s*\(",
    "help command dispatch": r'trimmed\s*==\s*"help"',
    "scan command dispatch": r'trimmed\s*==\s*"scan"',
    "probe command dispatch": r'trimmed\s*==\s*"probe"',
    "recover command dispatch": r'trimmed\s*==\s*"recover"',
    "driver command dispatch": r'trimmed\s*==\s*"drv"',
    "read command dispatch": r'trimmed\s*==\s*"read"',
    "verbose command dispatch": r'trimmed\s*==\s*"verbose"',
    "stress command dispatch": r'trimmed\s*==\s*"stress"',
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


def extract_section(text: str, start: str, end: str) -> str:
    start_index = text.find(start)
    end_index = text.find(end, start_index + len(start))
    if start_index < 0 or end_index < 0:
        fail(f"cannot locate source section from {start!r} to {end!r}")
    return text[start_index:end_index]


def main() -> int:
    common_dir = ROOT / "examples" / "common"
    bringup_main = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
    cli_shell = common_dir / "CliShell.h"

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
    shell_text = cli_shell.read_text(encoding="utf-8", errors="replace")

    for fragment in REQUIRED_FRAGMENTS:
        if fragment not in text:
            fail(f"mandatory CLI dirty diagnostic fragment '{fragment}' missing")

    for label, pattern in REQUIRED_PATTERNS.items():
        if re.search(pattern, text) is None:
            fail(f"missing {label} in {bringup_main.as_posix()}")

    line_reader_pattern = (
        r"if\s*\(\s*overflowed\s*\)\s*\{"
        r"[\s\S]*?outLine\s*=\s*LINE_TOO_LONG_MARKER\s*;"
        r"[\s\S]*?return\s+true\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(line_reader_pattern, shell_text) is None:
        fail("Arduino line reader must return the overlength marker")

    process_command = extract_section(text, "void processCommand", "void setup")
    warning_pattern = (
        r"if\s*\(\s*trimmed\s*==\s*cli_shell::LINE_TOO_LONG_MARKER\s*\)\s*\{"
        r"[\s\S]*?Input line too long"
        r"[\s\S]*?return\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(warning_pattern, process_command) is None:
        fail("Arduino command processor must report overlength input explicitly")

    if re.search(r"\bcfg\b", text) is None and re.search(r"\bsettings\b", text) is None:
        fail("either 'cfg' or 'settings' command must be present")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
