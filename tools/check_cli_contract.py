#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

from source_scan import strip_cpp_comments, strip_cpp_non_code

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

PROCESS_COMMAND_PATTERNS = {
    "help command dispatch",
    "scan command dispatch",
    "probe command dispatch",
    "recover command dispatch",
    "driver command dispatch",
    "read command dispatch",
    "verbose command dispatch",
    "stress command dispatch",
    "dirty command dispatch",
    "resync command dispatch",
    "resync before after output",
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
    diagnostics = common_dir / "E2Diagnostics.h"

    ensure_exists(common_dir, "common example directory")
    ensure_exists(bringup_main, "bringup CLI example")

    ensure_missing(ROOT / "examples" / "00_smoke_boot", "deprecated example 00_smoke_boot")
    ensure_missing(
        ROOT / "examples" / "03_feature_walkthrough",
        "deprecated example 03_feature_walkthrough",
    )

    for name in REQUIRED_COMMON:
        ensure_exists(common_dir / name, f"common helper {name}")

    text = strip_cpp_comments(bringup_main.read_text(encoding="utf-8", errors="replace"))
    shell_text = strip_cpp_comments(cli_shell.read_text(encoding="utf-8", errors="replace"))
    diagnostics_text = strip_cpp_comments(
        diagnostics.read_text(encoding="utf-8", errors="replace")
    )
    process_command = extract_section(text, "void processCommand", "void setup")

    for fragment in REQUIRED_FRAGMENTS:
        if fragment not in text:
            fail(f"mandatory CLI dirty diagnostic fragment '{fragment}' missing")

    for label, pattern in REQUIRED_PATTERNS.items():
        scope = process_command if label in PROCESS_COMMAND_PATTERNS else text
        if re.search(pattern, scope) is None:
            fail(f"missing {label} in {bringup_main.as_posix()}")

    line_reader_pattern = (
        r"if\s*\(\s*overflowed\s*\)\s*\{"
        r"[\s\S]*?outLine\s*=\s*LINE_TOO_LONG_MARKER\s*;"
        r"[\s\S]*?return\s+true\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(line_reader_pattern, shell_text) is None:
        fail("Arduino line reader must return the overlength marker")

    warning_pattern = (
        r"if\s*\(\s*trimmed\s*==\s*cli_shell::LINE_TOO_LONG_MARKER\s*\)\s*\{"
        r"[\s\S]*?Input line too long"
        r"[\s\S]*?return\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(warning_pattern, process_command) is None:
        fail("Arduino command processor must report overlength input explicitly")

    scanner = extract_section(diagnostics_text, "inline void scanAddresses", "struct TimingResult")
    scanner_code = strip_cpp_non_code(scanner)
    if re.search(r"SCAN_ATTEMPTS\s*=\s*5\s*;", scanner_code) is None:
        fail("Arduino scanner must use five bounded attempts")
    if re.search(
        r"for\s*\(\s*uint8_t\s+attempt\s*=\s*0\s*;"
        r"\s*attempt\s*<\s*SCAN_ATTEMPTS\s*;\s*\+\+attempt\s*\)",
        scanner_code,
    ) is None:
        fail("Arduino scanner must retry through SCAN_ATTEMPTS")
    if re.search(r"candidate\s*\.\s*begin\s*\(", scanner_code) is None:
        fail("Arduino scanner must use production begin() identity validation")
    if re.search(r"candidate\s*\.\s*readStatus\s*\(", scanner_code) is None:
        fail("Arduino scanner must use the production status/PEC path")
    for raw_call in ("sendStart", "sendByteRaw", "readByteRaw"):
        if re.search(rf"\b{raw_call}\s*\(", scanner_code):
            fail(f"Arduino scanner still uses raw diagnostic call {raw_call!r}")
    success_gate = (
        r"if\s*\(\s*st\s*\.\s*ok\s*\(\s*\)\s*\)\s*\{\s*"
        r"status\s*\[\s*addr\s*\]\s*=\s*candidateStatus\s*;\s*"
        r"found\s*\[\s*addr\s*\]\s*=\s*true\s*;\s*\}"
    )
    if len(re.findall(success_gate, scanner_code)) != 1:
        fail("Arduino scanner must gate its only discovery assignment on final success")
    retained_error = (
        r"if\s*\(\s*lastError\s*\[\s*addr\s*\]\s*\.\s*ok\s*\(\s*\)\s*"
        r"\|\|\s*st\s*\.\s*code\s*!=\s*EE871::Err::NACK\s*\)\s*\{\s*"
        r"lastError\s*\[\s*addr\s*\]\s*=\s*st\s*;\s*\}"
    )
    if re.search(retained_error, scanner_code) is None:
        fail("Arduino scanner must retain non-NACK evidence across later attempts")

    if re.search(r"\bcfg\b", text) is None and re.search(r"\bsettings\b", text) is None:
        fail("either 'cfg' or 'settings' command must be present")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
