#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

from source_scan import strip_cpp_comments, strip_cpp_non_code

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN = ROOT / "examples" / "idf" / "basic_bringup" / "main" / "main.cpp"
IDF_TRANSPORT = ROOT / "examples" / "idf" / "common" / "E2GpioTransport.h"

HELP_ITEM_RE = re.compile(
    r"printHelpItem\(\s*\"([^\"]+)\"\s*,\s*\"([^\"]+)\"\s*\)"
)
HELP_SECTION_RE = re.compile(r"printHelpSection\(\s*\"([^\"]+)\"\s*\)")

IDF_REQUIRED_FRAGMENTS = [
    "=== EE871 Bringup Example ===",
    "=== EE871-E2 CLI Help ===",
    "=== Driver Health ===",
    "=== Version Info ===",
    "=== Stress Summary ===",
    "=== stress_mix summary ===",
    "=== EE871 selftest (safe commands) ===",
    "=== FULL E2 BUS DIAGNOSTICS ===",
    "=== Custom Register Dump ===",
    "=== Bus Trace ===",
    "Status:",
    "> ",
    "processCommand",
    "readControlByte",
    "readU16",
    "customRead",
    "customWrite",
    "persistentConfigDirty",
    "persistentConfigDirtyError",
    "persistentConfigDirtyError message",
    "resyncNeeded",
    "resyncPersistentConfig",
    "Write persistent custom register (bench only)",
    "Write persistent interval",
    "Write persistent CO2 offset",
    "Write persistent CO2 gain",
    "recover",
    "busReset",
    "checkBusIdle",
]

IDF_REQUIRED_PATTERNS = {
    "dirty command dispatch": r'std::strcmp\(\s*trimmed\s*,\s*"dirty"\s*\)\s*==\s*0',
    "resync command dispatch": r'std::strcmp\(\s*trimmed\s*,\s*"resync"\s*\)\s*==\s*0',
    "dirty accessor": r"persistentConfigDirty",
    "dirty error accessor": r"persistentConfigDirtyError",
    "resync API": r"resyncPersistentConfig",
    "newline-terminated prompt": r'void\s+printPrompt\s*\([^)]*\)\s*\{[\s\S]*?std::printf\(\s*">\s*\\n"\s*\)',
    "prompt after trace drain": r"buslog::flush\s*\(\s*\)\s*;[\s\S]*?if\s*\(\s*promptPending\s*&&\s*buslog::empty\s*\(\s*\)\s*\)[\s\S]*?printPrompt\s*\(\s*\)",
    "driver health dirty output": r"void\s+printDriverHealth\s*\([^)]*\)\s*\{[\s\S]*?printPersistentDirtyFields\s*\(\s*settings\s*\)",
    "status dirty summary": r"hasCo2Error\(\):[\s\S]*?printPersistentDirtySummaryIfDirty\s*\(",
    "resync before after output": r'std::strcmp\(\s*trimmed\s*,\s*"resync"\s*\)\s*==\s*0[\s\S]*?Before:[\s\S]*?resyncPersistentConfig\s*\(\s*\)[\s\S]*?After:',
    "dirty error code detail output": r"persistentConfigDirtyError:[\s\S]*?code=%u,\s*detail=%ld",
    "library test dispatch": r"testLibraryCommands\s*\(\s*device\s*\)",
}

PROCESS_COMMAND_PATTERNS = {
    "dirty command dispatch",
    "resync command dispatch",
    "resync before after output",
    "library test dispatch",
}

STALE_IDF_WORDING = [
    "minimal idf example",
    "minimal esp-idf example",
    "minimal `app_main()`",
    "small native esp-idf example",
]


def fail(msg: str) -> None:
    print(f"IDF example contract FAILED: {msg}")
    raise SystemExit(1)


def read(path: pathlib.Path) -> str:
    if not path.exists():
        fail(f"missing required file: {path.relative_to(ROOT).as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def extract_help(text: str) -> tuple[list[str], list[tuple[str, str]]]:
    return HELP_SECTION_RE.findall(text), HELP_ITEM_RE.findall(text)


def extract_section(text: str, start: str, end: str) -> str:
    start_index = text.find(start)
    end_index = text.find(end, start_index + len(start))
    if start_index < 0 or end_index < 0:
        fail(f"cannot locate source section from {start!r} to {end!r}")
    return text[start_index:end_index]


def main() -> int:
    arduino = strip_cpp_comments(read(ARDUINO_MAIN))
    idf = strip_cpp_comments(read(IDF_MAIN))
    transport = strip_cpp_comments(read(IDF_TRANSPORT))
    process_command = extract_section(idf, "void processCommand", "void configureConsoleInput")

    arduino_sections, arduino_items = extract_help(arduino)
    idf_sections, idf_items = extract_help(idf)

    if idf_sections != arduino_sections:
        fail(f"help sections differ: arduino={arduino_sections!r} idf={idf_sections!r}")

    if idf_items != arduino_items:
        missing = [item for item in arduino_items if item not in idf_items]
        extra = [item for item in idf_items if item not in arduino_items]
        fail(f"help items differ: missing={missing!r} extra={extra!r}")

    for fragment in IDF_REQUIRED_FRAGMENTS:
        if fragment not in idf:
            fail(f"IDF CLI missing required fragment: {fragment!r}")

    for label, pattern in IDF_REQUIRED_PATTERNS.items():
        scope = process_command if label in PROCESS_COMMAND_PATTERNS else idf
        if re.search(pattern, scope) is None:
            fail(f"IDF CLI missing {label}")

    scanner = extract_section(idf, "void scanAddresses", "struct TimingResult")
    scanner_code = strip_cpp_non_code(scanner)
    if re.search(r"SCAN_ATTEMPTS\s*=\s*5\s*;", scanner_code) is None:
        fail("IDF scanner must use five bounded attempts")
    if re.search(
        r"for\s*\(\s*uint8_t\s+attempt\s*=\s*0\s*;"
        r"\s*attempt\s*<\s*SCAN_ATTEMPTS\s*;\s*\+\+attempt\s*\)",
        scanner_code,
    ) is None:
        fail("IDF scanner must retry through SCAN_ATTEMPTS")
    success_gate = (
        r"if\s*\(\s*st\s*\.\s*ok\s*\(\s*\)\s*\)\s*\{\s*"
        r"status\s*\[\s*addr\s*\]\s*=\s*candidateStatus\s*;\s*"
        r"found\s*\[\s*addr\s*\]\s*=\s*true\s*;\s*\}"
    )
    if len(re.findall(success_gate, scanner_code)) != 1:
        fail("IDF scanner must gate its only discovery assignment on final success")
    retained_error = (
        r"if\s*\(\s*lastError\s*\[\s*addr\s*\]\s*\.\s*ok\s*\(\s*\)\s*"
        r"\|\|\s*st\s*\.\s*code\s*!=\s*EE871::Err::NACK\s*\)\s*\{\s*"
        r"lastError\s*\[\s*addr\s*\]\s*=\s*st\s*;\s*\}"
    )
    if re.search(retained_error, scanner_code) is None:
        fail("IDF scanner must retain non-NACK evidence across later attempts")
    if re.search(r"candidate\s*\.\s*begin\s*\(", scanner_code) is None:
        fail("IDF scanner must use production begin() identity validation")
    if re.search(r"candidate\s*\.\s*readStatus\s*\(", scanner_code) is None:
        fail("IDF scanner must use the production status/PEC path")
    for raw_call in ("sendStart", "sendByteRaw", "readByteRaw"):
        if re.search(rf"\b{raw_call}\s*\(", scanner_code):
            fail(f"IDF scanner still uses raw diagnostic call {raw_call!r}")
    if re.search(r"\bfound\s*\+\+", scanner_code):
        fail("IDF scanner still counts ACK-only responses")

    libtest = extract_section(idf, "void testLibraryCommands", "void runFullDiagnostics")
    libtest_code = strip_cpp_non_code(libtest)
    if re.search(
        r"void\s+testLibraryCommands\s*\(\s*EE871::EE871\s*&\s*driver\s*\)",
        libtest_code,
    ) is None:
        fail("IDF library test must accept the initialized driver")
    if re.search(r"driver\s*\.\s*readControlByte\s*\(", libtest_code) is None:
        fail("IDF library test must use the production control-byte path")
    for raw_call in ("sendStart", "sendByteRaw", "readByteRaw"):
        if re.search(rf"\b{raw_call}\s*\(", libtest_code):
            fail(f"IDF library test still uses raw diagnostic call {raw_call!r}")

    warning_pattern = (
        r"if\s*\(\s*std::strcmp\s*\(\s*trimmed\s*,\s*LINE_TOO_LONG_MARKER\s*\)"
        r"\s*==\s*0\s*\)\s*\{"
        r"[\s\S]*?Input line too long"
        r"[\s\S]*?return\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(warning_pattern, process_command) is None:
        fail("IDF command processor must report overlength input explicitly")

    poll_line = extract_section(idf, "bool pollLine", "void configureDevice")
    line_reader_pattern = (
        r"if\s*\(\s*overflowed\s*\)\s*\{"
        r"[\s\S]*?std::strncpy\s*\(\s*out\s*,\s*LINE_TOO_LONG_MARKER\s*,"
        r"[\s\S]*?return\s+true\s*;"
        r"[\s\S]*?\}"
    )
    if re.search(line_reader_pattern, poll_line) is None:
        fail("IDF line reader must return the overlength marker and preserve prompt flow")

    if "driver/gpio.h" not in transport:
        fail("ESP-IDF E2 GPIO transport must use driver/gpio.h")

    docs_to_scan = [
        ROOT / "README.md",
        ROOT / "CHANGELOG.md",
        ROOT / "docs" / "IDF_PORT.md",
        ROOT / "docs" / "IDF_PORT_IMPLEMENTATION.md",
        ROOT / "examples" / "idf" / "basic_bringup" / "README.md",
    ]
    for path in docs_to_scan:
        text = read(path).lower()
        for phrase in STALE_IDF_WORDING:
            if phrase in text:
                fail(f"stale IDF wording {phrase!r} remains in {path.relative_to(ROOT).as_posix()}")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
