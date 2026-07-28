#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

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
    "samplefast",
    "sampleavg",
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
    "driver health dirty output": r"void\s+printDriverHealth\s*\([^)]*\)\s*\{[\s\S]*?printPersistentDirtyFields\s*\(\s*settings\s*\)",
    "status dirty summary": r"hasCo2Error\(\):[\s\S]*?printPersistentDirtySummaryIfDirty\s*\(",
    "resync before after output": r'std::strcmp\(\s*trimmed\s*,\s*"resync"\s*\)\s*==\s*0[\s\S]*?Before:[\s\S]*?resyncPersistentConfig\s*\(\s*\)[\s\S]*?After:',
    "dirty error code detail output": r"persistentConfigDirtyError:[\s\S]*?code=%u,\s*detail=%ld",
    "samplefast help entry": r'printHelpItem\(\s*"samplefast"\s*,',
    "sampleavg help entry": r'printHelpItem\(\s*"sampleavg"\s*,',
    "checked result formatter": r"void\s+printCheckedSample\s*\([^)]*Co2ReadResult",
    "mutation target formatter": r"mutationTargetToStr\s*\(",
    "mutation effect formatter": r"mutationEffectToStr\s*\(",
    "CO2 sensor error formatter": r"co2SensorErrorToStr\s*\(",
    "IDF delayMs callback use": r"deviceCfg\.delayMs\s*=\s*ee871_idf::delayMs\s*;",
    "IDF yield callback use": r"deviceCfg\.yield\s*=\s*ee871_idf::yieldTask\s*;",
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


def command_branch(text: str, command: str) -> str:
    start_match = re.search(
        rf'(?:if|else\s+if)\s*\(\s*std::strcmp\(\s*trimmed\s*,\s*"{re.escape(command)}"\s*\)\s*==\s*0\s*\)\s*\{{',
        text,
    )
    if start_match is None:
        fail(f"IDF command dispatch branch {command!r} missing")
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
        fail(f"IDF {command!r} must call {required_call}")
    for forbidden in forbidden_calls:
        if forbidden in branch:
            fail(f"IDF {command!r} must not call {forbidden}")


def main() -> int:
    arduino = read(ARDUINO_MAIN)
    idf = read(IDF_MAIN)
    transport = read(IDF_TRANSPORT)

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
        if re.search(pattern, idf) is None:
            fail(f"IDF CLI missing {label}")

    require_command_call(
        idf,
        "co2fast",
        "readCo2Fast(ppm)",
        ("readCo2FastSample", "readStatus(", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        idf,
        "co2avg",
        "readCo2Average(ppm)",
        ("readCo2AverageSample", "readStatus(", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        idf,
        "samplefast",
        "readCo2FastSample(result)",
        ("readCo2AverageSample", "customWrite(", "startAutoAdjust("),
    )
    require_command_call(
        idf,
        "sampleavg",
        "readCo2AverageSample(result)",
        ("readCo2FastSample", "customWrite(", "startAutoAdjust("),
    )

    if "driver/gpio.h" not in transport:
        fail("ESP-IDF E2 GPIO transport must use driver/gpio.h")
    for fragment in (
        "inline void delayMs(uint32_t ms, void*)",
        "vTaskDelay(",
        "inline void yieldTask(void*)",
        "taskYIELD();",
        "cfg.delayMs = delayMs;",
        "cfg.yield = yieldTask;",
    ):
        if fragment not in transport:
            fail(f"ESP-IDF transport missing long-wait mapping: {fragment!r}")
    forbidden_patterns = {
        "Arduino.h": r'#\s*include\s*[<"]Arduino\.h[>"]',
        "Wire.h": r'#\s*include\s*[<"]Wire\.h[>"]',
        "Arduino String": r"\bString\s+[A-Za-z_]",
        "Arduino Serial": r"\bSerial\s*\.",
    }
    for label, pattern in forbidden_patterns.items():
        if re.search(pattern, idf) or re.search(pattern, transport):
            fail(f"ESP-IDF example contains forbidden Arduino surface: {label}")

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
