#!/usr/bin/env python3
"""Audit the public EE871 blocking-timing classification.

The public header is the source-level inventory:

    /// @note Timing contract: BUS CONTROL_READ.
    /// @note Timing contract: NO_E2_IO.

Every public callable except constructors and deleted operators must carry
exactly one marker. Every BUS declaration must also have one row in the
exhaustive method map in docs/EE871_E2_OPERATION_TIMING_BOUNDS.md.
"""

from __future__ import annotations

import re
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parent.parent
HEADER = ROOT / "include" / "EE871" / "EE871.h"
SOURCE = ROOT / "src" / "EE871.cpp"
TIMING_DOC = ROOT / "docs" / "EE871_E2_OPERATION_TIMING_BOUNDS.md"

MAP_BEGIN = "<!-- PUBLIC_BUS_TIMING_MAP_BEGIN -->"
MAP_END = "<!-- PUBLIC_BUS_TIMING_MAP_END -->"

MARKER_RE = re.compile(
    r"^\s*///\s*@note\s+Timing contract:\s*"
    r"(?P<classification>BUS\s+.+|NO_E2_IO)\.\s*$"
)
METHOD_RE = re.compile(
    r"^[ \t]*"
    r"(?P<declaration>"
    r"(?:(?:static|constexpr|inline|virtual|explicit)\s+)*"
    r"[A-Za-z_~][A-Za-z0-9_:<>,*& \t]*?[ \t]+"
    r"(?P<name>operator=|[A-Za-z_][A-Za-z0-9_]*)"
    r"\s*\((?P<parameters>[^)]*)\)"
    r"\s*(?:const\s*)?(?:=\s*delete\s*)?[;{])",
    re.MULTILINE,
)
ENUM_RE = re.compile(r"^\s*([A-Z][A-Z0-9_]*)\s*=", re.MULTILINE)
DOC_ROW_RE = re.compile(
    r"^\|\s*`(?P<method>[A-Za-z_][A-Za-z0-9_]*(?:\([^`]*)\)?)`\s*"
    r"\|\s*(?P<classification>[^|]+?)\s*\|",
    re.MULTILINE,
)


@dataclass(frozen=True)
class PublicMethod:
    name: str
    line: int
    classification: str
    kinds: frozenset[str]


def fail(messages: Iterable[str]) -> int:
    items = list(messages)
    if not items:
        print("Public timing contract audit: PASS")
        return 0
    print("Public timing contract audit: FAIL", file=sys.stderr)
    for item in items:
        print(f"- {item}", file=sys.stderr)
    return 1


def operation_kinds(header: str) -> tuple[str, ...]:
    match = re.search(
        r"enum\s+class\s+OperationKind\s*:\s*uint8_t\s*\{(?P<body>.*?)\};",
        header,
        re.DOTALL,
    )
    if not match:
        raise ValueError("OperationKind enum not found")
    return tuple(ENUM_RE.findall(match.group("body")))


def public_class_text(header: str) -> tuple[str, int]:
    class_match = re.search(r"class\s+EE871\s*\{", header)
    if not class_match:
        raise ValueError("class EE871 not found")
    public_match = re.search(r"\bpublic\s*:", header[class_match.end() :])
    if not public_match:
        raise ValueError("EE871 public section not found")
    start = class_match.end() + public_match.end()
    private_match = re.search(r"^\s*private\s*:", header[start:], re.MULTILINE)
    if not private_match:
        raise ValueError("EE871 private section not found")
    end = start + private_match.start()
    return header[start:end], start


def preceding_doc_lines(header_lines: list[str], declaration_line: int) -> list[str]:
    index = declaration_line - 2
    result: list[str] = []
    while index >= 0 and header_lines[index].lstrip().startswith("///"):
        result.append(header_lines[index])
        index -= 1
    result.reverse()
    return result


def extract_public_methods(
    header: str, known_kinds: set[str]
) -> tuple[list[PublicMethod], list[str]]:
    public_text, offset = public_class_text(header)
    header_lines = header.splitlines()
    methods: list[PublicMethod] = []
    errors: list[str] = []

    for match in METHOD_RE.finditer(public_text):
        declaration = match.group("declaration")
        name = match.group("name")
        if name == "operator=" or "= delete" in declaration:
            continue

        absolute_offset = offset + match.start()
        line = header.count("\n", 0, absolute_offset) + 1
        docs = preceding_doc_lines(header_lines, line)
        marker_lines = [doc for doc in docs if "Timing contract:" in doc]
        if len(marker_lines) != 1:
            errors.append(
                f"include/EE871/EE871.h:{line}: {name}() has "
                f"{len(marker_lines)} timing markers; expected exactly one"
            )
            continue

        marker_match = MARKER_RE.match(marker_lines[0])
        if not marker_match:
            errors.append(
                f"include/EE871/EE871.h:{line}: malformed timing marker for "
                f"{name}(): {marker_lines[0].strip()}"
            )
            continue

        classification = marker_match.group("classification")
        kinds = frozenset(
            kind for kind in known_kinds
            if re.search(rf"\b{re.escape(kind)}\b", classification)
        )
        if classification.startswith("BUS ") and not kinds:
            errors.append(
                f"include/EE871/EE871.h:{line}: BUS marker for {name}() "
                "does not name an OperationKind"
            )
        if classification == "NO_E2_IO" and kinds:
            errors.append(
                f"include/EE871/EE871.h:{line}: NO_E2_IO marker for {name}() "
                "also names a bus kind"
            )
        methods.append(PublicMethod(name, line, classification, kinds))

    return methods, errors


def extract_method_map(document: str) -> list[tuple[str, str, str]]:
    if document.count(MAP_BEGIN) != 1 or document.count(MAP_END) != 1:
        raise ValueError(
            "timing document must contain exactly one public-method map marker pair"
        )
    section = document.split(MAP_BEGIN, 1)[1].split(MAP_END, 1)[0]
    rows: list[tuple[str, str, str]] = []
    for match in DOC_ROW_RE.finditer(section):
        method_label = match.group("method")
        method_name = method_label.split("(", 1)[0]
        classification = match.group("classification")
        rows.append((method_name, method_label, classification))
    return rows


def main() -> int:
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    timing_doc = TIMING_DOC.read_text(encoding="utf-8")
    errors: list[str] = []

    try:
        kinds = operation_kinds(header)
        methods, method_errors = extract_public_methods(header, set(kinds))
        errors.extend(method_errors)
        rows = extract_method_map(timing_doc)
    except ValueError as exc:
        return fail([str(exc)])

    for kind in kinds:
        if not re.search(
            rf"\bcase\s+OperationKind::{re.escape(kind)}\s*:", source
        ):
            errors.append(f"src/EE871.cpp: calculator has no case for {kind}")
        if not re.search(
            rf"^\|\s*`{re.escape(kind)}`\s*\|",
            timing_doc,
            re.MULTILINE,
        ):
            errors.append(
                "docs/EE871_E2_OPERATION_TIMING_BOUNDS.md: "
                f"formula table has no row for {kind}"
            )

    bus_methods = [method for method in methods if method.classification.startswith("BUS ")]
    header_counts = Counter(method.name for method in bus_methods)
    doc_counts = Counter(row[0] for row in rows)

    for name in sorted(set(header_counts) | set(doc_counts)):
        expected = header_counts[name]
        actual = doc_counts[name]
        if expected != actual:
            errors.append(
                "docs/EE871_E2_OPERATION_TIMING_BOUNDS.md: "
                f"public BUS method {name} has {expected} declaration(s) but "
                f"{actual} method-map row(s)"
            )

    rows_by_name: dict[str, list[tuple[str, str, str]]] = {}
    for row in rows:
        rows_by_name.setdefault(row[0], []).append(row)
    method_occurrence: Counter[str] = Counter()
    for method in bus_methods:
        occurrence = method_occurrence[method.name]
        method_occurrence[method.name] += 1
        named_rows = rows_by_name.get(method.name, [])
        if occurrence >= len(named_rows):
            continue
        classification = named_rows[occurrence][2]
        documented_kinds = {
            kind for kind in kinds
            if re.search(rf"\b{re.escape(kind)}\b", classification)
        }
        if documented_kinds != set(method.kinds):
            errors.append(
                "docs/EE871_E2_OPERATION_TIMING_BOUNDS.md: "
                f"{named_rows[occurrence][1]} documents "
                f"{sorted(documented_kinds)}, header marker names "
                f"{sorted(method.kinds)}"
            )

    block_read = re.search(
        r"blockRead\s*&&\s*\(elementCount\s*==\s*0U\s*\|\|\s*"
        r"elementCount\s*>\s*256U\)",
        source,
    )
    block_write = re.search(
        r"blockWrite\s*&&\s*\(elementCount\s*==\s*0U\s*\|\|\s*"
        r"elementCount\s*>\s*16U\)",
        source,
    )
    fixed_count = re.search(
        r"!blockRead\s*&&\s*!blockWrite\s*&&\s*elementCount\s*!=\s*1U",
        source,
    )
    if not block_read:
        errors.append("calculator does not visibly enforce block-read count 1..256")
    if not block_write:
        errors.append("calculator does not visibly enforce block-write count 1..16")
    if not fixed_count:
        errors.append("calculator does not visibly enforce fixed-operation count 1")

    return fail(errors)


if __name__ == "__main__":
    raise SystemExit(main())
