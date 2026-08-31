"""Small C/C++ lexical helpers for repository contract checks.

The functions preserve source length and newlines so regular-expression based
checks can ignore comments and literals without being confused by comment
markers inside strings.
"""

from __future__ import annotations


def _blank(segment: str) -> str:
    return "".join("\n" if char == "\n" else " " for char in segment)


def _raw_string_end(text: str, start: int) -> int | None:
    if not text.startswith('R"', start):
        return None
    delimiter_end = text.find("(", start + 2, start + 19)
    if delimiter_end < 0:
        return None
    delimiter = text[start + 2 : delimiter_end]
    if any(char.isspace() or char in "()\\" for char in delimiter):
        return None
    terminator = ")" + delimiter + '"'
    end = text.find(terminator, delimiter_end + 1)
    return len(text) if end < 0 else end + len(terminator)


def _strip_cpp(text: str, *, strip_literals: bool) -> str:
    output: list[str] = []
    index = 0
    length = len(text)

    while index < length:
        if text.startswith("//", index):
            end = text.find("\n", index + 2)
            end = length if end < 0 else end
            output.append(_blank(text[index:end]))
            index = end
            continue

        if text.startswith("/*", index):
            end_marker = text.find("*/", index + 2)
            end = length if end_marker < 0 else end_marker + 2
            output.append(_blank(text[index:end]))
            index = end
            continue

        raw_end = _raw_string_end(text, index)
        if raw_end is not None:
            segment = text[index:raw_end]
            output.append(_blank(segment) if strip_literals else segment)
            index = raw_end
            continue

        quote = text[index]
        if quote in {'"', "'"}:
            end = index + 1
            while end < length:
                if text[end] == "\\":
                    end = min(length, end + 2)
                    continue
                if text[end] == quote:
                    end += 1
                    break
                end += 1
            segment = text[index:end]
            output.append(_blank(segment) if strip_literals else segment)
            index = end
            continue

        output.append(text[index])
        index += 1

    return "".join(output)


def strip_cpp_comments(text: str) -> str:
    """Replace C/C++ comments with spaces while preserving literals."""

    return _strip_cpp(text, strip_literals=False)


def strip_cpp_non_code(text: str) -> str:
    """Replace C/C++ comments and string/character literals with spaces."""

    return _strip_cpp(text, strip_literals=True)
