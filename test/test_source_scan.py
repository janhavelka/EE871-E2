from __future__ import annotations

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from source_scan import strip_cpp_comments, strip_cpp_non_code


class SourceScanTests(unittest.TestCase):
    def test_digit_separators_do_not_hide_following_comments(self) -> None:
        source = "int n = 1'000; // printHelpItem(\"dirty\", \"help\");\nrecover();\n"
        for strip in (strip_cpp_comments, strip_cpp_non_code):
            result = strip(source)
            self.assertIn("1'000", result)
            self.assertNotIn("printHelpItem", result)
            self.assertIn("recover()", result)
            self.assertEqual(len(source), len(result))

    def test_unterminated_quotes_do_not_hide_later_code(self) -> None:
        for quote in ('"', "'"):
            for newline in ("\n", "\r\n"):
                source = f"auto broken = {quote}oops{newline}millis();{newline}"
                result = strip_cpp_non_code(source)
                self.assertIn("millis()", result)
                self.assertEqual(len(source), len(result))
                self.assertEqual(source.count("\n"), result.count("\n"))

    def test_line_comment_continuation_hides_commented_code(self) -> None:
        for newline in ("\n", "\r\n"):
            source = "// comment\\" + newline + 'printHelpItem("dirty", "help");\\' + newline
            source += "micros();" + newline + "millis();" + newline
            for strip in (strip_cpp_comments, strip_cpp_non_code):
                result = strip(source)
                self.assertNotIn("printHelpItem", result)
                self.assertNotIn("micros()", result)
                self.assertIn("millis()", result)
                self.assertEqual(len(source), len(result))

    def test_escaped_newline_remains_inside_literal(self) -> None:
        for quote in ('"', "'"):
            for newline in ("\n", "\r\n"):
                source = quote + "text\\" + newline + "millis()" + quote + "; micros();\n"
                result = strip_cpp_non_code(source)
                self.assertNotIn("millis()", result)
                self.assertIn("micros()", result)

    def test_character_literals_and_digit_separator_context(self) -> None:
        for prefix in ("", "u8", "u", "U", "L"):
            source = f"auto c = {prefix}'/'; int n = 0xAB'CD; // ignored()\n"
            result = strip_cpp_non_code(source)
            self.assertNotIn("'/'", result)
            self.assertIn("0xAB'CD", result)
            self.assertNotIn("ignored()", result)
            result = strip_cpp_comments(source)
            self.assertIn(prefix + "'/'", result)
            self.assertNotIn("ignored()", result)

    def test_comment_marker_inside_string_does_not_hide_following_code(self) -> None:
        source = 'const char* url = "https://example.test"; millis();\n'
        code = strip_cpp_non_code(source)
        self.assertIn("millis()", code)
        self.assertNotIn("https://", code)

    def test_calls_in_comments_and_literals_are_removed(self) -> None:
        source = '"millis()"; // micros()\n/* yield(); */ delayMicroseconds(1);\n'
        code = strip_cpp_non_code(source)
        self.assertNotIn("millis()", code)
        self.assertNotIn("micros()", code)
        self.assertNotIn("yield()", code)
        self.assertIn("delayMicroseconds(1)", code)

    def test_comment_stripping_preserves_real_include_only(self) -> None:
        source = '/* #include <Arduino.h> */\n#include "Arduino.h"\n'
        comments_removed = strip_cpp_comments(source)
        self.assertEqual(1, comments_removed.count("Arduino.h"))

    def test_raw_string_comment_markers_are_not_parsed_as_comments(self) -> None:
        source = 'auto text = R"tag(// text /* text */)tag"; micros();\n'
        self.assertIn("micros()", strip_cpp_non_code(source))
        self.assertIn("// text /* text */", strip_cpp_comments(source))


if __name__ == "__main__":
    unittest.main()
