from __future__ import annotations

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from source_scan import strip_cpp_comments, strip_cpp_non_code


class SourceScanTests(unittest.TestCase):
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
