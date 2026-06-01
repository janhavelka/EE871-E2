#!/usr/bin/env python3
"""Parser regression tests for the EE871-E2 HIL runner."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "ee871_hil_runner.py"

spec = importlib.util.spec_from_file_location("ee871_hil_runner", RUNNER)
assert spec is not None
hil = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = hil
spec.loader.exec_module(hil)


class ParserTests(unittest.TestCase):
    def test_parse_selftest_counters(self) -> None:
        parsed = hil.parse_response(
            "selftest",
            "\x1b[32mSelftest result: pass=18 fail=0 skip=2\x1b[0m\n",
        )

        self.assertEqual({"pass": 18, "fail": 0, "skip": 2}, parsed["selftest"])
        result, reason = hil.classify_response(
            hil.CommandSpec("selftest", "selftest", expected_any=("Selftest result:",), validators=("selftest",)),
            "Selftest result: pass=18 fail=0 skip=2\n",
            False,
            parsed,
        )
        self.assertEqual(hil.RESULT_PASS, result, reason)

    def test_parse_stress_summary(self) -> None:
        text = """=== Stress Summary ===
  Total: 50
  Success: 50
  Errors: 0
  Health delta: success +50, failures +0
"""
        parsed = hil.parse_response("stress 50", text)

        self.assertEqual({"kind": "stress", "total": 50, "success": 50, "errors": 0}, parsed["stress"])
        result, reason = hil.classify_response(
            hil.CommandSpec("stress 50", "stress", expected_any=("Stress Summary",), validators=("stress",)),
            text,
            False,
            parsed,
        )
        self.assertEqual(hil.RESULT_PASS, result, reason)

    def test_stress_nonzero_errors_fail(self) -> None:
        text = """=== Stress Summary ===
  Total: 50
  Success: 49
  Errors: 1
"""
        parsed = hil.parse_response("stress 50", text)
        result, reason = hil.classify_response(
            hil.CommandSpec("stress 50", "stress", expected_any=("Stress Summary",), validators=("stress",)),
            text,
            False,
            parsed,
        )

        self.assertEqual(hil.RESULT_FAIL, result)
        self.assertIn("stress errors=1", reason)

    def test_parse_driver_health(self) -> None:
        text = """=== Driver Health ===
  State: READY
  Online: yes
  Consecutive failures: 0
  Total success: 61
  Total failures: 0
  persistentConfigDirty: no
  persistentConfigDirtyError: OK (code=0, detail=0)
  persistentConfigDirtyError message: OK
  resyncNeeded: no
"""
        parsed = hil.parse_response("drv", text)

        self.assertEqual("READY", parsed["driver_state"])
        self.assertTrue(parsed["online"])
        self.assertFalse(parsed["persistent_config_dirty"])
        result, reason = hil.classify_response(
            hil.CommandSpec("drv", "drv", expected_any=("Driver Health",), validators=("health_ready",)),
            text,
            False,
            parsed,
        )
        self.assertEqual(hil.RESULT_PASS, result, reason)

    def test_parse_dirty_uses_after_state_from_resync(self) -> None:
        text = """=== Persistent Config Resync ===
Before:
  persistentConfigDirty: yes
  persistentConfigDirtyError: E2_ERROR (code=3, detail=17)
  persistentConfigDirtyError message: Write verify failed
  resyncNeeded: yes
  Status: OK (code=0, detail=0)
After:
  persistentConfigDirty: no
  persistentConfigDirtyError: OK (code=0, detail=0)
  persistentConfigDirtyError message: OK
  resyncNeeded: no
"""
        parsed = hil.parse_response("resync", text)

        self.assertFalse(parsed["persistent_config_dirty"])
        self.assertFalse(parsed["resync_needed"])
        self.assertEqual("OK", parsed["persistent_config_dirty_error"]["name"])

    def test_missing_dirty_parse_requires_operator_review(self) -> None:
        parsed = hil.parse_response("dirty", "=== Persistent Config Dirty State ===\n")
        result, reason = hil.classify_response(
            hil.CommandSpec("dirty", "dirty", expected_any=("Persistent Config Dirty State",), validators=("dirty_clean",)),
            "=== Persistent Config Dirty State ===\n",
            False,
            parsed,
        )

        self.assertEqual(hil.RESULT_OPERATOR, result)
        self.assertIn("persistent dirty flag not parsed", reason)

    def test_verdict_rules(self) -> None:
        self.assertEqual(hil.VERDICT_INCOMPLETE, hil.verdict([], dry_run=True))
        self.assertEqual(hil.VERDICT_PASS, hil.verdict([{"result": hil.RESULT_PASS}], dry_run=False))
        self.assertEqual(hil.VERDICT_FAIL, hil.verdict([{"result": hil.RESULT_FAIL}], dry_run=False))
        self.assertEqual(hil.VERDICT_OPERATOR, hil.verdict([{"result": hil.RESULT_OPERATOR}], dry_run=False))
        self.assertEqual(hil.VERDICT_INCOMPLETE, hil.verdict([{"result": hil.RESULT_SKIP}], dry_run=False))


if __name__ == "__main__":
    unittest.main()
