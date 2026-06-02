from __future__ import annotations

import importlib.util
import contextlib
import io
import pathlib
import sys
import time
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "ee871_hil_runner.py"
SPEC = importlib.util.spec_from_file_location("ee871_hil_runner", MODULE_PATH)
assert SPEC is not None
runner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


class HilRunnerParserTest(unittest.TestCase):
    def test_parse_selftest_counts_ansi_output(self) -> None:
        text = """
\x1b[36m=== EE871 selftest (safe commands) ===\x1b[0m
  [\x1b[32mPASS\x1b[0m] probe responds
  [\x1b[32mPASS\x1b[0m] probe no-health-side-effects
  [\x1b[33mSKIP\x1b[0m] readErrorCode - not supported
Selftest result: pass=\x1b[32m2\x1b[0m fail=\x1b[32m0\x1b[0m skip=\x1b[33m1\x1b[0m
> """
        parsed = runner.parse_selftest(text)
        self.assertEqual(parsed["selftest"]["pass"], 2)
        self.assertEqual(parsed["selftest"]["fail"], 0)
        self.assertEqual(parsed["selftest"]["skip"], 1)

    def test_parse_selftest_failure_sets_fail_verdict(self) -> None:
        text = """
=== EE871 selftest (safe commands) ===
  [PASS] probe responds
  [FAIL] readGroup - TIMEOUT
Selftest result: pass=1 fail=1 skip=0
> """
        parsed = runner.parse_selftest(text)
        self.assertEqual(parsed["selftest"]["fail"], 1)

    def test_parse_selftest_uses_latest_result_block(self) -> None:
        text = """
Selftest result: pass=10 fail=0 skip=0
noise from previous command
Selftest result: pass=9 fail=1 skip=0
> """
        parsed = runner.parse_response("selftest", text)
        self.assertEqual(parsed["selftest"], {"pass": 9, "fail": 1, "skip": 0})
        result, reason = runner.classify_response(
            runner.CommandSpec("selftest", "selftest", expected_any=("Selftest result:",), validators=("selftest",)),
            text,
            False,
            parsed,
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("selftest fail=1", reason)

    def test_parse_regular_stress_summary(self) -> None:
        text = """
=== Stress Summary ===
  Total: 50
  Success: \x1b[32m50\x1b[0m
  Errors: \x1b[32m0\x1b[0m
  Success rate: \x1b[32m100.00%\x1b[0m
  Duration: 3400 ms
  Rate: 14.71 ops/s
  Health delta: \x1b[32msuccess +50\x1b[0m, \x1b[32mfailures +0\x1b[0m
> """
        parsed = runner.parse_stress(text)
        self.assertEqual(parsed["stress"]["kind"], "stress")
        self.assertEqual(parsed["stress"]["total"], 50)
        self.assertEqual(parsed["stress"]["success"], 50)
        self.assertEqual(parsed["stress"]["errors"], 0)
        self.assertEqual(parsed["health_delta_success"], 50)
        self.assertEqual(parsed["health_delta_failures"], 0)

    def test_parse_stress_mix_summary(self) -> None:
        text = """
=== stress_mix summary ===
  Total: ok=19 fail=1 (95.00%)
  Duration: 1500 ms
  Rate: 13.33 ops/s
  readStatus  ok=3 fail=0
  readCo2Avg  ok=2 fail=1
  Health delta: success +19, failures +1
> """
        parsed = runner.parse_stress(text)
        self.assertEqual(parsed["stress"]["kind"], "stress_mix")
        self.assertEqual(parsed["stress"]["total"], 20)
        self.assertEqual(parsed["stress"]["success"], 19)
        self.assertEqual(parsed["stress"]["errors"], 1)

    def test_parse_stress_uses_latest_block_and_health_delta(self) -> None:
        text = """
=== Stress Summary ===
  Total: 50
  Success: 50
  Errors: 0
  Health delta: success +50, failures +0
=== Stress Summary ===
  Total: 50
  Success: 50
  Errors: 0
  Health delta: success +49, failures +1
> """
        parsed = runner.parse_response("stress 50", text)
        self.assertEqual(parsed["stress"]["errors"], 0)
        self.assertEqual(parsed["health_delta_failures"], 1)
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("stress 50", "stress", validators=("stress",)),
            parsed,
        )
        self.assertIn("stress health failures +1", failures)
        self.assertEqual(reviews, [])

    def test_parse_health_and_dirty(self) -> None:
        text = """
=== Driver Health ===
  State: \x1b[32mREADY\x1b[0m
  Online: \x1b[32myes\x1b[0m
  Consecutive failures: \x1b[32m0\x1b[0m
  Total success: \x1b[32m42\x1b[0m
  Total failures: \x1b[32m0\x1b[0m
  Success rate: \x1b[32m100.0%\x1b[0m
  Last OK: 5 ms ago (at 1000 ms)
  Last error: never
  persistentConfigDirty: \x1b[32mno\x1b[0m
  persistentConfigDirtyError: OK (code=0, detail=0)
  persistentConfigDirtyError message: <none>
  resyncNeeded: \x1b[32mno\x1b[0m
> """
        health = runner.parse_health(text)
        dirty = runner.parse_dirty(text)
        self.assertEqual(health["driver_state"], "READY")
        self.assertTrue(health["online"])
        self.assertEqual(health["total_success"], 42)
        self.assertFalse(dirty["persistent_config_dirty"])
        self.assertFalse(dirty["resync_needed"])

    def test_parse_health_uses_latest_driver_block(self) -> None:
        text = """
=== Driver Health ===
  State: READY
  Online: yes
  Consecutive failures: 0
=== Driver Health ===
  State: OFFLINE
  Online: no
  Consecutive failures: 5
> """
        parsed = runner.parse_response("drv", text)
        self.assertEqual(parsed["driver_state"], "OFFLINE")
        self.assertFalse(parsed["online"])
        self.assertEqual(parsed["consecutive_failures"], 5)

    def test_dirty_state_fails_safe_run(self) -> None:
        text = """
=== Persistent Config Dirty State ===
  persistentConfigDirty: yes
  persistentConfigDirtyError: VERIFY_FAILED (code=8, detail=199)
  persistentConfigDirtyError message: verify mismatch
  resyncNeeded: yes
> """
        parsed = runner.parse_dirty(text)
        self.assertTrue(parsed["persistent_config_dirty"])
        self.assertEqual(parsed["persistent_config_dirty_error"]["name"], "VERIFY_FAILED")

    def test_validate_parsed_marks_dirty_and_stress_failures(self) -> None:
        dirty_spec = runner.CommandSpec("dirty", "dirty", validators=("dirty_clean",))
        dirty_failures, dirty_reviews = runner.validate_parsed(
            dirty_spec,
            {"persistent_config_dirty": True, "resync_needed": True},
        )
        self.assertIn("persistent config is dirty", dirty_failures)
        self.assertEqual(dirty_reviews, [])

        stress_spec = runner.CommandSpec("stress 50", "stress", validators=("stress",))
        stress_failures, stress_reviews = runner.validate_parsed(
            stress_spec,
            {"stress": {"kind": "stress", "total": 50, "success": 49, "errors": 1}},
        )
        self.assertTrue(any("stress errors=1" in item for item in stress_failures))
        self.assertEqual(stress_reviews, [])

    def test_expected_failure_ok_is_hard_failure(self) -> None:
        spec = runner.CommandSpec("read", "fault read", expected_any=("Status:",), validators=("expected_failure",))
        parsed = runner.parse_response("read", "  Status: OK (code=0, detail=0)\n")
        result, reason = runner.classify_response(spec, "  Status: OK (code=0, detail=0)\n", False, parsed)

        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("command reported OK during operator fault step", reason)

    def test_fault_health_ready_is_hard_failure(self) -> None:
        spec = runner.CommandSpec("drv", "fault health", expected_any=("Driver Health",), validators=("health_faulted",))
        text = """
=== Driver Health ===
  State: READY
  Online: yes
  Consecutive failures: 0
> """
        parsed = runner.parse_response("drv", text)
        result, reason = runner.classify_response(spec, text, False, parsed)

        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("driver health did not reflect induced fault", reason)

    def test_persistent_readbacks_compare_expected_values(self) -> None:
        state = {
            "expected_measurement_interval_ds": 150,
            "expected_co2_offset_ppm": -12,
            "expected_co2_gain": 32768,
        }

        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("interval", "interval", validators=("interval_read", "interval_expected")),
            {"measurement_interval_ds": 151},
            state,
        )
        self.assertIn("measurement interval readback 151 != expected 150", failures)
        self.assertEqual(reviews, [])

        failures, _ = runner.validate_parsed(
            runner.CommandSpec("offset", "offset", validators=("offset_read", "offset_expected")),
            {"co2_offset_ppm": -11},
            state,
        )
        self.assertIn("CO2 offset readback -11 != expected -12", failures)

        failures, _ = runner.validate_parsed(
            runner.CommandSpec("gain", "gain", validators=("gain_read", "gain_expected")),
            {"co2_gain": 32769},
            state,
        )
        self.assertIn("CO2 gain readback 32769 != expected 32768", failures)

    def test_help_requires_header_token(self) -> None:
        parsed = runner.parse_response("help", "selftest\n")
        result, reason = runner.classify_response(
            runner.CommandSpec("help", "help", expected_any=("EE871-E2 CLI Help",)),
            "selftest\n",
            False,
            parsed,
        )

        self.assertEqual(runner.RESULT_OPERATOR, result)
        self.assertIn("expected output token missing", reason)

    def test_persistent_writes_require_exact_confirmation(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(["--dry-run", "--include-persistent-writes", "--confirm-persistent-writes", "yes"])
        args = runner.parse_args(
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
                runner.PERSISTENT_CONFIRM_TEXT,
            ]
        )
        self.assertTrue(args.include_persistent_writes)

        args = runner.parse_args(["--dry-run", "--include-persistent-writes", "--confirm-persistent-writes"])
        self.assertTrue(args.include_persistent_writes)

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--dry-run",
                        "--include-persistent-writes",
                        "--confirm-persistent-writes",
                        "--write-co2-offset",
                        "40000",
                    ]
                )

    def test_maintenance_writes_are_gated_by_clean_dirty_state(self) -> None:
        spec = runner.CommandSpec(
            "interval 150",
            "write interval",
            group="maintenance",
            destructive=True,
        )

        self.assertIsNotNone(runner.maintenance_write_block_reason(spec, {}))
        self.assertIsNotNone(
            runner.maintenance_write_block_reason(
                spec,
                {"persistent_config_dirty": True, "resync_needed": True},
            )
        )
        self.assertIsNone(
            runner.maintenance_write_block_reason(
                spec,
                {"persistent_config_dirty": False, "resync_needed": False},
            )
        )

    def test_record_persistent_write_expectation(self) -> None:
        state: dict[str, object] = {}
        runner.record_persistent_write_expectation(
            {"result": runner.RESULT_PASS, "destructive": True, "command": "interval 150"},
            state,
        )
        runner.record_persistent_write_expectation(
            {"result": runner.RESULT_PASS, "destructive": True, "command": "offset -15"},
            state,
        )
        runner.record_persistent_write_expectation(
            {"result": runner.RESULT_PASS, "destructive": True, "command": "gain 32768"},
            state,
        )

        self.assertEqual(150, state["expected_measurement_interval_ds"])
        self.assertEqual(-15, state["expected_co2_offset_ppm"])
        self.assertEqual(32768, state["expected_co2_gain"])

    def test_non_pass_verdicts_have_nonzero_exit_codes(self) -> None:
        self.assertEqual(0, runner.exit_code_for_verdict(runner.VERDICT_PASS))
        self.assertEqual(1, runner.exit_code_for_verdict(runner.VERDICT_FAIL))
        self.assertEqual(2, runner.exit_code_for_verdict(runner.VERDICT_OPERATOR))
        self.assertEqual(3, runner.exit_code_for_verdict(runner.VERDICT_INCOMPLETE))

    def test_metadata_reports_clean_worktree_for_empty_git_status(self) -> None:
        args = runner.parse_args(["--dry-run"])
        meta = runner.metadata(args, pathlib.Path("hil_logs/example"), "", "branch", "abcdef")

        self.assertEqual("clean", meta["git_worktree"])

    def test_startup_drain_waits_for_prompt_when_required(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.read_count = 0

            @property
            def in_waiting(self) -> int:
                return 0

            def read(self, _size: int) -> bytes:
                self.read_count += 1
                if self.read_count == 1:
                    return b"booting\r\n"
                if self.read_count < 5:
                    time.sleep(0.002)
                    return b""
                return b"> "

        text, reason, timed_out = runner.read_until_ready(
            FakeSerial(),
            timeout_s=0.2,
            idle_s=0.001,
            command=None,
            require_prompt=True,
        )

        self.assertFalse(timed_out)
        self.assertEqual("prompt", reason)
        self.assertIn("> ", text)


if __name__ == "__main__":
    unittest.main()
