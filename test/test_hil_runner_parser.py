from __future__ import annotations

import importlib.util
import contextlib
import io
import pathlib
import sys
import time
import types
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "ee871_hil_runner.py"
SPEC = importlib.util.spec_from_file_location("ee871_hil_runner", MODULE_PATH)
assert SPEC is not None
runner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)
SOAK_MODULE_PATH = ROOT / "tools" / "ee871_soak_runner.py"
SOAK_SPEC = importlib.util.spec_from_file_location(
    "ee871_soak_runner",
    SOAK_MODULE_PATH,
)
assert SOAK_SPEC is not None
soak = importlib.util.module_from_spec(SOAK_SPEC)
assert SOAK_SPEC.loader is not None
sys.modules[SOAK_SPEC.name] = soak
SOAK_SPEC.loader.exec_module(soak)
SERIAL_MODULE_PATH = ROOT / "tools" / "ee871_serial_discriminator.py"
SERIAL_SPEC = importlib.util.spec_from_file_location(
    "ee871_serial_discriminator",
    SERIAL_MODULE_PATH,
)
assert SERIAL_SPEC is not None
serial_discriminator = importlib.util.module_from_spec(SERIAL_SPEC)
assert SERIAL_SPEC.loader is not None
sys.modules[SERIAL_SPEC.name] = serial_discriminator
SERIAL_SPEC.loader.exec_module(serial_discriminator)


class HilRunnerParserTest(unittest.TestCase):
    def test_open_serial_deasserts_control_lines_before_open(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.dtr = True
                self.rts = True
                self.port = None
                self.baudrate = None
                self.timeout = None
                self.write_timeout = None
                self.open_snapshot = None

            def open(self) -> None:
                self.open_snapshot = (self.dtr, self.rts)

        fake_module = types.SimpleNamespace(Serial=FakeSerial)
        args = types.SimpleNamespace(port="COM20", baud=115200)
        with mock.patch.dict(sys.modules, {"serial": fake_module}):
            ser = runner.open_serial(args)

        self.assertEqual((False, False), ser.open_snapshot)
        self.assertEqual("COM20", ser.port)
        self.assertEqual(115200, ser.baudrate)

    def test_parse_version_records_framework_stack(self) -> None:
        text = """
=== Version Info ===
  Example firmware build: Jul 31 2026 12:00:00
  MCU: ESP32-S3 rev 2, flash 4194304 bytes, PSRAM ready (2097152 bytes)
  Arduino-ESP32: 3.3.11
  ESP-IDF: v5.5.5
  EE871 library version: 1.0.1
  EE871 library full: 1.0.1 (0123456, 2026-07-31 12:00:00, clean)
  EE871 library build: 2026-07-31 12:00:00
  EE871 library commit: 0123456 (clean)
> """
        parsed = runner.parse_response("version", text)

        self.assertEqual("3.3.11", parsed["arduino_esp32_version"])
        self.assertEqual("v5.5.5", parsed["esp_idf_version"])
        self.assertEqual("1.0.1", parsed["library_version"])

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
        parsed = runner.parse_response("resync", text)

        self.assertFalse(parsed["persistent_config_dirty"])
        self.assertFalse(parsed["resync_needed"])
        self.assertEqual("OK", parsed["persistent_config_dirty_error"]["name"])

    def test_missing_dirty_parse_requires_operator_review(self) -> None:
        text = "=== Persistent Config Dirty State ===\n"
        parsed = runner.parse_response("dirty", text)
        result, reason = runner.classify_response(
            runner.CommandSpec(
                "dirty",
                "dirty",
                expected_any=("Persistent Config Dirty State",),
                validators=("dirty_clean",),
            ),
            text,
            False,
            parsed,
        )

        self.assertEqual(runner.RESULT_OPERATOR, result)
        self.assertIn("persistent dirty flag not parsed", reason)

    def test_aggregate_verdict_rules(self) -> None:
        self.assertEqual(runner.VERDICT_INCOMPLETE, runner.verdict([], dry_run=True))
        self.assertEqual(runner.VERDICT_PASS, runner.verdict([{"result": runner.RESULT_PASS}], dry_run=False))
        self.assertEqual(runner.VERDICT_FAIL, runner.verdict([{"result": runner.RESULT_FAIL}], dry_run=False))
        self.assertEqual(
            runner.VERDICT_OPERATOR,
            runner.verdict([{"result": runner.RESULT_OPERATOR}], dry_run=False),
        )
        self.assertEqual(
            runner.VERDICT_INCOMPLETE,
            runner.verdict([{"result": runner.RESULT_SKIP}], dry_run=False),
        )

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

    def test_expected_all_tokens_and_exact_status_are_enforced(self) -> None:
        spec = runner.CommandSpec(
            "mode",
            "unsupported mode read",
            expected_all=("Status:", "Message:"),
            expected_status="NOT_SUPPORTED",
            validators=("status_expected",),
        )
        good_text = (
            "Status: NOT_SUPPORTED (code=13, detail=0)\n"
            "Message: Operating mode not supported\n"
        )
        parsed = runner.parse_response("mode", good_text)
        result, reason = runner.classify_response(spec, good_text, False, parsed)
        self.assertEqual(runner.RESULT_PASS, result)
        self.assertEqual("", reason)

        wrong_text = "Status: OUT_OF_RANGE (code=12, detail=85)\nMessage: invalid\n"
        result, reason = runner.classify_response(
            spec,
            wrong_text,
            False,
            runner.parse_response("mode", wrong_text),
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("expected NOT_SUPPORTED", reason)

        missing_text = "Status: NOT_SUPPORTED (code=13, detail=0)\n"
        result, reason = runner.classify_response(
            spec,
            missing_text,
            False,
            runner.parse_response("mode", missing_text),
        )
        self.assertEqual(runner.RESULT_OPERATOR, result)
        self.assertIn("expected output token missing", reason)

    def test_niche_plan_is_fixed_and_has_no_persistent_write(self) -> None:
        args = runner.parse_args(["--dry-run", "--include-niche"])
        plan = runner.build_plan(args)
        niche = [spec for spec in plan if spec.group.startswith("niche-")]

        self.assertGreaterEqual(len(niche), 35)
        self.assertTrue(any(spec.command == "stress_mix 500" for spec in niche))
        self.assertTrue(any(spec.command == "diag" for spec in niche))
        self.assertFalse(any(spec.destructive for spec in niche))

    def test_persistent_read_completion_waits_for_value_line(self) -> None:
        self.assertFalse(runner.response_has_completion("interval", "  Status: OK\n"))
        self.assertFalse(runner.response_has_completion("offset", "  Status: OK\n"))
        self.assertFalse(runner.response_has_completion("gain", "  Status: OK\n"))
        self.assertFalse(runner.response_has_completion("addr", "  Status: OK\n"))

        self.assertTrue(
            runner.response_has_completion(
                "interval",
                "  Status: OK\n  Interval: 150 deciseconds (15.0 s)\n",
            )
        )
        self.assertTrue(
            runner.response_has_completion(
                "offset",
                "  Status: OK\n  CO2 offset: 0 ppm\n",
            )
        )
        self.assertTrue(
            runner.response_has_completion(
                "gain",
                "  Status: OK\n  CO2 gain: 32768 (factor=1.0000)\n",
            )
        )
        self.assertTrue(
            runner.response_has_completion(
                "addr",
                "  Status: OK\n  Bus address: 0\n",
            )
        )

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
                return b"> \r\n"

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

    def test_command_prompt_requirement_prevents_response_shift(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.chunks = [
                    b"  Status: OK\n  CO2 offset: 0 ppm\n",
                    b"",
                    b"",
                    b"> \r\n",
                ]

            @property
            def in_waiting(self) -> int:
                return 0

            def read(self, _size: int) -> bytes:
                if not self.chunks:
                    return b""
                chunk = self.chunks.pop(0)
                if not chunk:
                    time.sleep(0.002)
                return chunk

        text, reason, timed_out = runner.read_until_ready(
            FakeSerial(),
            timeout_s=0.2,
            idle_s=0.001,
            command="offset",
            require_prompt=True,
        )

        self.assertFalse(timed_out)
        self.assertEqual("prompt", reason)
        self.assertIn("CO2 offset: 0 ppm", text)
        self.assertIn("> ", text)

    def test_prompt_without_value_line_times_out(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.chunks = [b"> \r\n"]

            @property
            def in_waiting(self) -> int:
                return 0

            def read(self, _size: int) -> bytes:
                if self.chunks:
                    return self.chunks.pop(0)
                time.sleep(0.001)
                return b""

        text, reason, timed_out = runner.read_until_ready(
            FakeSerial(),
            timeout_s=0.02,
            idle_s=0.001,
            command="offset",
            require_prompt=True,
        )

        self.assertTrue(timed_out)
        self.assertEqual("timeout", reason)
        self.assertEqual("> \r\n", text)

    def test_prompt_framed_value_nack_completes_without_timeout(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.chunks = [
                    b"  Status: NACK (code=8, detail=0)\r\n",
                    b"  Message: Control byte NACK\r\n> \r\n",
                ]

            @property
            def in_waiting(self) -> int:
                return 0

            def read(self, _size: int) -> bytes:
                if self.chunks:
                    return self.chunks.pop(0)
                time.sleep(0.001)
                return b""

        text, reason, timed_out = runner.read_until_ready(
            FakeSerial(),
            timeout_s=0.1,
            idle_s=0.001,
            command="interval",
            require_prompt=True,
        )

        self.assertFalse(timed_out)
        self.assertEqual("prompt", reason)
        self.assertIn("Status: NACK", text)

    def test_prompt_waits_for_terminating_newline(self) -> None:
        class FakeSerial:
            def __init__(self) -> None:
                self.chunks = [
                    b"  Status: OK\n  CO2 offset: 0 ppm\n> ",
                    b"",
                    b"\r\n",
                ]

            @property
            def in_waiting(self) -> int:
                return 0

            def read(self, _size: int) -> bytes:
                if not self.chunks:
                    return b""
                chunk = self.chunks.pop(0)
                if not chunk:
                    time.sleep(0.002)
                return chunk

        text, reason, timed_out = runner.read_until_ready(
            FakeSerial(),
            timeout_s=0.2,
            idle_s=0.001,
            command="offset",
            require_prompt=True,
        )

        self.assertFalse(timed_out)
        self.assertEqual("prompt", reason)
        self.assertTrue(text.endswith("> \r\n"))

    def test_non_ok_status_is_failure_before_missing_value_review(self) -> None:
        spec = runner.CommandSpec(
            "co2fast",
            "Read MV3.",
            expected_any=("CO2 fast:",),
            validators=("status_ok",),
        )
        text = (
            "  Status: NACK (code=8, detail=0)\r\n"
            "  Message: Control byte NACK\r\n"
            "> \r\n"
        )
        parsed = runner.parse_response("co2fast", text)

        result, reason = runner.classify_response(
            spec,
            text,
            False,
            parsed,
        )

        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertEqual("status is NACK", reason)

    def test_soak_recognizes_only_scheduled_sample_control_nack(self) -> None:
        spec = runner.CommandSpec(
            "co2fast",
            "Read MV3.",
            group="soak-sample",
            expected_any=("CO2 fast:",),
            validators=("status_ok",),
        )
        row = {
            "command": "co2fast",
            "result": runner.RESULT_FAIL,
            "raw": (
                "Status: NACK (code=8, detail=0)\r\n"
                "Message: Control byte NACK\r\n> \r\n"
            ),
            "parsed": {"status": {"name": "NACK", "code": 8, "detail": 0}},
        }

        self.assertTrue(soak.is_scheduled_control_nack(spec, row))

        wrong_group = runner.CommandSpec(
            "co2fast",
            "Read MV3.",
            group="soak-final",
        )
        self.assertFalse(soak.is_scheduled_control_nack(wrong_group, row))

        wrong_failure = dict(row)
        wrong_failure["raw"] = "Status: NACK (code=8, detail=0)\r\n> \r\n"
        self.assertFalse(soak.is_scheduled_control_nack(spec, wrong_failure))

    def test_soak_counts_scheduled_control_nack_recovery_separately(self) -> None:
        counts = soak.aggregate_counts(
            [
                {"result": runner.RESULT_PASS},
                {"result": soak.RESULT_SCHEDULED_CONTROL_NACK_RECOVERED},
            ]
        )

        self.assertEqual(1, counts[runner.RESULT_PASS])
        self.assertEqual(1, counts[soak.RESULT_SCHEDULED_CONTROL_NACK_RECOVERED])

    def test_soak_compaction_keeps_only_abnormal_excerpt(self) -> None:
        passing = soak.compact_row(
            {
                "command": "dirty",
                "result": runner.RESULT_PASS,
                "raw": "full reply",
                "clean_excerpt": "full reply",
            }
        )
        self.assertNotIn("raw", passing)
        self.assertNotIn("clean_excerpt", passing)
        self.assertNotIn("failure_excerpt", passing)

        failing = soak.compact_row(
            {
                "command": "co2fast",
                "result": runner.RESULT_FAIL,
                "raw": "Status: NACK",
                "clean_excerpt": "Status: NACK",
            }
        )
        self.assertNotIn("raw", failing)
        self.assertNotIn("clean_excerpt", failing)
        self.assertEqual("Status: NACK", failing["failure_excerpt"])

    def test_soak_default_scheduled_nack_retry_is_bounded(self) -> None:
        args = soak.parse_args(["--port", "COM20"])

        self.assertEqual(1500, args.scheduled_nack_retry_ms)

    def test_serial_discriminator_checks_exact_requested_stack(self) -> None:
        args = serial_discriminator.parse_args(
            [
                "--port",
                "COM20",
                "--run-dir",
                "unused",
                "--expected-arduino-version",
                "3.3.11",
                "--expected-idf-version",
                "v5.5.5",
                "--expected-library-version",
                "1.0.1",
            ]
        )
        state = {
            "arduino_esp32_version": "3.3.11",
            "esp_idf_version": "v5.5.5",
            "library_version": "1.0.1",
        }
        self.assertEqual(
            [],
            serial_discriminator.expected_version_failures(state, args),
        )
        state["esp_idf_version"] = "v5.5.4"
        self.assertIn(
            "ESP-IDF 'v5.5.4' != expected 'v5.5.5'",
            serial_discriminator.expected_version_failures(state, args),
        )

if __name__ == "__main__":
    unittest.main()
