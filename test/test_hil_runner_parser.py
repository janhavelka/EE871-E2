from __future__ import annotations

import importlib.util
import contextlib
import io
import json
import pathlib
import sys
import tempfile
import time
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
                {
                    "baseline_custom_memory_complete": True,
                    "baseline_complete": True,
                    "persistent_config_dirty": False,
                    "resync_needed": False,
                    "mutation_epoch": 0,
                    "dirty_observation_epoch": 0,
                },
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
        self.assertEqual(0x5A, state["expected_mutation_first_address"])
        self.assertEqual(0x5B, state["expected_mutation_last_address"])

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

    def test_checked_sample_parser_and_semantic_validation(self) -> None:
        text = """
  Status: OK (code=0, detail=0)
  Sample kind: FAST (MV3)
  Value step: attempted=yes status=OK detail=0
  Value step message: <none>
  CO2 value: 612 ppm, valid=yes
  Status step: attempted=yes status=OK detail=0 valid=yes, byte=0x00, co2Error=no
  Status step message: <none>
  Error-code step: attempted=no status=OK detail=0 valid=no
  Error-code step message: <none>
  Sensor error: NONE (enum=0)
> """
        parsed = runner.parse_response("samplefast", text)
        self.assertEqual("FAST", parsed["checked_sample_kind"])
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("samplefast", "checked", validators=("checked_fast",)),
            parsed,
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

        sensor_error = text.replace(
            "Status: OK (code=0, detail=0)",
            "Status: CO2_SENSOR_ERROR (code=15, detail=200)",
            1,
        ).replace(
            "byte=0x00, co2Error=no",
            "byte=0x08, co2Error=yes",
        ).replace(
            "attempted=no status=OK detail=0 valid=no",
            "attempted=yes status=OK detail=0 valid=yes, code=200 (sensor counts low)",
        ).replace(
            "Sensor error: NONE (enum=0)",
            "Sensor error: SENSOR_COUNTS_LOW (enum=200)",
        )
        parsed = runner.parse_response("samplefast", sensor_error)
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("samplefast", "checked", validators=("checked_fast",)),
            parsed,
        )
        self.assertTrue(any("sensor reported SENSOR_COUNTS_LOW" in item for item in failures))
        self.assertEqual([], reviews)

    def test_checked_sample_kind_mismatch_and_missing_fields_are_not_pass(self) -> None:
        parsed = runner.parse_response(
            "sampleavg",
            "Sample kind: FAST (MV3)\nValue step: attempted=yes status=OK detail=0\n",
        )
        result, reason = runner.classify_response(
            runner.CommandSpec(
                "sampleavg",
                "checked",
                expected_any=("Sample kind:",),
                validators=("checked_average",),
            ),
            "Sample kind: FAST (MV3)\nValue step: attempted=yes status=OK detail=0\n",
            False,
            parsed,
        )
        self.assertEqual(runner.RESULT_OPERATOR, result)
        self.assertIn("checked sample fields not parsed", reason)

        transport_failure = """
Status: TIMEOUT (code=4, detail=25000)
Sample kind: AVERAGE (MV4)
Value step: attempted=yes status=TIMEOUT detail=25000
CO2 value: 0 ppm, valid=no
Status step: attempted=no status=OK detail=0 valid=no
Error-code step: attempted=no status=OK detail=0 valid=no
Sensor error: NONE (enum=0)
"""
        parsed = runner.parse_response("sampleavg", transport_failure)
        result, reason = runner.classify_response(
            runner.CommandSpec(
                "sampleavg",
                "checked",
                expected_any=("Sample kind:",),
                validators=("checked_average",),
            ),
            transport_failure,
            False,
            parsed,
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("checked value status is TIMEOUT", reason)

    def test_parse_complete_mutation_diagnostic_and_verify_counts(self) -> None:
        text = """
  persistentConfigDirty: no
  persistentConfigDirtyError: OK (code=0, detail=0)
  resyncNeeded: no
  mutation.unresolved: no
  mutation.target: GLOBAL_INTERVAL (value=4)
  mutation.effect: VERIFIED (value=4)
  mutation.addresses: first=0xC6 last=0xC7
  mutation.elements: requested=2 acknowledged=2 observed=2 matched=2
  mutation.attemptedValue: 0x00
  mutation.preObservedValue: valid=no value=0x00
  mutation.observedValue: valid=yes value=0x00
  mutation.cause: OK (code=0, detail=0)
"""
        parsed = runner.parse_dirty(text)
        self.assertEqual("GLOBAL_INTERVAL", parsed["mutation_target"])
        self.assertEqual(2, parsed["mutation_elements_matched"])
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("dirty", "dirty", validators=("dirty_clean", "mutation_verified")),
            parsed,
            {
                "expected_mutation_target": "GLOBAL_INTERVAL",
                "expected_mutation_count": 2,
                "expected_mutation_first_address": 0xC6,
                "expected_mutation_last_address": 0xC7,
            },
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

    def test_custom_memory_dump_requires_exact_256_byte_coverage(self) -> None:
        rows = []
        for start in range(0, 256, 16):
            values = " ".join(f"{(start + offset) & 0xFF:02X}" for offset in range(16))
            rows.append(f"  0x{start:02X}: {values}")
        text = "=== Custom Register Dump ===\n" + "\n".join(rows)
        parsed = runner.parse_custom_dump(text)
        self.assertTrue(parsed["custom_memory_complete"])
        self.assertEqual(list(range(256)), parsed["custom_memory"])

        incomplete = runner.parse_custom_dump("\n".join(rows[:-1]))
        self.assertFalse(incomplete["custom_memory_complete"])
        self.assertEqual(16, len(incomplete["custom_memory_missing"]))

        duplicate = runner.parse_custom_dump(text + "\n" + rows[0])
        self.assertFalse(duplicate["custom_memory_complete"])
        self.assertTrue(duplicate["custom_memory_errors"])

    def test_extended_plan_covers_prompt04_safe_surface(self) -> None:
        commands = [spec.command for spec in runner.extended_specs(1, 1)]
        for command in (
            "buscheck",
            "levels",
            "status",
            "co2fast",
            "co2avg",
            "samplefast",
            "sampleavg",
            "features",
            "caps",
            "fw",
            "e2spec",
            "stress_mix 100",
            "resync",
        ):
            self.assertIn(command, commands)
        self.assertFalse(any(spec.destructive for spec in runner.extended_specs(1, 1)))
        for sample_command in ("samplefast", "sampleavg"):
            index = commands.index(sample_command)
            self.assertEqual("drv", commands[index - 1])
            self.assertEqual("drv", commands[index + 1])

    def test_checked_sample_health_counter_comparison(self) -> None:
        spec = runner.CommandSpec(
            "drv",
            "health",
            validators=("health_failures_unchanged",),
        )
        failures, reviews = runner.validate_parsed(
            spec,
            {"total_failures": 3},
            {"total_failures": 3},
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)
        failures, _ = runner.validate_parsed(
            spec,
            {"total_failures": 4},
            {"total_failures": 3},
        )
        self.assertTrue(any("3 -> 4" in failure for failure in failures))

    def test_maintenance_plan_snapshots_then_restores_typed_values(self) -> None:
        args = runner.parse_args(
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
                "--maintenance-interval",
                "160",
                "--write-interval-factor",
                "2",
            ]
        )
        plan = runner.maintenance_specs(args)
        commands = [spec.command for spec in plan]
        first_write = next(index for index, spec in enumerate(plan) if spec.destructive)
        self.assertLess(commands.index("reg dump 0 256"), first_write)
        self.assertIn("interval <recorded-baseline>", commands)
        self.assertIn("factor <recorded-baseline>", commands)
        self.assertEqual("reg dump 0 256", commands[-2])
        self.assertTrue(all("reg write" not in command for command in commands))

    def test_baseline_capture_is_immutable_and_dynamic_restore_uses_it(self) -> None:
        state: dict[str, object] = {"mutation_epoch": 0}
        spec = runner.CommandSpec(
            "interval",
            "baseline",
            capture=("measurement_interval_ds",),
        )
        row = runner.result_row(
            spec,
            "interval",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {"measurement_interval_ds": 150},
        )
        runner.update_state(state, row)
        row["parsed"] = {"measurement_interval_ds": 160}
        runner.update_state(state, row)
        self.assertEqual(150, state["baseline_measurement_interval_ds"])
        command, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "interval <recorded-baseline>",
                "restore",
                dynamic="interval_baseline",
            ),
            state,
        )
        self.assertEqual("interval 150", command)
        self.assertIsNone(reason)

    def test_failed_destructive_command_latches_later_writes(self) -> None:
        state: dict[str, object] = {
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
            "baseline_custom_memory_complete": True,
            "persistent_config_dirty": False,
            "resync_needed": False,
        }
        failed = {
            "result": runner.RESULT_FAIL,
            "destructive": True,
            "command": "interval 160",
            "group": "maintenance-interval",
        }
        runner.record_persistent_write_expectation(failed, state)
        reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "interval 150",
                "restore",
                group="maintenance-interval",
                destructive=True,
            ),
            state,
        )
        self.assertIn("earlier destructive command failed", reason or "")

    def test_restore_requires_fresh_verified_diagnostic_after_test_write(self) -> None:
        state: dict[str, object] = {
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
            "baseline_custom_memory_complete": True,
            "baseline_complete": True,
            "baseline_measurement_interval_ds": 150,
            "persistent_config_dirty": False,
            "resync_needed": False,
        }
        test_write = {
            "result": runner.RESULT_PASS,
            "destructive": True,
            "command": "interval 160",
            "group": "maintenance-interval",
        }
        runner.record_persistent_write_expectation(test_write, state)
        restore = runner.CommandSpec(
            "interval <recorded-baseline>",
            "restore",
            group="maintenance-interval",
            destructive=True,
            dynamic="interval_baseline",
        )
        self.assertIn(
            "no fresh successful dirty diagnostic",
            runner.maintenance_write_block_reason(restore, state) or "",
        )

        diagnostic = {
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_unresolved": False,
            "mutation_target": "GLOBAL_INTERVAL",
            "mutation_effect": "VERIFIED",
            "mutation_first_address": 0xC6,
            "mutation_last_address": 0xC7,
            "mutation_elements_requested": 2,
            "mutation_elements_acknowledged": 2,
            "mutation_elements_observed": 2,
            "mutation_elements_matched": 2,
            "mutation_cause": {"name": "OK", "code": 0, "detail": 0},
        }
        spec = runner.CommandSpec(
            "dirty",
            "verified",
            group="maintenance-interval",
            validators=("dirty_clean", "mutation_verified"),
        )
        failures, reviews = runner.validate_parsed(spec, diagnostic, state)
        self.assertEqual([], failures)
        self.assertEqual([], reviews)
        row = runner.result_row(
            spec,
            "dirty",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            diagnostic,
        )
        runner.update_state(state, row)
        self.assertIsNone(runner.maintenance_write_block_reason(restore, state))

    def test_custom_memory_diff_allows_only_documented_volatile_addresses(self) -> None:
        baseline = [0] * 256
        current = baseline.copy()
        current[0xC1] = 1
        current[0xD9] = 1
        expected, unexpected = runner.custom_memory_diff(baseline, current)
        self.assertEqual({0xC1, 0xD9}, {item["address"] for item in expected})
        self.assertEqual([], unexpected)
        current[0xC6] = 1
        _, unexpected = runner.custom_memory_diff(baseline, current)
        self.assertEqual(0xC6, unexpected[0]["address"])

    def test_hazardous_plans_require_separate_exact_confirmations(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(["--dry-run", "--include-auto-adjust"])
            with self.assertRaises(SystemExit):
                runner.parse_args(["--dry-run", "--include-stuck-line"])
            with self.assertRaises(SystemExit):
                runner.parse_args(["--dry-run", "--include-power-cycle"])
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--dry-run",
                        "--include-address-change",
                        "--candidate-address",
                        "1",
                        "--confirm-address-change",
                    ]
                )
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--dry-run",
                        "--include-persistent-writes",
                        "--confirm-persistent-writes",
                        "--write-co2-offset",
                        "1",
                    ]
                )
        address_args = runner.parse_args(
            [
                "--dry-run",
                "--include-address-change",
                "--candidate-address",
                "1",
                "--confirm-address-change",
                "--confirm-address-restore",
            ]
        )
        commands = [spec.command for spec in runner.address_change_specs(address_args)]
        self.assertFalse(any(command == "scan" for command in commands))
        self.assertIn("addr rebegin 1", commands)
        self.assertIn("addr rebegin <recorded-baseline>", commands)

    def test_live_hazardous_plan_requires_structured_metadata(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--port",
                        "COM1",
                        "--include-stuck-line",
                        "--confirm-stuck-line",
                    ]
                )

    def test_calibration_options_cannot_mix_with_configuration_targets(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--dry-run",
                        "--include-persistent-writes",
                        "--confirm-persistent-writes",
                        "--include-calibration-writes",
                        "--confirm-calibration-writes",
                        "--write-co2-offset",
                        "1",
                        "--write-co2-filter",
                        "2",
                    ]
                )

    def test_auto_adjust_plan_starts_once_and_never_restores_or_replays(self) -> None:
        args = runner.parse_args(
            ["--dry-run", "--include-auto-adjust", "--confirm-auto-adjust"]
        )
        plan = runner.auto_adjust_specs(args)
        commands = [spec.command for spec in plan]
        self.assertEqual(1, commands.count("autoadj start"))
        self.assertEqual(["autoadj start"], [spec.command for spec in plan if spec.destructive])

    def test_checkpoint_writes_forensic_baseline_without_replay_commands(self) -> None:
        args = runner.parse_args(["--dry-run"])
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = pathlib.Path(tmp)
            meta = runner.metadata(args, log_dir, "", "branch", "abcdef")
            state = {"baseline_custom_memory": list(range(256))}
            runner.write_checkpoint(log_dir, meta, "", [], state)
            self.assertTrue((log_dir / "checkpoint.json").exists())
            self.assertTrue((log_dir / "custom_memory_baseline.json").exists())
            baseline_hex = (log_dir / "custom_memory_baseline.hex").read_text(encoding="utf-8")
            self.assertIn("0xF0:", baseline_hex)
            self.assertNotIn("reg write", baseline_hex)

    def test_resync_parser_uses_final_after_state(self) -> None:
        text = """
=== Persistent Config Resync ===
Before:
  persistentConfigDirty: yes
  persistentConfigDirtyError: E2_ERROR (code=3, detail=17)
  resyncNeeded: yes
After:
  persistentConfigDirty: no
  persistentConfigDirtyError: OK (code=0, detail=0)
  resyncNeeded: no
"""
        parsed = runner.parse_response("resync", text)
        self.assertFalse(parsed["persistent_config_dirty"])
        self.assertFalse(parsed["resync_needed"])
        self.assertEqual("OK", parsed["persistent_config_dirty_error"]["name"])

    def test_missing_dirty_fields_require_review_and_verdict_precedence(self) -> None:
        spec = runner.CommandSpec(
            "dirty",
            "dirty",
            expected_any=("Persistent Config Dirty State",),
            validators=("dirty_clean",),
        )
        text = "=== Persistent Config Dirty State ===\n"
        parsed = runner.parse_response("dirty", text)
        result, reason = runner.classify_response(spec, text, False, parsed)
        self.assertEqual(runner.RESULT_OPERATOR, result)
        self.assertIn("persistent dirty flag not parsed", reason)
        self.assertEqual(
            runner.VERDICT_FAIL,
            runner.verdict(
                [
                    {"result": runner.RESULT_OPERATOR},
                    {"result": runner.RESULT_FAIL},
                ],
                dry_run=False,
            ),
        )

    def test_checked_sample_rejects_internally_inconsistent_evidence(self) -> None:
        text = """
Status: OK (code=0, detail=0)
Sample kind: FAST (MV4)
Value step: attempted=yes status=OK detail=0
CO2 value: 612 ppm, valid=yes
Status step: attempted=yes status=OK detail=0 valid=yes, byte=0x08, co2Error=no
Error-code step: attempted=no status=OK detail=0 valid=no
Sensor error: NONE (enum=0)
"""
        parsed = runner.parse_response("samplefast", text)
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("samplefast", "checked", validators=("checked_fast",)),
            parsed,
        )
        self.assertTrue(any("MV4 != MV3" in failure for failure in failures))
        self.assertTrue(any("bit3 disagrees" in failure for failure in failures))
        self.assertEqual([], reviews)

    def test_dirty_state_requires_complete_mutation_evidence(self) -> None:
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("dirty", "dirty", validators=("dirty_state",)),
            {
                "persistent_config_dirty": False,
                "persistent_config_dirty_error": {"name": "OK", "code": 0, "detail": 0},
                "resync_needed": False,
            },
        )
        self.assertEqual([], failures)
        self.assertTrue(any("complete mutation diagnostic" in review for review in reviews))

    def test_post_test_snapshot_rejects_wrong_register_change(self) -> None:
        baseline = [0] * 256
        current = baseline.copy()
        current[0xC6] = 1
        current[0xA5] = 2
        parsed = {
            "custom_memory": current,
            "custom_memory_complete": True,
        }
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "reg dump 0 256",
                "post-test",
                validators=("custom_memory_target_only",),
            ),
            parsed,
            {
                "baseline_custom_memory": baseline,
                "expected_mutation_first_address": 0xC6,
                "expected_mutation_last_address": 0xC7,
            },
        )
        self.assertTrue(any("0xA5" in failure for failure in failures))
        self.assertEqual([], reviews)

    def test_failed_baseline_preflight_blocks_first_write(self) -> None:
        state: dict[str, object] = {
            "baseline_custom_memory_complete": True,
            "baseline_measurement_interval_ds": 150,
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
        }
        failed = runner.result_row(
            runner.CommandSpec("serial", "baseline", group="maintenance-baseline"),
            "serial",
            runner.RESULT_FAIL,
            "failed",
            0.1,
            "",
            "test",
            {},
        )
        runner.update_state(state, failed)
        terminal = runner.result_row(
            runner.CommandSpec("autoadj", "baseline", group="maintenance-baseline"),
            "autoadj",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {},
        )
        runner.update_state(state, terminal)
        reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "interval 160",
                "write",
                group="maintenance-interval",
                destructive=True,
            ),
            state,
        )
        self.assertIn("baseline preflight did not pass", reason or "")

    def test_unrestorable_mode_baseline_blocks_mode_write(self) -> None:
        state = {
            "baseline_complete": True,
            "baseline_custom_memory_complete": True,
            "baseline_operating_mode": 0x55,
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
        }
        reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "mode 1",
                "write mode",
                group="maintenance-mode",
                destructive=True,
            ),
            state,
        )
        self.assertIn("outside the typed restore range", reason or "")

    def test_unrestorable_interval_and_address_baselines_block_first_write(self) -> None:
        common = {
            "baseline_complete": True,
            "baseline_custom_memory_complete": True,
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
        }
        interval_reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "interval 160",
                "write interval",
                group="maintenance-interval",
                destructive=True,
            ),
            {
                **common,
                "baseline_measurement_interval_ds": 42,
            },
        )
        self.assertIn("restore range 150..36000", interval_reason or "")
        address_reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "addr 1",
                "write address",
                group="address-candidate",
                destructive=True,
            ),
            {
                **common,
                "baseline_device_address": 9,
                "candidate_address": 1,
            },
        )
        self.assertIn("restore range 0..7", address_reason or "")

    def test_no_op_test_value_is_blocked_before_write(self) -> None:
        state = {
            "baseline_complete": True,
            "baseline_custom_memory_complete": True,
            "baseline_measurement_interval_ds": 150,
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
        }
        reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "interval 150",
                "write interval",
                group="maintenance-interval",
                destructive=True,
            ),
            state,
        )
        self.assertIn("equals the recorded baseline", reason or "")

    def test_address_and_auto_adjust_phase_gates_fail_closed(self) -> None:
        state = {
            "mutation_unresolved": True,
            "mutation_target": "BUS_ADDRESS",
            "mutation_attempted_value": 1,
            "candidate_address": 1,
        }
        _, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "operator: activate address candidate",
                "activate",
                dynamic="address_candidate_diagnostic_passed",
            ),
            state,
        )
        self.assertIn("diagnostic did not pass", reason or "")

        _, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "autoadj start",
                "start",
                dynamic="auto_adjust_authorized",
            ),
            {"baseline_auto_adjust_running": False},
        )
        self.assertIn("not explicitly confirmed", reason or "")

        failed_row = runner.result_row(
            runner.CommandSpec("dirty", "dirty", group="address-candidate"),
            "dirty",
            runner.RESULT_FAIL,
            "bad diagnostic",
            0.1,
            "",
            "test",
            {},
        )
        state["address_change_started"] = True
        runner.update_state(state, failed_row)
        blocked = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "operator: activate address candidate",
                "activate",
                group="address-candidate",
                operator_required=True,
            ),
            state,
        )
        self.assertIn("address-workflow step failed", blocked or "")

    def test_unconfirmed_operator_transition_aborts_plan(self) -> None:
        spec = runner.CommandSpec(
            "operator: release SDA-low jig",
            "release",
            operator_required=True,
        )
        with contextlib.redirect_stdout(io.StringIO()):
            with mock.patch("builtins.input", return_value="skip"):
                with self.assertRaises(KeyboardInterrupt):
                    runner.run_operator_step(spec)
            with mock.patch("builtins.input", return_value="wrong"):
                with self.assertRaises(KeyboardInterrupt):
                    runner.run_operator_step(spec)

    def test_destructive_in_flight_journal_survives_serial_exception(self) -> None:
        class FailingSerial:
            def write(self, _data: bytes) -> None:
                raise OSError("serial link failed")

        args = runner.parse_args(["--dry-run"])
        spec = runner.CommandSpec(
            "interval 160",
            "write",
            group="maintenance-interval",
            destructive=True,
        )
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = pathlib.Path(tmp)
            meta = runner.metadata(args, log_dir, "", "branch", "abcdef")
            state: dict[str, object] = {}
            runner.journal_destructive_start(state, spec, "interval 160")
            runner.write_checkpoint(log_dir, meta, "", [], state)
            with self.assertRaises(OSError):
                runner.run_serial_command(FailingSerial(), spec, "interval 160", args, state)
            checkpoint = json.loads(
                (log_dir / "checkpoint.json").read_text(encoding="utf-8")
            )
            self.assertEqual(
                "interval 160",
                checkpoint["in_flight_destructive"]["command"],
            )

    def test_main_serial_exception_emits_fail_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output_dir = pathlib.Path(tmp)
            with mock.patch.object(
                runner,
                "open_serial",
                side_effect=OSError("serial unavailable"),
            ):
                with contextlib.redirect_stdout(io.StringIO()):
                    result = runner.main(
                        [
                            "--port",
                            "COM1",
                            "--output-dir",
                            str(output_dir),
                        ]
                    )
            self.assertEqual(1, result)
            summaries = list(output_dir.glob("*/summary.json"))
            self.assertEqual(1, len(summaries))
            summary = json.loads(summaries[0].read_text(encoding="utf-8"))
            self.assertEqual(runner.VERDICT_FAIL, summary["final_verdict"])
            self.assertEqual("runner error", summary["commands"][-1]["command"])
            self.assertEqual(
                "OSError",
                summary["parsed_state"]["runner_exception"]["type"],
            )

    def test_calibration_plan_does_not_add_interval_mutation(self) -> None:
        args = runner.parse_args(
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
                "--include-calibration-writes",
                "--confirm-calibration-writes",
                "--write-co2-offset",
                "1",
            ]
        )
        destructive = [
            spec.command for spec in runner.maintenance_specs(args) if spec.destructive
        ]
        self.assertTrue(all(not command.startswith("interval") for command in destructive))
        self.assertEqual(["offset 1", "offset <recorded-baseline>"], destructive)


if __name__ == "__main__":
    unittest.main()
