from __future__ import annotations

import importlib.util
import contextlib
import io
import json
import pathlib
import re
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
    def admitted_state(self, **overrides: object) -> dict[str, object]:
        state: dict[str, object] = {
            "baseline_complete": True,
            "baseline_custom_memory_complete": True,
            "baseline_custom_memory": [0] * 256,
            "baseline_custom_memory_captured_utc": "2026-07-29T10:00:00Z",
            "configured_device_address": 0,
            "baseline_device_address": 0,
            "baseline_auto_adjust_preflight_complete": True,
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_unresolved": False,
            "mutation_epoch": 0,
            "dirty_observation_epoch": 0,
        }
        state.update(overrides)
        return state

    def resolved_mutation(
        self,
        target: str,
        count: int,
        first: int,
        last: int,
        attempted: int,
    ) -> dict[str, object]:
        return {
            "persistent_config_dirty": False,
            "persistent_config_dirty_error": {
                "name": "OK",
                "code": 0,
                "detail": 0,
            },
            "resync_needed": False,
            "mutation_unresolved": False,
            "mutation_target": target,
            "mutation_target_value": 0,
            "mutation_effect": "VERIFIED",
            "mutation_effect_value": 4,
            "mutation_first_address": first,
            "mutation_last_address": last,
            "mutation_elements_requested": count,
            "mutation_elements_acknowledged": count,
            "mutation_elements_observed": count,
            "mutation_elements_matched": count,
            "mutation_attempted_value": attempted,
            "mutation_pre_observed_valid": False,
            "mutation_pre_observed_value": 0,
            "mutation_observed_valid": True,
            "mutation_observed_value": attempted,
            "mutation_cause": {
                "name": "OK",
                "code": 0,
                "detail": 0,
            },
        }

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
            {
                "persistent_config_dirty": True,
                "resync_needed": True,
                "mutation_unresolved": True,
                "persistent_config_dirty_error": {
                    "name": "VERIFY_MISMATCH",
                    "code": 15,
                    "detail": 1,
                },
            },
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
        self.assertIn("in-fault driver state is READY", reason)

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
                self.admitted_state(baseline_measurement_interval_ds=151),
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
            {"operating_functions": 0x80},
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

        sensor_error = text.replace(
            "Status: OK (code=0, detail=0)",
            "Status: CO2_SENSOR_ERROR (code=17, detail=200)",
            1,
        ).replace(
            "CO2 value: 612 ppm, valid=yes",
            "CO2 value: 612 ppm, valid=no",
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
            {"operating_functions": 0x80},
        )
        self.assertEqual(
            ["sensor reported SENSOR_COUNTS_LOW (code=200)"],
            failures,
        )
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
        state = self.admitted_state(baseline_measurement_interval_ds=150)
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
            "persistent_config_dirty_error": {
                "name": "OK",
                "code": 0,
                "detail": 0,
            },
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

    def test_restore_is_blocked_after_each_post_write_failure_or_uncertainty(self) -> None:
        restore = runner.CommandSpec(
            "factor <recorded-baseline>",
            "restore",
            group="maintenance-factor",
            destructive=True,
            dynamic="factor_baseline",
        )
        for planned_command, result in (
            ("factor", runner.RESULT_FAIL),
            ("dirty", runner.RESULT_FAIL),
            ("reg dump 0 256", runner.RESULT_FAIL),
            ("factor", runner.RESULT_OPERATOR),
            ("dirty", runner.RESULT_OPERATOR),
            ("reg dump 0 256", runner.RESULT_OPERATOR),
        ):
            state = self.admitted_state(baseline_co2_interval_factor=-1)
            runner.record_persistent_write_expectation(
                {
                    "result": runner.RESULT_PASS,
                    "destructive": True,
                    "command": "factor 1",
                    "group": "maintenance-factor",
                },
                state,
            )
            failed = runner.result_row(
                runner.CommandSpec(
                    planned_command,
                    "post-write verification",
                    group="maintenance-factor",
                ),
                planned_command,
                result,
                "failed or uncertain",
                0.1,
                "",
                "test",
                {},
            )
            runner.update_state(state, failed)
            self.assertIn(
                "maintenance verification step failed",
                runner.maintenance_write_block_reason(restore, state) or "",
            )

    def test_custom_memory_diff_allows_only_documented_volatile_addresses(self) -> None:
        baseline = [0] * 256
        current = baseline.copy()
        current[0xC1] = 1
        current[0xD9] = 1
        expected, unexpected = runner.custom_memory_diff(baseline, current)
        self.assertEqual({0xC1}, {item["address"] for item in expected})
        self.assertEqual({0xD9}, {item["address"] for item in unexpected})
        expected, unexpected = runner.custom_memory_diff(
            baseline,
            current,
            frozenset({0xD9}),
            "AUTO_ADJUST",
        )
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

    def test_filter_write_option_is_unavailable(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(
                    [
                        "--dry-run",
                        "--include-persistent-writes",
                        "--confirm-persistent-writes",
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
            state = {
                "baseline_custom_memory": list(range(256)),
                "baseline_custom_memory_captured_utc": "2026-07-29T10:00:00Z",
            }
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
        state = self.admitted_state(baseline_operating_mode=0x55)
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
        common = self.admitted_state()
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
                "configured_device_address": 9,
                "candidate_address": 1,
            },
        )
        self.assertIn("restore range 0..7", address_reason or "")

    def test_no_op_test_value_is_blocked_before_write(self) -> None:
        state = self.admitted_state(baseline_measurement_interval_ds=150)
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
            {"auto_adjust_fresh_idle_passed": True},
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

    def test_auto_adjust_requires_the_fresh_pre_action_idle_result(self) -> None:
        pre_action = runner.CommandSpec(
            "autoadj",
            "Require auto-adjust to be idle before the one-shot request.",
            group="auto-adjust",
        )
        running = runner.result_row(
            pre_action,
            "autoadj",
            runner.RESULT_FAIL,
            "auto-adjust running",
            0.1,
            "",
            "test",
            {"auto_adjust_running": True},
        )
        state = self.admitted_state(
            baseline_auto_adjust_running=False,
            auto_adjust_authorized=True,
        )
        runner.update_state(state, running)
        self.assertIs(
            False,
            state["auto_adjust_fresh_idle_passed"],
        )
        _, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "autoadj start",
                "start",
                group="auto-adjust",
                dynamic="auto_adjust_authorized",
            ),
            state,
        )
        self.assertIn("fresh pre-action", reason or "")
        self.assertIn(
            "fresh pre-action",
            runner.maintenance_write_block_reason(
                runner.CommandSpec(
                    "autoadj start",
                    "start",
                    group="auto-adjust",
                    destructive=True,
                ),
                state,
            )
            or "",
        )

        idle = dict(running)
        idle["result"] = runner.RESULT_PASS
        idle["parsed"] = {"auto_adjust_running": False}
        runner.update_state(state, idle)
        self.assertIs(True, state["auto_adjust_fresh_idle_passed"])
        command, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "autoadj start",
                "start",
                group="auto-adjust",
                dynamic="auto_adjust_authorized",
            ),
            state,
        )
        self.assertEqual("autoadj start", command)
        self.assertIsNone(reason)

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

    def test_d9_change_is_target_only_and_breaks_unrelated_mutations(self) -> None:
        baseline = [0] * 256
        for target, address in (
            ("GLOBAL_INTERVAL", 0xC6),
            ("CO2_OFFSET", 0x58),
            ("BUS_ADDRESS", 0xC0),
        ):
            current = baseline.copy()
            current[address] = 1
            current[0xD9] = 1
            failures, reviews = runner.validate_parsed(
                runner.CommandSpec(
                    "reg dump 0 256",
                    "post-test",
                    validators=("custom_memory_target_only",),
                ),
                {"custom_memory": current, "custom_memory_complete": True},
                {
                    "baseline_custom_memory": baseline,
                    "expected_mutation_target": target,
                    "expected_mutation_first_address": address,
                    "expected_mutation_last_address": address,
                },
            )
            self.assertTrue(any("0xD9" in failure for failure in failures))
            self.assertEqual([], reviews)

    def test_definite_status_precedes_missing_success_text_and_contracts_remain(self) -> None:
        failure_text = "Status: TIMEOUT (code=4, detail=25000)\n"
        failure_spec = runner.CommandSpec(
            "read",
            "read",
            expected_any=("CO2 avg:",),
            validators=("status_ok",),
        )
        result, reason = runner.classify_response(
            failure_spec,
            failure_text,
            False,
            runner.parse_response("read", failure_text),
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("TIMEOUT", reason)

        no_validator = runner.CommandSpec(
            "read",
            "read",
            expected_any=("CO2 avg:",),
        )
        result, reason = runner.classify_response(
            no_validator,
            "Status: NACK (code=8, detail=0)\n",
            False,
            runner.parse_response(
                "read", "Status: NACK (code=8, detail=0)\n"
            ),
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("NACK", reason)

        mismatched_status = "Status: NACK (code=7, detail=0)\n"
        result, reason = runner.classify_response(
            no_validator,
            mismatched_status,
            False,
            runner.parse_response("read", mismatched_status),
        )
        self.assertEqual(runner.RESULT_FAIL, result)
        self.assertIn("expected 8", reason)

        accepted = (
            (
                runner.CommandSpec("filter", "optional", expected_any=("Status:",), validators=("status_optional",)),
                "Status: NOT_SUPPORTED (code=14, detail=0)\n",
            ),
            (
                runner.CommandSpec("addr 1", "address", expected_any=("Status:",), validators=("status_address_uncertain",)),
                "Status: PERSISTENT_STATE_UNCERTAIN (code=18, detail=1)\n",
            ),
            (
                runner.CommandSpec("autoadj start", "auto", expected_any=("Status:",), validators=("status_auto_adjust_start",)),
                "Status: PERSISTENT_STATE_UNCERTAIN (code=18, detail=217)\n",
            ),
            (
                runner.CommandSpec("read", "unplug", expected_any=("Status:",), validators=("expected_failure",)),
                "Status: NACK (code=8, detail=0)\n",
            ),
            (
                runner.CommandSpec("status", "stuck", expected_any=("Status:",), validators=("fault_bus_line",)),
                "Status: BUS_STUCK (code=11, detail=0)\n",
            ),
        )
        for spec, text in accepted:
            parsed = runner.parse_response(spec.command, text)
            result, reason = runner.classify_response(spec, text, False, parsed)
            self.assertEqual(runner.RESULT_PASS, result, reason)

        missing = runner.CommandSpec("read", "read", expected_any=("CO2 avg:",))
        result, _ = runner.classify_response(missing, "unparsed noise\n", False, {}, {})
        self.assertEqual(runner.RESULT_OPERATOR, result)

    def test_destructive_admission_requires_complete_clean_fresh_state(self) -> None:
        spec = runner.CommandSpec(
            "interval 160",
            "write",
            group="maintenance-interval",
            destructive=True,
        )
        complete = self.admitted_state(baseline_measurement_interval_ds=150)
        self.assertIsNone(runner.maintenance_write_block_reason(spec, complete))
        for key, value in (
            ("mutation_unresolved", None),
            ("mutation_unresolved", True),
            ("dirty_observation_epoch", -1),
            ("baseline_auto_adjust_preflight_complete", False),
            ("baseline_custom_memory_captured_utc", None),
        ):
            blocked = dict(complete)
            blocked[key] = value
            self.assertIsNotNone(runner.maintenance_write_block_reason(spec, blocked))

        mismatch = dict(complete)
        mismatch["baseline_device_address"] = 1
        self.assertIn(
            "does not match configured address",
            runner.maintenance_write_block_reason(spec, mismatch) or "",
        )
        for invalid_image in (None, [0] * 255, [0] * 255 + [256]):
            invalid = dict(complete)
            invalid["baseline_custom_memory"] = invalid_image
            self.assertIn(
                "baseline is missing or invalid",
                runner.maintenance_write_block_reason(spec, invalid) or "",
            )

    def test_configured_address_is_not_overwritten_by_candidate_state(self) -> None:
        state: dict[str, object] = {"configured_device_address": 0}
        runner.record_persistent_write_expectation(
            {
                "result": runner.RESULT_PASS,
                "destructive": True,
                "command": "addr 1",
                "group": "address-candidate",
            },
            state,
        )
        self.assertEqual(0, state["configured_device_address"])
        self.assertEqual(1, state["expected_device_address"])

    def test_capability_aware_auto_adjust_preflight(self) -> None:
        spec = runner.CommandSpec(
            "autoadj",
            "preflight",
            expected_any=("Status:",),
            validators=("status_optional", "auto_adjust_read", "auto_adjust_preflight"),
        )
        idle = "Status: OK (code=0, detail=0)\nAuto adjustment: idle\n"
        result, reason = runner.classify_response(
            spec,
            idle,
            False,
            runner.parse_response("autoadj", idle),
            {"special_features": 0x01},
        )
        self.assertEqual(runner.RESULT_PASS, result, reason)

        running = idle.replace("idle", "RUNNING")
        result, _ = runner.classify_response(
            spec,
            running,
            False,
            runner.parse_response("autoadj", running),
            {"special_features": 0x01},
        )
        self.assertEqual(runner.RESULT_FAIL, result)

        unsupported = "Status: NOT_SUPPORTED (code=14, detail=0)\n"
        result, reason = runner.classify_response(
            spec,
            unsupported,
            False,
            runner.parse_response("autoadj", unsupported),
            {"special_features": 0x00},
        )
        self.assertEqual(runner.RESULT_PASS, result, reason)

    def test_checked_sensor_error_without_detailed_capability_is_coherent_failure(self) -> None:
        text = """
Status: CO2_SENSOR_ERROR (code=17, detail=8)
Sample kind: FAST (MV3)
Value step: attempted=yes status=OK detail=0
CO2 value: 612 ppm, valid=no
Status step: attempted=yes status=OK detail=0 valid=yes, byte=0x08, co2Error=yes
Error-code step: attempted=no status=OK detail=0 valid=no
Sensor error: UNKNOWN (enum=255)
"""
        parsed = runner.parse_response("samplefast", text)
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("samplefast", "checked", validators=("checked_fast",)),
            parsed,
            {"operating_functions": 0x00},
        )
        self.assertEqual([], reviews)
        self.assertEqual(["sensor reported UNKNOWN (status=0x08)"], failures)
        self.assertFalse(any("transport" in item.lower() for item in failures))

    def test_checked_sensor_error_rejects_contradictory_step_evidence(self) -> None:
        advertised = """
Status: CO2_SENSOR_ERROR (code=17, detail=200)
Sample kind: FAST (MV3)
Value step: attempted=yes status=OK detail=0
CO2 value: 612 ppm, valid=no
Status step: attempted=yes status=OK detail=0 valid=yes, byte=0x08, co2Error=yes
Error-code step: attempted=yes status=OK detail=99 valid=yes, code=200 (sensor counts low)
Sensor error: SENSOR_COUNTS_LOW (enum=200)
"""
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "samplefast",
                "checked",
                validators=("checked_fast",),
            ),
            runner.parse_response("samplefast", advertised),
            {"operating_functions": 0x80},
        )
        self.assertTrue(any("error-code OK detail" in item for item in failures))
        self.assertEqual([], reviews)

        unadvertised = """
Status: CO2_SENSOR_ERROR (code=17, detail=8)
Sample kind: FAST (MV3)
Value step: attempted=yes status=OK detail=0
CO2 value: 612 ppm, valid=no
Status step: attempted=yes status=OK detail=0 valid=yes, byte=0x08, co2Error=yes
Error-code step: attempted=no status=TIMEOUT detail=99 valid=no
Sensor error: UNKNOWN (enum=255)
"""
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "samplefast",
                "checked",
                validators=("checked_fast",),
            ),
            runner.parse_response("samplefast", unadvertised),
            {"operating_functions": 0x00},
        )
        self.assertTrue(
            any("unattempted error-code step status" in item for item in failures)
        )
        self.assertEqual([], reviews)

    def test_stuck_line_validators_require_exact_selected_fault_and_bus_stuck(self) -> None:
        for validator, good in (
            ("levels_sda_low", {"scl_high": True, "sda_high": False}),
            ("levels_scl_low", {"scl_high": False, "sda_high": True}),
        ):
            failures, reviews = runner.validate_parsed(
                runner.CommandSpec("levels", "selected low", validators=(validator,)),
                good,
            )
            self.assertEqual([], failures)
            self.assertEqual([], reviews)
            failures, reviews = runner.validate_parsed(
                runner.CommandSpec("levels", "both low", validators=(validator,)),
                {"scl_high": False, "sda_high": False},
            )
            self.assertTrue(failures)
            self.assertEqual([], reviews)
        failures, _ = runner.validate_parsed(
            runner.CommandSpec("status", "fault", validators=("fault_bus_line",)),
            {"status": {"name": "TIMEOUT", "code": 4, "detail": 25000}},
        )
        self.assertTrue(any("TIMEOUT" in failure for failure in failures))

    def test_stuck_line_plans_capture_health_in_required_order(self) -> None:
        args = runner.parse_args(
            ["--dry-run", "--include-stuck-line", "--confirm-stuck-line"]
        )
        plan = runner.operator_fault_specs(args)
        for group in ("fault-sda-low", "fault-scl-low"):
            rows = [spec for spec in plan if spec.group == group]
            commands = [spec.command for spec in rows]
            self.assertEqual(
                [
                    "levels",
                    "drv",
                    f"operator: apply {group[6:9].upper()}-low jig",
                    "levels",
                    "buscheck",
                    "status",
                    "drv",
                    "libreset",
                    f"operator: release {group[6:9].upper()}-low jig",
                    "levels",
                    "recover",
                    "drv",
                ],
                commands,
            )

    def test_in_fault_health_requires_counter_movement_and_coherent_state(self) -> None:
        spec = runner.CommandSpec(
            "drv",
            "fault",
            group="fault-sda-low",
            validators=("health_faulted_since_pre",),
        )
        good = {
            "driver_state": "DEGRADED",
            "online": True,
            "consecutive_failures": 1,
            "total_failures": 11,
        }
        failures, reviews = runner.validate_parsed(
            spec,
            good,
            {
                "fault_pre_total_failures": 10,
                "fault_pre_group": "fault-sda-low",
            },
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)
        bad = dict(good, online=False, total_failures=10)
        failures, _ = runner.validate_parsed(
            spec,
            bad,
            {
                "fault_pre_total_failures": 10,
                "fault_pre_group": "fault-sda-low",
            },
        )
        self.assertGreaterEqual(len(failures), 2)

    def test_stuck_line_health_snapshot_is_scoped_to_each_fault_group(self) -> None:
        state: dict[str, object] = {}
        released_sda = runner.result_row(
            runner.CommandSpec(
                "levels",
                "Record idle levels before the SDA-low fault.",
                group="fault-sda-low",
            ),
            "levels",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {"scl_high": True, "sda_high": True},
        )
        runner.update_state(state, released_sda)
        pre_sda = runner.result_row(
            runner.CommandSpec(
                "drv",
                "Capture health before the SDA-low fault.",
                group="fault-sda-low",
            ),
            "drv",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {"total_failures": 10},
        )
        runner.update_state(state, pre_sda)
        self.assertEqual("fault-sda-low", state["fault_pre_group"])
        command, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "operator: apply SDA-low jig",
                "apply",
                group="fault-sda-low",
                dynamic="fault_preflight_passed",
            ),
            state,
        )
        self.assertEqual("operator: apply SDA-low jig", command)
        self.assertIsNone(reason)

        released_scl = dict(released_sda)
        released_scl["group"] = "fault-scl-low"
        released_scl["description"] = "Record idle levels before the SCL-low fault."
        runner.update_state(state, released_scl)
        pre_scl = dict(pre_sda)
        pre_scl["group"] = "fault-scl-low"
        pre_scl["description"] = "Capture health before the SCL-low fault."
        pre_scl["result"] = runner.RESULT_FAIL
        pre_scl["parsed"] = {}
        runner.update_state(state, pre_scl)
        self.assertNotIn("fault_pre_total_failures", state)
        self.assertNotIn("fault_pre_group", state)
        _, reason = runner.resolve_dynamic_command(
            runner.CommandSpec(
                "operator: apply SCL-low jig",
                "apply",
                group="fault-scl-low",
                dynamic="fault_preflight_passed",
            ),
            state,
        )
        self.assertIn("did not pass for this fault group", reason or "")

        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "drv",
                "in fault",
                group="fault-scl-low",
                validators=("health_faulted_since_pre",),
            ),
            {
                "driver_state": "DEGRADED",
                "online": True,
                "consecutive_failures": 1,
                "total_failures": 11,
            },
            state,
        )
        self.assertEqual([], failures)
        self.assertTrue(any("does not belong" in item for item in reviews))

    def test_repeated_checkpoints_preserve_capture_time_bytes_and_typed_baseline(self) -> None:
        args = runner.parse_args(["--dry-run"])
        state = {
            "baseline_custom_memory": list(range(256)),
            "baseline_custom_memory_captured_utc": "2026-07-29T10:00:00Z",
            "baseline_typed_results": {
                "filter": {
                    "status": {"name": "NOT_SUPPORTED", "code": 14, "detail": 0},
                    "supported": False,
                    "value": {},
                }
            },
        }
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = pathlib.Path(tmp)
            meta = runner.metadata(args, log_dir, "", "branch", "abcdef")
            with mock.patch.object(
                runner,
                "iso_timestamp",
                side_effect=["2026-07-29T10:01:00Z", "2026-07-29T10:02:00Z"],
            ):
                runner.write_checkpoint(log_dir, meta, "", [], state)
                first = json.loads(
                    (log_dir / "custom_memory_baseline.json").read_text(encoding="utf-8")
                )
                runner.write_checkpoint(log_dir, meta, "", [], state)
                second = json.loads(
                    (log_dir / "custom_memory_baseline.json").read_text(encoding="utf-8")
                )
        self.assertEqual(first["captured_utc"], second["captured_utc"])
        self.assertEqual(first["bytes"], second["bytes"])
        self.assertEqual(first["typed_baselines"], second["typed_baselines"])
        self.assertNotEqual(first["updated_utc"], second["updated_utc"])

    def test_hazardous_metadata_rejects_blank_whitespace_and_placeholder(self) -> None:
        base = [
            "--port", "COM1",
            "--include-stuck-line",
            "--confirm-stuck-line",
            "--board", "board",
            "--target-name", "target",
            "--operator", "operator",
            "--sensor-id", "sensor",
            "--fixture-id", "fixture",
            "--electrical-authority", "authority",
        ]
        required = (
            "--board",
            "--target-name",
            "--operator",
            "--sensor-id",
            "--fixture-id",
            "--electrical-authority",
        )
        for flag in required:
            for value in ("", "   ", "UnSpEcIfIeD"):
                argv = list(base)
                index = argv.index(flag)
                argv[index + 1] = value
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        runner.parse_args(argv)

        power = [
            "--port", "COM1",
            "--include-power-cycle",
            "--confirm-power-cycle",
            "--board", "board",
            "--target-name", "target",
            "--operator", "operator",
            "--sensor-id", "sensor",
            "--fixture-id", "fixture",
            "--electrical-authority", "authority",
            "--power-procedure", "   ",
        ]
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args(power)

    def test_factor_zero_is_rejected_and_nonzero_boundaries_are_accepted(self) -> None:
        prefix = [
            "--dry-run",
            "--include-persistent-writes",
            "--confirm-persistent-writes",
            "--write-interval-factor",
        ]
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.parse_args([*prefix, "0"])
        for value in ("-128", "-1", "1", "127"):
            args = runner.parse_args([*prefix, value])
            self.assertEqual(int(value), args.write_interval_factor)
        reason = runner.maintenance_write_block_reason(
            runner.CommandSpec(
                "factor 1",
                "write",
                group="maintenance-factor",
                destructive=True,
            ),
            self.admitted_state(baseline_co2_interval_factor=0),
        )
        self.assertIn("nonzero typed restore range", reason or "")

    def test_safe_plans_are_non_destructive_and_filter_write_is_not_buildable(self) -> None:
        quick = runner.build_plan(runner.parse_args(["--dry-run"]))
        complete = runner.build_plan(
            runner.parse_args(["--dry-run", "--complete-safe"])
        )
        self.assertFalse(any(spec.destructive for spec in quick))
        self.assertFalse(any(spec.destructive for spec in complete))
        args = runner.parse_args(
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
            ]
        )
        plan = runner.maintenance_specs(args)
        self.assertFalse(
            any(
                spec.destructive and re.fullmatch(r"filter\s+.+", spec.command)
                for spec in plan
            )
        )

        plan_args = (
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
                "--maintenance-interval", "160",
                "--write-interval-factor", "1",
                "--write-operating-mode", "1",
                "--write-part-name-hex", "41" * 16,
            ],
            [
                "--dry-run",
                "--include-persistent-writes",
                "--confirm-persistent-writes",
                "--include-calibration-writes",
                "--confirm-calibration-writes",
                "--write-co2-offset", "1",
                "--write-co2-gain", "32768",
            ],
            [
                "--dry-run",
                "--include-address-change",
                "--candidate-address", "1",
                "--confirm-address-change",
                "--confirm-address-restore",
            ],
            [
                "--dry-run",
                "--include-auto-adjust",
                "--confirm-auto-adjust",
            ],
            ["--dry-run", "--include-unplug-replug"],
            ["--dry-run", "--include-stuck-line", "--confirm-stuck-line"],
            ["--dry-run", "--include-power-cycle", "--confirm-power-cycle"],
        )
        all_plans = [quick, complete]
        all_plans.extend(
            runner.build_plan(runner.parse_args(argv))
            for argv in plan_args
        )
        self.assertFalse(
            any(
                re.search(r"\breg\s+write\b", spec.command)
                for built in all_plans
                for spec in built
            )
        )
        self.assertNotRegex(
            MODULE_PATH.read_text(encoding="utf-8"),
            r"CommandSpec\(\s*[\"']reg\s+write\b",
        )

    def test_baseline_records_versions_typed_values_and_filter_read_only(self) -> None:
        specs = runner.baseline_specs("maintenance-baseline", "opt-in")
        commands = [spec.command for spec in specs]
        self.assertLess(commands.index("e2spec"), commands.index("reg dump 0 256"))
        self.assertIn("filter", commands)
        self.assertTrue(
            all(not (spec.destructive and spec.command.startswith("filter ")) for spec in specs)
        )
        required = {
            "partnamehex", "addr", "interval", "factor", "filter", "mode",
            "offset", "gain", "calpoints", "autoadj",
        }
        self.assertTrue(required.issubset(set(commands)))

        interval_spec = next(spec for spec in specs if spec.command == "interval")
        failures, reviews = runner.validate_parsed(
            interval_spec,
            {"status": {"name": "NOT_SUPPORTED", "code": 14, "detail": 0}},
            {"operating_functions": 0x10},
        )
        self.assertTrue(any("advertised interval" in item for item in failures))
        self.assertEqual([], reviews)

        state: dict[str, object] = {}
        unsupported = runner.result_row(
            next(spec for spec in specs if spec.command == "filter"),
            "filter",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {
                "status": {
                    "name": "NOT_SUPPORTED",
                    "code": 14,
                    "detail": 0,
                }
            },
        )
        runner.update_state(state, unsupported)
        self.assertFalse(
            state["baseline_typed_results"]["filter"]["supported"]  # type: ignore[index]
        )

        all_results: dict[str, object] = {}
        for command in sorted(required):
            spec = next(item for item in specs if item.command == command)
            row = runner.result_row(
                spec,
                command,
                runner.RESULT_PASS,
                "",
                0.1,
                "",
                "test",
                {
                    "status": {
                        "name": "NOT_SUPPORTED",
                        "code": 14,
                        "detail": 0,
                    }
                },
            )
            runner.update_state(all_results, row)
        self.assertEqual(
            required,
            set(all_results["baseline_typed_results"]),  # type: ignore[arg-type]
        )
        self.assertTrue(
            all(
                result["supported"] is False
                and result["status"]["name"] == "NOT_SUPPORTED"
                for result in all_results["baseline_typed_results"].values()  # type: ignore[union-attr]
            )
        )

    def test_reversible_row_restores_with_typed_owner_and_full_image_check(self) -> None:
        specs = runner.reversible_target_specs(
            group="maintenance-factor",
            target="CO2 interval factor",
            read_command="factor",
            write_command="factor 1",
            dynamic_test=None,
            dynamic_restore="factor_baseline",
            read_validators=("factor_read", "factor_expected"),
            count=1,
            opt_in="opt-in",
        )
        self.assertEqual("factor <recorded-baseline>", specs[4].command)
        self.assertTrue(specs[4].destructive)
        self.assertEqual("factor", specs[5].command)
        self.assertIn("mutation_verified", specs[6].validators)
        self.assertIn("custom_memory_restored", specs[7].validators)
        self.assertFalse(any("reg write" in spec.command for spec in specs))

    def test_successful_reversible_row_preserves_typed_end_to_end_evidence(self) -> None:
        baseline = [0] * 256
        baseline[0xCB] = 0xFF
        state = self.admitted_state(
            baseline_custom_memory=baseline,
            baseline_co2_interval_factor=-1,
        )

        test_write = runner.result_row(
            runner.CommandSpec(
                "factor 1",
                "test",
                group="maintenance-factor",
                destructive=True,
            ),
            "factor 1",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {"status": {"name": "OK", "code": 0, "detail": 0}},
        )
        runner.record_persistent_write_expectation(test_write, state)

        readback = runner.CommandSpec(
            "factor",
            "readback",
            group="maintenance-factor",
            validators=("status_ok", "factor_read", "factor_expected"),
        )
        failures, reviews = runner.validate_parsed(
            readback,
            {
                "status": {"name": "OK", "code": 0, "detail": 0},
                "co2_interval_factor": 1,
            },
            state,
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

        test_diagnostic = self.resolved_mutation(
            "CO2_INTERVAL_FACTOR", 1, 0xCB, 0xCB, 1
        )
        dirty_spec = runner.CommandSpec(
            "dirty",
            "diagnostic",
            group="maintenance-factor",
            validators=("dirty_clean", "dirty_state", "mutation_verified"),
        )
        failures, reviews = runner.validate_parsed(
            dirty_spec, test_diagnostic, state
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)
        runner.update_state(
            state,
            runner.result_row(
                dirty_spec,
                "dirty",
                runner.RESULT_PASS,
                "",
                0.1,
                "",
                "test",
                test_diagnostic,
            ),
        )

        post_test = baseline.copy()
        post_test[0xCB] = 1
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "reg dump 0 256",
                "post-test",
                validators=("custom_memory_complete", "custom_memory_target_only"),
            ),
            {
                "custom_memory": post_test,
                "custom_memory_complete": True,
            },
            state,
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

        restore = runner.CommandSpec(
            "factor <recorded-baseline>",
            "restore",
            group="maintenance-factor",
            destructive=True,
            dynamic="factor_baseline",
        )
        self.assertIsNone(runner.maintenance_write_block_reason(restore, state))
        command, reason = runner.resolve_dynamic_command(restore, state)
        self.assertEqual("factor -1", command)
        self.assertIsNone(reason)
        restore_row = runner.result_row(
            restore,
            command or "",
            runner.RESULT_PASS,
            "",
            0.1,
            "",
            "test",
            {"status": {"name": "OK", "code": 0, "detail": 0}},
        )
        runner.record_persistent_write_expectation(restore_row, state)

        failures, reviews = runner.validate_parsed(
            readback,
            {
                "status": {"name": "OK", "code": 0, "detail": 0},
                "co2_interval_factor": -1,
            },
            state,
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

        restored_diagnostic = self.resolved_mutation(
            "CO2_INTERVAL_FACTOR", 1, 0xCB, 0xCB, 0xFF
        )
        failures, reviews = runner.validate_parsed(
            dirty_spec, restored_diagnostic, state
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)
        runner.update_state(
            state,
            runner.result_row(
                dirty_spec,
                "dirty",
                runner.RESULT_PASS,
                "",
                0.1,
                "",
                "test",
                restored_diagnostic,
            ),
        )
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec(
                "reg dump 0 256",
                "final",
                validators=("custom_memory_complete", "custom_memory_restored"),
            ),
            {
                "custom_memory": baseline.copy(),
                "custom_memory_complete": True,
            },
            state,
        )
        self.assertEqual([], failures)
        self.assertEqual([], reviews)

    def test_auto_adjust_plan_has_no_restore_retry_or_automatic_resync(self) -> None:
        args = runner.parse_args(
            ["--dry-run", "--include-auto-adjust", "--confirm-auto-adjust"]
        )
        commands = [spec.command for spec in runner.auto_adjust_specs(args)]
        self.assertEqual(1, commands.count("autoadj start"))
        self.assertNotIn("resync", commands)
        self.assertFalse(any("<recorded-baseline>" in command for command in commands))

    def test_complete_safe_captures_features_before_checked_samples(self) -> None:
        commands = [spec.command for spec in runner.extended_specs(1, 1)]
        self.assertLess(commands.index("features"), commands.index("samplefast"))
        self.assertLess(commands.index("caps"), commands.index("samplefast"))

    def test_clean_dirty_validator_rejects_contradictory_resolved_contract(self) -> None:
        parsed = {
            "persistent_config_dirty": False,
            "resync_needed": False,
            "mutation_unresolved": False,
            "persistent_config_dirty_error": {
                "name": "VERIFY_MISMATCH",
                "code": 15,
                "detail": 1,
            },
        }
        failures, reviews = runner.validate_parsed(
            runner.CommandSpec("dirty", "dirty", validators=("dirty_clean",)),
            parsed,
        )
        self.assertTrue(any("non-OK persistent dirty error" in item for item in failures))
        self.assertEqual([], reviews)


if __name__ == "__main__":
    unittest.main()
