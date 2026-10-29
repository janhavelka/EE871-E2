from __future__ import annotations

import contextlib
import io
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_cli_contract as arduino
import check_idf_example_contract as idf
import ee871_hil_runner as runner


class CliContractTests(unittest.TestCase):
    def check_mutation(self, old: str, new: str, expected: str) -> None:
        for checker, relative in (
            (arduino, "examples/common/E2Diagnostics.h"),
            (idf, "examples/idf/basic_bringup/main/main.cpp"),
        ):
            with self.subTest(checker=checker.__name__):
                target = ROOT / relative
                original_read = pathlib.Path.read_text
                source = target.read_text(encoding="utf-8")
                self.assertIn(old, source)
                mutated = source.replace(old, new, 1)

                def read(path: pathlib.Path, *args, **kwargs) -> str:
                    if path == target:
                        return mutated
                    return original_read(path, *args, **kwargs)

                output = io.StringIO()
                with mock.patch.object(pathlib.Path, "read_text", read), contextlib.redirect_stdout(output):
                    with self.assertRaises(SystemExit) as raised:
                        checker.main()
                self.assertEqual(1, raised.exception.code)
                self.assertIn(expected, output.getvalue())

    def test_current_contracts_pass(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(0, arduino.main())
            self.assertEqual(0, idf.main())

    def test_checked_sample_commands_remain_manual(self) -> None:
        for options in ([], ["--include-niche"], ["--include-extended"],
                        ["--include-niche", "--include-extended"]):
            with self.subTest(options=options):
                args = runner.parse_args(["--dry-run", *options])
                commands = {spec.command for spec in runner.build_plan(args)}
                self.assertTrue(commands.isdisjoint({"co2fastsample", "co2avgsample"}))

    def test_persistent_cli_contract_rejects_unguarded_mutations_and_bus_calls(self) -> None:
        methods = (
            "customWrite", "writePartName", "writeBusAddress", "writeMeasurementInterval",
            "writeCo2IntervalFactor", "writeCo2Filter", "writeOperatingMode", "writeCo2Offset",
            "writeCo2Gain", "startAutoAdjust",
        )
        for relative, end in (
            ("examples/01_basic_bringup_cli/main.cpp", "void setup"),
            ("examples/idf/basic_bringup/main/main.cpp", "void configureConsoleInput"),
        ):
            source = (ROOT / relative).read_text(encoding="utf-8")
            command = arduino.extract_section(source, "void processCommand", end)
            for method in methods:
                with self.subTest(source=relative, method=method):
                    position = command.index(f"auto st = device.{method}(")
                    guard = command.rfind("if (!allowMaintenanceWrite())", 0, position)
                    self.assertGreaterEqual(guard, 0)
                    mutated = command[:guard] + command[guard:].replace(
                        "if (!allowMaintenanceWrite())", "if (false)", 1
                    )
                    errors = arduino.persistent_cli_errors(source, mutated)
                    self.assertTrue(any(f"every {method} call" in error for error in errors), errors)
            for bus_call in ("probe()", "recover()", "resyncPersistentConfig()", "customWrite(0, 0)"):
                with self.subTest(source=relative, bus_call=bus_call):
                    mutated = source.replace(
                        "void printPersistentOperation() {",
                        f"void printPersistentOperation() {{ device.{bus_call};", 1,
                    )
                    self.assertIn(
                        "operation diagnostics and admission must use only cache accessors",
                        arduino.persistent_cli_errors(mutated, command),
                    )

    def test_persistent_operation_cli_preserves_evidence_and_gates_without_io(self) -> None:
        # Compile the actual shipped helpers with a fake exposing only cache
        # accessors. A hidden read, write, recovery or resync cannot compile.
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "Native persistent CLI regression requires g++")
        source = r'''
#include "EE871/EE871.h"
#include <cassert>
#include <cstdarg>
#include <cstdio>
struct FakeSerial {
  void printf(const char* format, ...) {
    va_list args; va_start(args, format); std::vprintf(format, args); va_end(args);
  }
} Serial;
struct FakeDriver {
  EE871::PersistentOperationResult operation;
  bool dirty = false;
  mutable unsigned operationReads = 0, dirtyReads = 0;
  EE871::PersistentOperationResult getLastPersistentOperation() const {
    ++operationReads; return operation;
  }
  bool persistentConfigDirty() const { ++dirtyReads; return dirty; }
};
unsigned statusPrints = 0;
EE871::Status lastStatus;
const char* errToStr(EE871::Err code) {
  switch (code) {
    case EE871::Err::OK: return "OK";
    case EE871::Err::NACK: return "NACK";
    case EE871::Err::TIMEOUT: return "TIMEOUT";
    case EE871::Err::BUSY: return "BUSY";
    case EE871::Err::OUT_OF_RANGE: return "OUT_OF_RANGE";
    case EE871::Err::E2_ERROR: return "E2_ERROR";
    default: return "OTHER";
  }
}
void printStatus(const EE871::Status& status) { ++statusPrints; lastStatus = status; }
'''
        for name, relative in (
            ("arduino", "examples/01_basic_bringup_cli/main.cpp"),
            ("idf", "examples/idf/basic_bringup/main/main.cpp"),
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            helpers = arduino.extract_section(text, "const char* persistentOperationName", "bool ensureProbeOk")
            source += f"namespace {name} {{ FakeDriver device;\n" + helpers + "\n}\n"
        source += r'''
int main() {
  using EE871::Err;
  using EE871::Status;
  using EE871::PersistentOperation;
  FakeDriver* drivers[] = {&arduino::device, &idf::device};
  void (*formatters[])() = {arduino::printPersistentOperation, idf::printPersistentOperation};
  bool (*gates[])() = {arduino::allowMaintenanceWrite, idf::allowMaintenanceWrite};
  const bool expectedAllowed[] = {true, true, true, false, false, false, true,
                                 false, false, false, true, false, false, true};
  for (unsigned implementation = 0; implementation < 2; ++implementation) {
    for (unsigned scenario = 0; scenario < 14; ++scenario) {
      auto& device = *drivers[implementation];
      device = FakeDriver{};
      auto& operation = device.operation;
      if (scenario != 0) {
        operation.operation = PersistentOperation::MEASUREMENT_INTERVAL;
        operation.startAddress = 0xC6;
        operation.length = 2;
        operation.requested[0] = 0xBC; operation.requested[1] = 0x02;
        operation.requestedValidMask = 3;
      }
      if (scenario == 1) operation.result = Status::Error(Err::OUT_OF_RANGE, "rejected", 150);
      if (scenario == 2) {
        operation.attemptedMask = 1;
        operation.result = Status::Error(Err::NACK, "control NACK", 77);
      }
      if (scenario == 3) {
        operation.attemptedMask = operation.possibleEffectMask = 1;
        operation.result = Status::Error(Err::TIMEOUT, "PEC timeout", 88);
      }
      if (scenario >= 4) {
        operation.attemptedMask = operation.possibleEffectMask = operation.acknowledgedMask = 3;
      }
      if (scenario >= 5) {
        operation.observed[0] = 0xBC; operation.observed[1] = 0;
        operation.observedValidMask = 3; operation.matchedMask = 1;
        operation.result = Status::Error(Err::E2_ERROR, "mismatch", 99);
      }
      if (scenario >= 6) {
        operation.observed[1] = 2; operation.matchedMask = 3;
        operation.result = Status::Ok();
      }
      device.dirty = scenario == 7;
      if (scenario >= 8 && scenario <= 11) {
        operation.operation = scenario == 8 ? PersistentOperation::BUS_ADDRESS :
                              scenario == 9 ? PersistentOperation::CUSTOM_WRITE :
                              PersistentOperation::AUTO_ADJUST;
        operation.startAddress = scenario <= 9 ? 0xC0 : 0xD9;
        operation.length = 1;
        operation.requested[0] = operation.observed[0] = 1;
        operation.requestedValidMask = operation.attemptedMask = operation.possibleEffectMask = 1;
        operation.acknowledgedMask = operation.observedValidMask = operation.matchedMask = 1;
        if (scenario == 11) {
          operation.acknowledgedMask = operation.observedValidMask = operation.matchedMask = 0;
          operation.result = Status::Error(Err::TIMEOUT, "uncertain start", 100);
        }
      }
      if (scenario == 12) {
        operation.operation = PersistentOperation::PART_NAME;
        operation.startAddress = 0xB0; operation.length = 16;
        for (unsigned i = 0; i < 16; ++i) {
          operation.requested[i] = static_cast<uint8_t>(0x40 + i);
          operation.observed[i] = static_cast<uint8_t>(0x40 + i);
        }
        operation.requestedValidMask = 0xFFFF; operation.attemptedMask = 0x3F;
        operation.possibleEffectMask = operation.acknowledgedMask = 0x1F;
        operation.observedValidMask = operation.matchedMask = 7;
        operation.result = Status::Error(Err::NACK, "part name interrupted", 101);
      }
      if (scenario == 13) {
        operation.operation = PersistentOperation::CUSTOM_WRITE;
        operation.startAddress = 0xC1;
      }
      std::printf("CASE %u:%u\n", implementation, scenario);
      statusPrints = 0;
      formatters[implementation]();
      assert(statusPrints == 0);
      assert(device.operationReads == 1 && device.dirtyReads == 1);
      const bool allowed = gates[implementation]();
      assert(allowed == expectedAllowed[scenario]);
      assert(statusPrints == (allowed ? 0U : 1U));
      assert(allowed || lastStatus.code == Err::BUSY);
      assert(device.operationReads == 2 && device.dirtyReads == 2);
      assert(operation.result.detail == device.getLastPersistentOperation().result.detail);
    }
  }
}
'''
        with tempfile.TemporaryDirectory(prefix="ee871-persistent-cli-") as directory:
            cpp = pathlib.Path(directory) / "persistent.cpp"
            executable = pathlib.Path(directory) / "persistent.exe"
            cpp.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++17", "-I", str(ROOT / "include"), str(cpp), "-o", str(executable)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(0, build.returncode, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(0, run.returncode, run.stdout + run.stderr)
        captures = re.split(r"CASE \d:\d+\n", run.stdout)[1:]
        self.assertEqual(28, len(captures))
        for scenario in range(14):
            self.assertEqual(captures[scenario], captures[14 + scenario])
            self.assertEqual([], runner.parse_status(captures[scenario]).get("statuses", []))
        self.assertIn("Target: <none>", captures[0])
        self.assertIn("Effect: NONE", captures[2])
        self.assertIn("Effect: POSSIBLE", captures[3])
        self.assertIn("Operation result: TIMEOUT (code=", captures[3])
        self.assertIn("detail=88)", captures[3])
        self.assertIn("Operation result message: PEC timeout", captures[3])
        self.assertIn("Observed bytes: -- --", captures[3])
        self.assertIn("Effect: ACKNOWLEDGED", captures[4])
        self.assertIn("Requested bytes: BC 02", captures[5])
        self.assertIn("Observed bytes: BC 00", captures[5])
        self.assertIn("Matched mask: 0x0001", captures[5])
        self.assertIn("Maintenance admission: blocked", captures[5])
        self.assertIn("Effect: VERIFIED", captures[6])
        self.assertIn("Maintenance admission: open", captures[6])
        self.assertIn("Maintenance admission: blocked", captures[7])
        for scenario in (8, 9):
            self.assertIn("Effect: VERIFIED", captures[scenario])
            self.assertIn("Maintenance admission: blocked", captures[scenario])
            self.assertIn("does not establish active address", captures[scenario])
        self.assertIn("Maintenance admission: open", captures[10])
        self.assertIn("does not establish calibration completion or success", captures[10])
        self.assertIn("Maintenance admission: blocked", captures[11])
        self.assertIn("Target: 0xB0..0xBF (length=16)", captures[12])
        self.assertIn("Requested bytes: " + " ".join(f"{value:02X}" for value in range(0x40, 0x50)), captures[12])
        self.assertIn("Observed bytes: 40 41 42" + " --" * 13, captures[12])
        self.assertIn("Maintenance admission: open", captures[13])

    def test_checked_sample_formatters_preserve_evidence_and_overall_status(self) -> None:
        # Execute the shipped formatters with fixed driver evidence. This checks
        # both native output parity and compatibility with the runner's generic
        # Status parser without adding side-effecting commands to a live plan.
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "Native sample formatter regression requires g++")
        source = '''
#include "EE871/EE871.h"
#include <cassert>
#include <cstdarg>
#include <cstdio>
struct FakeSerial {
  void printf(const char* format, ...) {
    va_list args; va_start(args, format); std::vprintf(format, args); va_end(args);
  }
} Serial;
struct FakeDriver {
  EE871::Co2ReadResult sample;
  EE871::Status result;
  unsigned fastReads = 0, averageReads = 0;
  EE871::Status readCo2FastSample(EE871::Co2ReadResult& out) {
    ++fastReads; out = sample; return result;
  }
  EE871::Status readCo2AverageSample(EE871::Co2ReadResult& out) {
    ++averageReads; out = sample; return result;
  }
};
const char* errToStr(EE871::Err code) {
  switch (code) {
    case EE871::Err::OK: return "OK";
    case EE871::Err::NACK: return "NACK";
    case EE871::Err::CO2_SENSOR_ERROR: return "CO2_SENSOR_ERROR";
    default: return "OTHER";
  }
}
void printStatus(const EE871::Status& st) {
  std::printf("  Status: %s (code=%u, detail=%ld)\\n", errToStr(st.code),
              static_cast<unsigned>(st.code), static_cast<long>(st.detail));
}
'''
        for name, relative in (
            ("arduino", "examples/01_basic_bringup_cli/main.cpp"),
            ("idf", "examples/idf/basic_bringup/main/main.cpp"),
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            formatter = arduino.extract_section(text, "void printSampleStep", "void printPersistentDirtyFields")
            source += f"namespace {name} {{ FakeDriver device;\n" + formatter + "\n}\n"
        source += '''
int main() {
  using EE871::Err;
  using EE871::Status;
  FakeDriver* drivers[] = {&arduino::device, &idf::device};
  void (*formatters[])(bool) = {arduino::printCo2Sample, idf::printCo2Sample};
  for (unsigned implementation = 0; implementation < 2; ++implementation) {
    auto& device = *drivers[implementation];
    for (unsigned scenario = 0; scenario < 4; ++scenario) {
      device.sample = EE871::Co2ReadResult{};
      auto& sample = device.sample;
      sample.valueReadAttempted = true;
      sample.valueValid = scenario != 3;
      sample.ppm = sample.valueValid ? 65535 : 0;
      sample.statusReadAttempted = sample.statusValid = sample.valueValid;
      sample.ppmValid = scenario == 0;
      sample.co2Error = scenario == 1 || scenario == 2;
      sample.statusByte = sample.co2Error ? 8 : 0;
      sample.errorCodeReadAttempted = sample.co2Error;
      sample.errorCodeValid = scenario == 1;
      sample.errorCode = sample.errorCodeValid ? 255 : 0;
      device.result = Status::Ok();
      if (scenario == 1) device.result = Status::Error(Err::CO2_SENSOR_ERROR, "sensor", 255);
      if (scenario == 2) {
        sample.errorCodeReadStatus = Status::Error(Err::NACK, "detail read failed", 77);
        device.result = sample.errorCodeReadStatus;
      }
      if (scenario == 3) {
        sample.valueReadStatus = Status::Error(Err::NACK, "value read failed", 88);
        device.result = sample.valueReadStatus;
      }
      std::printf("CASE %u:%u\\n", implementation, scenario);
      formatters[implementation](scenario % 2 == 0);
    }
    assert(device.fastReads == 2 && device.averageReads == 2);
  }
}
'''
        with tempfile.TemporaryDirectory(prefix="ee871-checked-cli-") as directory:
            cpp = pathlib.Path(directory) / "formatter.cpp"
            executable = pathlib.Path(directory) / "formatter.exe"
            cpp.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++17", "-I", str(ROOT / "include"), str(cpp), "-o", str(executable)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(0, build.returncode, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(0, run.returncode, run.stdout + run.stderr)
        captures = re.split(r"CASE \d:\d\n", run.stdout)[1:]
        self.assertEqual(8, len(captures))
        for scenario in range(4):
            self.assertEqual(captures[scenario], captures[4 + scenario])
        expected = (("OK", 0), ("CO2_SENSOR_ERROR", 255), ("NACK", 77), ("NACK", 88))
        for scenario, capture in enumerate(captures[:4]):
            parsed = runner.parse_status(capture)
            self.assertEqual(1, len(parsed["statuses"]))
            self.assertEqual(expected[scenario], (parsed["status"]["name"], parsed["status"]["detail"]))
            self.assertIn("Checked sample valid: " + ("yes" if scenario == 0 else "no"), capture)
            if scenario == 3:
                self.assertNotIn("Raw CO2:", capture)
                self.assertIn("Status read: attempted=no, valid=no", capture)
                self.assertNotIn("Status read result:", capture)
            else:
                self.assertIn("Raw CO2: 65535 ppm", capture)
            if scenario == 0 or scenario == 3:
                self.assertIn("Error code read: attempted=no, valid=no", capture)
                self.assertNotIn("Error code read result:", capture)
            if scenario == 1:
                self.assertIn("Error code: 255", capture)
            if scenario == 2:
                self.assertIn("CO2 error: yes", capture)
                self.assertIn("Error code read: attempted=yes, valid=no", capture)
                self.assertIn("detail read failed", capture)
                self.assertNotIn("  Error code:", capture)

    def test_scanner_cannot_demote_incompatible_identity(self) -> None:
        self.check_mutation(
            "lastError[addr].code != EE871::Err::NOT_SUPPORTED",
            "true", "never demote NOT_SUPPORTED",
        )

    def test_scanner_error_retention_native(self) -> None:
        # Compile the actual decision block from each example so the runtime
        # regression tests the shipped condition, not a copied implementation.
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "Native scanner regression requires g++")
        functions = []
        for name, relative in (
            ("arduino", "examples/common/E2Diagnostics.h"),
            ("idf", "examples/idf/basic_bringup/main/main.cpp"),
        ):
            source = arduino.strip_cpp_non_code((ROOT / relative).read_text(encoding="utf-8"))
            scanner = arduino.extract_section(source, "void scanAddresses", "struct TimingResult")
            block = re.search(r"if\s*\(\s*lastError\[addr\].*?lastError\[addr\]\s*=\s*st;\s*\}", scanner, re.S)
            self.assertIsNotNone(block)
            functions.append(
                f"EE871::Status {name}(EE871::Status previous, EE871::Status st) {{\n"
                "EE871::Status lastError[1] = {previous}; const int addr = 0;\n"
                + block.group() + "\nreturn lastError[0];\n}"
            )
        source = '#include "EE871/Status.h"\n#include <cassert>\n' + "\n".join(functions)
        source += '''
int main() {
  using EE871::Err;
  using EE871::Status;
  const auto incompatible = Status::Error(Err::NOT_SUPPORTED, "wrong subgroup", 8);
  const auto nack = Status::Error(Err::NACK, "absent");
  const Err laterErrors[] = {Err::BUS_STUCK, Err::TIMEOUT, Err::PEC_MISMATCH, Err::NACK};
  using Retain = Status (*)(Status, Status);
  const Retain implementations[] = {arduino, idf};
  for (Retain retain : implementations) {
    for (Err code : laterErrors) {
      auto result = retain(incompatible, Status::Error(code, "later fault", 99));
      assert(result.code == Err::NOT_SUPPORTED && result.detail == 8);
      assert(result.msg == incompatible.msg);
    }
    assert(retain(Status::Ok(), nack).code == Err::NACK);
    assert(retain(nack, incompatible).code == Err::NOT_SUPPORTED);
    const auto timeout = Status::Error(Err::TIMEOUT, "stretch");
    assert(retain(nack, timeout).code == Err::TIMEOUT);
    assert(retain(timeout, nack).code == Err::TIMEOUT);
    assert(retain(nack, Status::Ok()).ok());
  }
}
'''
        with tempfile.TemporaryDirectory(prefix="ee871-scanner-") as directory:
            cpp = pathlib.Path(directory) / "scanner.cpp"
            executable = pathlib.Path(directory) / "scanner.exe"
            cpp.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++17", "-I", str(ROOT / "include"), str(cpp), "-o", str(executable)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(0, build.returncode, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(0, run.returncode, run.stdout + run.stderr)

    def test_scanner_cannot_count_ack_only_responses(self) -> None:
        for increment in ("found ++;", "++ found;"):
            self.check_mutation("candidate.end();", f"candidate.end(); {increment}", "ACK-only")

    def test_libtest_requires_production_read(self) -> None:
        self.check_mutation("driver.readControlByte(", "readByteRaw(", "production control-byte path")

    def test_libtest_rejects_raw_traffic_alongside_production_read(self) -> None:
        self.check_mutation(
            "driver.readControlByte(tests[i].mainCmd, data);",
            "driver.readControlByte(tests[i].mainCmd, data); sendStart(cfg);",
            "raw diagnostic call",
        )

    def test_libtest_explains_health_effects(self) -> None:
        self.check_mutation(
            "Reads are tracked; OFFLINE returns the latched status without bus traffic.",
            "", "explain tracked reads",
        )

    def test_timing_rejects_old_candidates_and_commented_correct_list(self) -> None:
        self.check_mutation(
            "{995, 500, 250, 200, 150, 100}",
            "{1000, 500, 250, 200, 150, 100, 75, 50}; // timings[] = {995, 500, 250, 200, 150, 100}",
            "six in-spec candidates",
        )

    def test_timing_requires_data_setup_in_frequency(self) -> None:
        self.check_mutation(
            "1000000.0f / (10.0f + 2.0f * clockUs)",
            "1000000.0f / (2.0f * clockUs)", "10 us data setup",
        )


if __name__ == "__main__":
    unittest.main()
